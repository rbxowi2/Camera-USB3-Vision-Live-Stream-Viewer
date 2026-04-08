"""
u3v_viewer_v2.py — Hikvision Industrial Camera USB3 Vision Live Stream Viewer

Architecture Overview
─────────────────────────────────────────────────────────
  CameraThread    Background capture thread: grabs frames → Ring Buffer / Ping-Pong / Buffer
  DiskWriteThread Disk thread: waits for signal, writes frames from Ping-Pong Buffer
  LiveViewer      Main window: display, parameter adjustment, recording controls

Memory Strategy
─────────────────────────────────────────────────────────
  Ring Buffer             — Allocated by resolution after camera opens; continuously
  (limit: RING_MAX_BYTES)   overwritten; used for live display.
  Ping-Pong (2 frames)    — Normal recording; two slots alternate so the capture
                            thread is never blocked.
  Buffered recording      — Fills RAM up to BUF_MAX_BYTES then stops; frames are
  (limit: BUF_MAX_BYTES)    flushed to disk in the background. Ring Buffer is
                            released during this phase to avoid excessive memory usage.

Parameter Ranges
─────────────────────────────────────────────────────────
  Exposure, gain, and FPS slider ranges are read from hardware after the camera
  opens. If a query fails, the FALLBACK_* safe defaults are used instead.

Hardware Timestamps
─────────────────────────────────────────────────────────
  Each frame's camera-internal clock value (ns) is read via buf.get_timestamp()
  before push_buffer() returns the buffer to Aravis.
  Filename format: frame_000000_HH_MM_SS_mmm.bmp/.jpg  (relative to recording start)

Save Format (v2)
─────────────────────────────────────────────────────────
  A format selector (BMP / JPG) has been added to the sidebar.
  The selection affects photos, normal recording, and buffered recording output.
  JPG quality is fixed at JPEG_QUALITY (default 85).

Hotkeys: s = snapshot  /  r = record  /  q = quit

Porting to Other Camera Brands
─────────────────────────────────────────────────────────
This program was developed and tested with Hikvision USB3 Vision cameras.
When using a different brand, the following four areas may need adjustment:

1. udev rule  (/etc/udev/rules.d/99-hikvision.rules)
     ATTRS{idVendor}=="2bdf" is Hikvision's USB VID. Replace it with your
     camera's VID, or use the generic USB3 Vision rule:
       SUBSYSTEM=="usb", ATTR{bDeviceClass}=="ef", MODE="0666", GROUP="plugdev"
     Find the VID with: lsusb | grep <brand keyword>

2. GenICam node names
     Affected locations: CameraThread.open(), run(), _apply_pending(), _query_bounds()
     Common differences:
       ExposureTime         → ExposureTimeAbs / ExposureTimeRaw
       Gain                 → AnalogGain / GainRaw
       AcquisitionFrameRate → AcquisitionFrameRateAbs
     List supported nodes with: arv-tool-0.8 --name <camera name> features

3. Frame rate enable flag
     Affected locations: CameraThread.open(), run()
     Some brands (Basler, FLIR, etc.) require an enable flag before the frame
     rate can be written:
       self.cam.set_boolean("AcquisitionFrameRateEnable", True)
     Add this line before the _cam_set("AcquisitionFrameRate", ...) call.

4. Pixel format
     Affected location: CameraThread._decode_frame()
     Currently handled: MONO_8 / MONO_16 / BAYER_RG_8 / BAYER_GB_8
     If the camera outputs a different format (BAYER_BG_8, BAYER_GR_8, RGB8,
     YUV422, etc.), add the corresponding if-branch and cv2 colour conversion
     inside _decode_frame().
"""

import math
import os
import sys
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np

try:
    from PIL import Image, ImageTk
except ImportError:
    print("Pillow is not installed. Run: pip install Pillow")
    sys.exit(1)

import gi
gi.require_version("Aravis", "0.8")
from gi.repository import Aravis

# ── Startup defaults ───────────────────────────────────────────────────────────
DEFAULT_EXPOSURE = 30000.0   # µs
DEFAULT_GAIN     = 0.0       # dB
DEFAULT_FPS      = 200.0     # fps

# ── Fallback ranges used when hardware query fails ─────────────────────────────
FALLBACK_EXP_MIN  = 100.0
FALLBACK_EXP_MAX  = 200_000.0
FALLBACK_GAIN_MIN = 0.0
FALLBACK_GAIN_MAX = 15.0
FALLBACK_FPS_MIN  = 1.0
FALLBACK_FPS_MAX  = 1000.0

# ── Memory limits ──────────────────────────────────────────────────────────────
RING_MAX_BYTES = 4 * 1024 ** 3   # Ring Buffer memory cap (adjust to suit available RAM)
BUF_MAX_BYTES  = 4 * 1024 ** 3   # Buffered recording memory cap (adjust to suit available RAM)

# ── JPEG quality ──────────────────────────────────────────────────────────────
JPEG_QUALITY = 85

# ── Display window ─────────────────────────────────────────────────────────────
DISPLAY_W = 860   # Image canvas width (px)
SIDEBAR_W = 340   # Sidebar width (px)

# ── Default camera resolution (used for layout before a camera connects) ───────
DEFAULT_CAM_W = 1280
DEFAULT_CAM_H = 1024

# ── Ring Buffer frame count cap ────────────────────────────────────────────────
# Actual frame count = min(RING_MAX_BYTES // bytes_per_frame, RING_MAX_FRAMES)
# At low resolutions, RING_MAX_BYTES alone could allow tens of thousands of
# frames. A single np.zeros allocation that large slows startup and wastes RAM,
# so RING_MAX_FRAMES provides a hard upper bound independent of resolution.
RING_MAX_FRAMES = 5000

# ── Capture thread ─────────────────────────────────────────────────────────────
# Aravis uses a producer/consumer model: the camera writes frames into empty
# buffers and places them in a completion queue. The program pops completed
# buffers, processes them, then returns them with push_buffer().
# STREAM_PREBUF is the number of empty buffers pushed before acquisition starts.
# Too few buffers will cause frame drops at high frame rates.
STREAM_PREBUF     = 10          # Number of pre-allocated stream buffers
BUF_TIMEOUT_US    = 1_000_000  # timeout_pop_buffer timeout (µs)
FPS_SAMPLE_FRAMES = 30         # Sliding-window size for capture FPS calculation

# ── UI refresh ─────────────────────────────────────────────────────────────────
DISPLAY_REFRESH_MS = 16   # Display update interval (ms, ~60 Hz)

# ── Thread shutdown timeouts ───────────────────────────────────────────────────
CAM_JOIN_TIMEOUT  = 2   # Max wait for camera thread to join (seconds)
DISK_JOIN_TIMEOUT = 3   # Max wait for disk thread to join (seconds)

# ── Buffered recording save parallelism ────────────────────────────────────────
# After buffered recording stops, frames are written to disk via a
# ThreadPoolExecutor. cv2.imwrite releases the GIL, so multiple threads
# genuinely run in parallel on multi-core systems.
# BMP: almost no CPU work — bottleneck is disk bandwidth; extra workers help
#      only on SSDs.
# JPG: compression is CPU-bound — workers close to physical core count give the
#      best speedup.
# Default None = Python auto-detects (typically equals CPU thread count).
# Set to a specific integer to pin the worker count to physical cores.
BUF_SAVE_WORKERS = None

# ── Paths ──────────────────────────────────────────────────────────────────────
ROOT_DIR    = os.path.dirname(os.path.abspath(__file__))
CAPTURE_DIR = os.path.join(ROOT_DIR, "captures")
os.makedirs(CAPTURE_DIR, exist_ok=True)


# ─────────────────────────────────────────────────────────────────────────────
# Utility functions
# ─────────────────────────────────────────────────────────────────────────────

def disk_free_gb(path: str) -> float:
    """Return free disk space (GB) on the volume containing *path*."""
    try:
        st = os.statvfs(path)
        return st.f_bavail * st.f_frsize / 1024 ** 3
    except Exception:
        return 0.0


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def elapsed_str(start_ns: int, ts_ns: int) -> str:
    """Convert a hardware-timestamp delta to an HH_MM_SS_mmm string
    relative to *start_ns*."""
    ms  = (ts_ns - start_ns) // 1_000_000
    h   = ms // 3_600_000
    m   = (ms % 3_600_000) // 60_000
    s   = (ms % 60_000) // 1000
    mms = ms % 1000
    return f"{h:02d}_{m:02d}_{s:02d}_{mms:03d}"


def imwrite_fmt(path: str, frame: np.ndarray, fmt: str):
    """Write *frame* to *path* using the specified format ('BMP' or 'JPG')."""
    if fmt == "JPG":
        cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    else:
        cv2.imwrite(path, frame)


# ─────────────────────────────────────────────────────────────────────────────
# Disk write thread
# ─────────────────────────────────────────────────────────────────────────────

class DiskWriteThread(threading.Thread):
    """
    Ping-Pong Buffer → disk writer.

    The camera thread calls signal(slot) and returns immediately; this thread
    finishes the write in the background. It lags at most one frame behind the
    capture loop without blocking it.

    ts_buf shares its index with buf: ts_buf[slot] holds the hardware timestamp
    (ns) for the corresponding frame. The first frame's timestamp becomes the
    reference point; subsequent filenames encode the relative time offset.
    fmt selects the output format ('BMP' or 'JPG').
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.buf: np.ndarray = None   # shape (2, h, w, 3), set by LiveViewer
        self.ts_buf: list    = None   # [int, int], set by LiveViewer
        self.save_dir: str   = ""
        self.fmt: str        = "BMP"  # 'BMP' or 'JPG'
        self.running         = False
        self._count          = 0
        self._slot           = 0
        self._start_ts: int  = 0      # first-frame timestamp used as time origin
        self._event          = threading.Event()

    @property
    def count(self) -> int:
        return self._count

    def signal(self, slot: int):
        """Called by the camera thread to indicate that *slot* has a new frame
        (ts_buf[slot] has already been updated)."""
        self._slot = slot
        self._event.set()

    def run(self):
        ext = ".jpg" if self.fmt == "JPG" else ".bmp"
        self.running = True
        while self.running:
            if self._event.wait(timeout=0.5):
                self._event.clear()
                slot  = self._slot
                ts_ns = self.ts_buf[slot]

                # Establish time origin from the first frame
                if self._start_ts == 0:
                    self._start_ts = ts_ns

                tc   = elapsed_str(self._start_ts, ts_ns)
                path = os.path.join(self.save_dir, f"frame_{self._count:06d}_{tc}{ext}")
                try:
                    imwrite_fmt(path, self.buf[slot], self.fmt)
                    self._count += 1
                except Exception as e:
                    print(f"[Disk] Write failed: {e}")

    def stop(self):
        self.running = False


# ─────────────────────────────────────────────────────────────────────────────
# Camera background thread
# ─────────────────────────────────────────────────────────────────────────────

class CameraThread(threading.Thread):
    """
    Opens an Aravis camera and continuously captures frames, distributing them to:
      - Ring Buffer      (live display)
      - Ping-Pong Buffer (normal recording)
      - Buffer list      (buffered recording)

    Parameter changes are submitted via set_param() and applied by the capture
    loop before each frame. The hardware timestamp is read before returning the
    buffer to Aravis and is passed along with the frame data to the save side.
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.running = False
        self.cam     = None
        self._lock   = threading.Lock()
        self._pending: dict = {}

        # ── Gain state ────────────────────────────────────────────
        self._gain_auto    = False
        self._current_gain = DEFAULT_GAIN

        # ── Ring Buffer (allocated by LiveViewer, then assigned) ──
        self._ring_buf:  np.ndarray = None
        self._ring_n:    int        = 0
        self._ring_widx: int        = 0   # monotonically increasing, no wrap

        # ── Capture FPS (sliding window) ──────────────────────────
        self._cap_fps_buf: deque = deque()   # timestamps of the last FPS_SAMPLE_FRAMES frames
        self._cap_fps: float     = 0.0

        # ── Normal recording Ping-Pong ─────────────────────────────
        self._recording    = False
        self._pp_buf:      np.ndarray      = None   # shape (2, h, w, 3)
        self._pp_ts:       list            = [0, 0] # hardware timestamps (ns) per slot
        self._pp_wslot:    int             = 0
        self._disk_thread: DiskWriteThread = None

        # ── Buffered recording ─────────────────────────────────────
        self._buf_active   = False
        self._buf_done     = False
        self._buf_list:    list = None
        self._buf_ts_list: list = None   # timestamps in sync with _buf_list
        self._buf_max:     int  = 0
        self._buf_count:   int  = 0
        self._buf_bytes_c: int  = 0

        # ── Latest single frame (displayed while Ring Buffer is released) ──
        self._latest_frame: np.ndarray = None

    # ── Open camera ───────────────────────────────────────────────────────────

    def open(self) -> dict:
        """Initialize the camera, apply default parameters, and query hardware
        ranges. Returns an info dict with model, resolution, and min/max for
        the three controllable parameters."""
        Aravis.update_device_list()
        if Aravis.get_n_devices() == 0:
            raise RuntimeError(
                "No camera found.\n"
                "Please check:\n"
                "  1. USB cable is connected to a USB 3.0 port\n"
                "  2. udev rules are configured (see tutorial/01_install.md)\n"
                "  3. Your user is in the plugdev group (re-login required)"
            )
        self.cam = Aravis.Camera.new(None)

        self._cam_set("ExposureAuto",        "Off",            "exposure mode")
        self._cam_set("ExposureTime",        DEFAULT_EXPOSURE, "exposure time")
        self._cam_set("GainAuto",            "Off",            "gain mode")
        self._cam_set("Gain",                DEFAULT_GAIN,     "gain")
        self._cam_set("AcquisitionFrameRate", DEFAULT_FPS,     "frame rate")

        return {
            "model"  : self.cam.get_model_name(),
            "serial" : self.cam.get_device_serial_number(),
            "width"  : self.cam.get_integer("Width"),
            "height" : self.cam.get_integer("Height"),
            **self._query_bounds(),
        }

    def _cam_set(self, feature: str, value, label: str):
        """Generic camera parameter setter; prints a warning on failure."""
        try:
            if isinstance(value, str):
                self.cam.set_string(feature, value)
            elif isinstance(value, float):
                self.cam.set_float(feature, value)
            else:
                self.cam.set_integer(feature, int(value))
        except Exception as e:
            print(f"[Camera] Failed to set {label}: {e}")

    def _query_bounds(self) -> dict:
        """Query hardware min/max for exposure, gain, and frame rate.
        Falls back to FALLBACK_* values if a query fails."""
        def get_bounds(feature, fallback_min, fallback_max):
            try:
                lo, hi = self.cam.get_float_bounds(feature)
                return lo, hi
            except Exception:
                return fallback_min, fallback_max

        exp_min,  exp_max  = get_bounds("ExposureTime",         FALLBACK_EXP_MIN,  FALLBACK_EXP_MAX)
        gain_min, gain_max = get_bounds("Gain",                 FALLBACK_GAIN_MIN, FALLBACK_GAIN_MAX)
        fps_min,  fps_max  = get_bounds("AcquisitionFrameRate", FALLBACK_FPS_MIN,  FALLBACK_FPS_MAX)

        return {
            "exp_min":  exp_min,  "exp_max":  exp_max,
            "gain_min": gain_min, "gain_max": gain_max,
            "fps_min":  fps_min,  "fps_max":  fps_max,
        }

    # ── Parameter changes (called from main thread, applied in capture loop) ──

    def set_param(self, key: str, value):
        with self._lock:
            self._pending[key] = value

    def _apply_pending(self):
        with self._lock:
            pending, self._pending = self._pending.copy(), {}
        for key, value in pending.items():
            try:
                if key == "exposure":
                    self.cam.set_string("ExposureAuto", "Off")
                    self.cam.set_float("ExposureTime", float(value))
                elif key == "gain":
                    self.cam.set_string("GainAuto", "Off")
                    self.cam.set_float("Gain", float(value))
                elif key == "gain_auto":
                    mode = "Continuous" if value else "Off"
                    self.cam.set_string("GainAuto", mode)
                    self._gain_auto = bool(value)
                elif key == "fps":
                    self.cam.set_float("AcquisitionFrameRate", float(value))
            except Exception as e:
                print(f"[Camera] Failed to apply {key}={value}: {e}")

    # ── Capture loop ──────────────────────────────────────────────────────────

    def run(self):
        self.running = True
        stream  = self.cam.create_stream(None, None)
        payload = self.cam.get_payload()
        for _ in range(STREAM_PREBUF):
            stream.push_buffer(Aravis.Buffer.new_allocate(payload))

        self.cam.set_acquisition_mode(Aravis.AcquisitionMode.CONTINUOUS)
        self.cam.start_acquisition()

        # Re-apply defaults after acquisition starts to overwrite any stale
        # firmware state the camera may have retained.
        self._cam_set("ExposureAuto",        "Off",            "exposure mode")
        self._cam_set("ExposureTime",        DEFAULT_EXPOSURE, "exposure time")
        self._cam_set("GainAuto",            "Off",            "gain mode")
        self._cam_set("Gain",                DEFAULT_GAIN,     "gain")
        self._cam_set("AcquisitionFrameRate", DEFAULT_FPS,     "frame rate")

        try:
            while self.running:
                self._apply_pending()

                buf = stream.timeout_pop_buffer(BUF_TIMEOUT_US)
                if buf is None:
                    continue

                # Timestamp must be read before push_buffer returns the buffer
                hw_ts_ns = buf.get_timestamp()
                frame    = self._decode_frame(buf)
                stream.push_buffer(buf)

                self._push_ring(frame)
                self._push_record(frame, hw_ts_ns)
                self._push_buf(frame, hw_ts_ns)
                self._update_cap_fps()

                if self._gain_auto:
                    try:
                        self._current_gain = self.cam.get_float("Gain")
                    except Exception:
                        pass
        finally:
            self.cam.stop_acquisition()

    def _push_ring(self, frame: np.ndarray):
        if self._ring_buf is not None:
            np.copyto(self._ring_buf[self._ring_widx % self._ring_n], frame)
            self._ring_widx += 1
        else:
            self._latest_frame = frame

    def _push_record(self, frame: np.ndarray, hw_ts_ns: int):
        if not self._recording or self._pp_buf is None:
            return
        slot = self._pp_wslot
        self._pp_ts[slot] = hw_ts_ns        # write timestamp first, then frame, then signal
        np.copyto(self._pp_buf[slot], frame)
        if self._disk_thread:
            self._disk_thread.signal(slot)
        self._pp_wslot = 1 - slot

    def _push_buf(self, frame: np.ndarray, hw_ts_ns: int):
        if not self._buf_active:
            return
        self._buf_list.append(frame)
        self._buf_ts_list.append(hw_ts_ns)
        self._buf_count   += 1
        self._buf_bytes_c += frame.nbytes
        if self._buf_bytes_c >= self._buf_max:
            self._buf_active = False
            self._buf_done   = True

    def _update_cap_fps(self):
        now = time.time()
        self._cap_fps_buf.append(now)
        if len(self._cap_fps_buf) > FPS_SAMPLE_FRAMES:
            self._cap_fps_buf.popleft()
        n = len(self._cap_fps_buf)
        if n >= 2:
            elapsed = self._cap_fps_buf[-1] - self._cap_fps_buf[0]
            self._cap_fps = (n - 1) / elapsed if elapsed > 0 else 0.0

    def stop(self):
        self.running = False

    # ── Frame decoding ────────────────────────────────────────────────────────

    @staticmethod
    def _decode_frame(buf: Aravis.Buffer) -> np.ndarray:
        """Convert an Aravis Buffer to a BGR uint8 array."""
        h   = buf.get_image_height()
        w   = buf.get_image_width()
        fmt = buf.get_image_pixel_format()
        raw = buf.get_data()

        if fmt == Aravis.PIXEL_FORMAT_MONO_8:
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w))
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

        if fmt == Aravis.PIXEL_FORMAT_MONO_16:
            arr = np.frombuffer(raw, dtype=np.uint16).reshape((h, w))
            return cv2.cvtColor((arr >> 4).astype(np.uint8), cv2.COLOR_GRAY2BGR)

        if fmt == Aravis.PIXEL_FORMAT_BAYER_RG_8:
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w))
            return cv2.cvtColor(arr, cv2.COLOR_BayerRG2BGR)

        if fmt == Aravis.PIXEL_FORMAT_BAYER_GB_8:
            arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w))
            return cv2.cvtColor(arr, cv2.COLOR_BayerGB2BGR)

        # Unknown format: fall back to Mono8 interpretation
        arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w))
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)


# ─────────────────────────────────────────────────────────────────────────────
# Main window
# ─────────────────────────────────────────────────────────────────────────────

class LiveViewer:

    # ── Colour palette ─────────────────────────────────────────────
    C_BG        = "#1e1e1e"
    C_SIDEBAR   = "#252526"
    C_FG        = "#d4d4d4"
    C_FG_DIM    = "#888888"
    C_ACCENT    = "#3a7bd5"
    C_RECORD    = "#c0392b"
    C_RECORD_ON = "#555555"
    C_BUF       = "#7b3fa0"
    C_BUF_ON    = "#555555"

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("U3V Viewer v2")
        self.root.configure(bg=self.C_BG)
        self.root.resizable(False, False)

        self.cam_thread: CameraThread = None

        # ── Camera dimensions (defaults used for layout before camera opens) ──
        self._cam_w = DEFAULT_CAM_W
        self._cam_h = DEFAULT_CAM_H

        # ── Recording state ───────────────────────────────────────
        self.recording     = False
        self._record_dir   = ""
        self._ring_buf:    np.ndarray      = None
        self._pp_buf:      np.ndarray      = None
        self._pp_ts:       list            = [0, 0]
        self._disk_thread: DiskWriteThread = None

        # ── Buffered recording state ──────────────────────────────
        self._buf_recording = False
        self._buf_saving    = False
        self._buf_frames:   list = []
        self._buf_ts:       list = []

        # ── Display FPS counter ────────────────────────────────────
        self._disp_fps_count = 0
        self._disp_fps_tick  = time.time()

        # ── Most recent frame (used for snapshot) ─────────────────
        self._last_frame: np.ndarray = None

        # ── Slider / entry references (updated after camera opens) ─
        self._slider_exposure:    ttk.Scale    = None
        self._slider_gain:        ttk.Scale    = None
        self._slider_fps:         ttk.Scale    = None
        self._entry_var_exposure: tk.StringVar = None
        self._entry_var_gain:     tk.StringVar = None
        self._entry_var_fps:      tk.StringVar = None

        # ── Save format (v2) ──────────────────────────────────────
        self.var_fmt = tk.StringVar(value="BMP")

        self._build_ui()
        self._bind_keys()

    # ══════════════════════════════════════════════════════════════════════════
    # UI construction
    # ══════════════════════════════════════════════════════════════════════════

    def _build_ui(self):
        container = tk.Frame(self.root, bg=self.C_BG)
        container.pack(fill=tk.BOTH, expand=True)
        self._build_canvas(container)
        self._build_sidebar(container)

    def _build_canvas(self, parent):
        self._display_h = int(DISPLAY_W * self._cam_h / self._cam_w)
        self.canvas = tk.Canvas(
            parent, width=DISPLAY_W, height=self._display_h,
            bg="#000000", highlightthickness=0,
        )
        self.canvas.pack(side=tk.LEFT)
        self.canvas.create_text(
            DISPLAY_W // 2, self._display_h // 2,
            text="Waiting for camera...", fill="#555555",
            font=("Helvetica", 16), tags="placeholder",
        )

    def _build_sidebar(self, parent):
        self._sb = tk.Frame(parent, width=SIDEBAR_W, bg=self.C_SIDEBAR)
        self._sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._sb.pack_propagate(False)

        # Camera info
        self._section("Camera Info")
        self.lbl_model      = self._info_row("Model",       "—")
        self.lbl_serial     = self._info_row("Serial",      "—")
        self.lbl_resolution = self._info_row("Resolution",  "—")
        self.lbl_fps_cap    = self._info_row("Capture FPS", "—")
        self.lbl_fps_disp   = self._info_row("Display FPS", "—")
        self._divider()

        # Camera on/off
        self.btn_cam = self._btn("Open Camera", self.C_ACCENT, self._toggle_camera)

        # Exposure
        self._section("Exposure (µs)")
        self.var_exposure = tk.DoubleVar(value=DEFAULT_EXPOSURE)
        self._slider_exposure, self._entry_var_exposure, self._exp_bounds = self._param_row(
            self.var_exposure,
            from_=FALLBACK_EXP_MIN, to=FALLBACK_EXP_MAX, resolution=100,
            on_change=lambda: self._apply("exposure", self.var_exposure.get()),
            log_scale=True,
        )

        # Gain
        self._section("Gain (dB)")
        self.var_gain      = tk.DoubleVar(value=DEFAULT_GAIN)
        self.var_gain_auto = tk.BooleanVar(value=False)
        gain_header = tk.Frame(self._sb, bg=self.C_SIDEBAR)
        gain_header.pack(fill=tk.X, padx=14, pady=(0, 2))
        tk.Checkbutton(
            gain_header, text="Auto Gain",
            variable=self.var_gain_auto,
            bg=self.C_SIDEBAR, fg=self.C_FG, selectcolor="#3a3a3a",
            activebackground=self.C_SIDEBAR, activeforeground=self.C_FG,
            font=("Helvetica", 9),
            command=self._on_gain_auto_toggle,
        ).pack(side=tk.LEFT)
        self._slider_gain, self._entry_var_gain, _ = self._param_row(
            self.var_gain,
            from_=FALLBACK_GAIN_MIN, to=FALLBACK_GAIN_MAX, resolution=0.1,
            on_change=lambda: self._apply("gain", self.var_gain.get()),
        )

        # Frame rate
        self._section("Frame Rate Limit (fps)")
        self.var_fps = tk.DoubleVar(value=DEFAULT_FPS)
        self._slider_fps, self._entry_var_fps, self._fps_bounds = self._param_row(
            self.var_fps,
            from_=FALLBACK_FPS_MIN, to=FALLBACK_FPS_MAX, resolution=1,
            on_change=lambda: self._apply("fps", self.var_fps.get()),
            log_scale=True,
        )
        self._divider()

        # Capture
        self._section("Capture")

        # ── Format selector (v2) ───────────────────────────────────
        fmt_row = tk.Frame(self._sb, bg=self.C_SIDEBAR)
        fmt_row.pack(fill=tk.X, padx=14, pady=(0, 6))
        tk.Label(
            fmt_row, text="Save format:",
            bg=self.C_SIDEBAR, fg=self.C_FG_DIM,
            font=("Helvetica", 9),
        ).pack(side=tk.LEFT)
        for fmt_val, fmt_lbl in (("BMP", "BMP (lossless)"), ("JPG", f"JPG (Q{JPEG_QUALITY})")):
            tk.Radiobutton(
                fmt_row, text=fmt_lbl,
                variable=self.var_fmt, value=fmt_val,
                bg=self.C_SIDEBAR, fg=self.C_FG,
                selectcolor="#3a3a3a",
                activebackground=self.C_SIDEBAR, activeforeground=self.C_FG,
                font=("Helvetica", 9),
            ).pack(side=tk.LEFT, padx=(6, 0))
        # ──────────────────────────────────────────────────────────

        self._btn("📷  Snapshot  (s)", self.C_ACCENT, self._take_photo)
        self.btn_record = self._btn("⏺  Start Recording  (r)", self.C_RECORD, self._toggle_record)
        self.btn_buf    = self._btn("⏺  Buffer Record (RAM)", self.C_BUF, self._toggle_buf_record)
        self._divider()

        # Status bar
        self.lbl_status = tk.Label(
            self._sb, text="Camera not connected",
            bg=self.C_SIDEBAR, fg=self.C_FG_DIM,
            font=("Helvetica", 9), wraplength=SIDEBAR_W - 24,
            justify=tk.LEFT, anchor=tk.W,
        )
        self.lbl_status.pack(fill=tk.X, padx=14, pady=(0, 10))

    # ── Widget factories ──────────────────────────────────────────────────────

    def _section(self, title: str):
        tk.Label(
            self._sb, text=title,
            bg=self.C_SIDEBAR, fg=self.C_FG,
            font=("Helvetica", 10, "bold"),
        ).pack(anchor=tk.W, padx=14, pady=(10, 2))

    def _divider(self):
        tk.Frame(self._sb, height=1, bg="#3a3a3a").pack(fill=tk.X, padx=14, pady=8)

    def _info_row(self, label: str, default: str) -> tk.Label:
        row = tk.Frame(self._sb, bg=self.C_SIDEBAR)
        row.pack(fill=tk.X, padx=14, pady=1)
        tk.Label(
            row, text=label + ":",
            bg=self.C_SIDEBAR, fg=self.C_FG_DIM,
            font=("Helvetica", 9), width=10, anchor=tk.W,
        ).pack(side=tk.LEFT)
        lbl = tk.Label(row, text=default, bg=self.C_SIDEBAR, fg=self.C_FG,
                       font=("Helvetica", 9), anchor=tk.W)
        lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)
        return lbl

    def _param_row(self, var: tk.DoubleVar, from_: float, to: float,
                   resolution: float, on_change, log_scale: bool = False) -> tuple:
        """Create a slider + numeric entry row.

        When log_scale=True the slider's internal range is 0.0–1.0 (normalised
        log position) while the entry box always shows the physical value.
        Returns (slider, entry_var, bounds).
        bounds = [lo, hi] is a mutable list so _update_slider_range can update
        the log transform's physical range after hardware bounds are known.
        When log_scale=False, bounds is None.
        """
        row = tk.Frame(self._sb, bg=self.C_SIDEBAR)
        row.pack(fill=tk.X, padx=14, pady=(0, 4))
        entry_var = tk.StringVar(value=str(var.get()))

        # Mutable list so the closure can see external updates to the range
        bounds = [from_, to] if log_scale else None

        def _log_to_phys(pos: float) -> float:
            lo, hi = bounds
            return lo * (hi / lo) ** pos

        def _phys_to_log(v: float) -> float:
            lo, hi = bounds
            return math.log(v / lo) / math.log(hi / lo)

        def on_slider(val):
            if log_scale:
                v = round(_log_to_phys(float(val)))
                v = int(clamp(v, bounds[0], bounds[1]))
            else:
                v = round(round(float(val) / resolution) * resolution, 2)
                v = clamp(v, slider.cget("from"), slider.cget("to"))
            var.set(v)
            entry_var.set(str(v))
            on_change()

        def on_entry(event=None):
            try:
                lo, hi = bounds if log_scale else (slider.cget("from"), slider.cget("to"))
                v = clamp(float(entry_var.get()), lo, hi)
                v = int(round(v)) if log_scale else round(v, 2)
                var.set(v)
                entry_var.set(str(v))
                slider.set(_phys_to_log(v) if log_scale else v)
                on_change()
            except ValueError:
                entry_var.set(str(var.get()))

        if log_scale:
            slider = ttk.Scale(row, from_=0.0, to=1.0,
                               orient=tk.HORIZONTAL, command=on_slider)
            slider.set(_phys_to_log(var.get()))
        else:
            slider = ttk.Scale(row, from_=from_, to=to,
                               orient=tk.HORIZONTAL, command=on_slider)
            slider.set(var.get())
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

        entry = tk.Entry(row, textvariable=entry_var, width=8,
                         bg="#3a3a3a", fg=self.C_FG,
                         insertbackground=self.C_FG,
                         relief=tk.FLAT, font=("Helvetica", 9))
        entry.pack(side=tk.RIGHT, padx=(6, 0))
        entry.bind("<Return>",   on_entry)
        entry.bind("<FocusOut>", on_entry)

        return slider, entry_var, bounds

    def _btn(self, text: str, bg: str, command) -> tk.Button:
        btn = tk.Button(
            self._sb, text=text, font=("Helvetica", 11),
            bg=bg, fg="white", activebackground=bg, activeforeground="white",
            bd=0, pady=9, cursor="hand2", command=command,
        )
        btn.pack(fill=tk.X, padx=14, pady=3)
        return btn

    # ── Slider helpers ────────────────────────────────────────────────────────

    def _update_slider_range(self, slider: ttk.Scale, entry_var: tk.StringVar,
                             var: tk.DoubleVar, lo: float, hi: float, init: float,
                             log_scale: bool = False, bounds: list = None):
        """Update slider range and set the value to init (clamped).

        For log_scale=True the slider's internal range stays at 0–1; only the
        bounds list and the displayed position are updated.
        """
        if log_scale and bounds is not None:
            bounds[0] = lo
            bounds[1] = hi
            v = int(clamp(init, lo, hi))
            pos = math.log(v / lo) / math.log(hi / lo)
            slider.set(pos)
        else:
            v = round(clamp(init, lo, hi), 2)
            slider.configure(from_=lo, to=hi)
            slider.set(v)
        var.set(v)
        entry_var.set(str(v))

    # ══════════════════════════════════════════════════════════════════════════
    # Camera control
    # ══════════════════════════════════════════════════════════════════════════

    def _toggle_camera(self):
        if self.cam_thread is None:
            self._open_camera()
        else:
            self._close_camera()

    def _open_camera(self):
        try:
            self.cam_thread = CameraThread()
            info = self.cam_thread.open()
        except Exception as e:
            self.cam_thread = None
            self._status(f"Failed to open camera: {e}")
            return

        self._cam_w = info["width"]
        self._cam_h = info["height"]

        # Resize canvas to match sensor aspect ratio
        new_h = int(DISPLAY_W * self._cam_h / self._cam_w)
        if new_h != self._display_h:
            self._display_h = new_h
            self.canvas.config(height=new_h)

        # Update info labels
        self.lbl_model.config(text=info["model"])
        self.lbl_serial.config(text=info["serial"])
        self.lbl_resolution.config(text=f"{info['width']} × {info['height']}")

        # Update slider ranges from hardware bounds
        self._update_slider_range(
            self._slider_exposure, self._entry_var_exposure, self.var_exposure,
            info["exp_min"], info["exp_max"], DEFAULT_EXPOSURE,
            log_scale=True, bounds=self._exp_bounds,
        )
        self._update_slider_range(
            self._slider_gain, self._entry_var_gain, self.var_gain,
            info["gain_min"], info["gain_max"], DEFAULT_GAIN,
        )
        # FPS slider uses fixed range (1–1000) to avoid invalid hardware values
        self._update_slider_range(
            self._slider_fps, self._entry_var_fps, self.var_fps,
            FALLBACK_FPS_MIN, FALLBACK_FPS_MAX, DEFAULT_FPS,
            log_scale=True, bounds=self._fps_bounds,
        )
        self.var_gain_auto.set(False)

        # Allocate Ring Buffer
        frame_bytes = info["width"] * info["height"] * 3
        n_frames    = min(RING_MAX_BYTES // frame_bytes, RING_MAX_FRAMES)
        self._ring_buf = np.zeros(
            (n_frames, info["height"], info["width"], 3), dtype=np.uint8
        )

        # Allocate Ping-Pong Buffer (normal recording)
        self._pp_buf = np.zeros(
            (2, info["height"], info["width"], 3), dtype=np.uint8
        )
        self._pp_ts = [0, 0]

        # Pass buffer references to camera thread
        self.cam_thread._ring_buf = self._ring_buf
        self.cam_thread._ring_n   = n_frames
        self.cam_thread._pp_ts    = self._pp_ts

        self.cam_thread.start()
        self.btn_cam.config(text="Close Camera", bg=self.C_RECORD)
        self._status("Camera connected")
        self._update_frame()

    def _close_camera(self):
        if self.recording:
            self._stop_record()
        if self._buf_recording and self.cam_thread:
            self.cam_thread._buf_active = False
            self._buf_recording = False

        if self.cam_thread:
            self.cam_thread.stop()
            self.cam_thread.join(timeout=CAM_JOIN_TIMEOUT)
            self.cam_thread = None

        self._ring_buf = None
        self._pp_buf   = None

        for lbl in (self.lbl_model, self.lbl_serial, self.lbl_resolution,
                    self.lbl_fps_cap, self.lbl_fps_disp):
            lbl.config(text="—")

        self.btn_cam.config(text="Open Camera", bg=self.C_ACCENT)
        self._status("Camera disconnected")

    def _on_gain_auto_toggle(self):
        if self.cam_thread:
            self.cam_thread.set_param("gain_auto", self.var_gain_auto.get())

    def _apply(self, key: str, value):
        if key == "gain" and self.var_gain_auto.get():
            return
        if self.cam_thread:
            self.cam_thread.set_param(key, value)

    # ══════════════════════════════════════════════════════════════════════════
    # Frame update loop (main thread, ~60 Hz)
    # ══════════════════════════════════════════════════════════════════════════

    def _update_frame(self):
        ct = self.cam_thread
        if ct is None:
            return

        # Fetch the latest frame from the Ring Buffer or latest single frame
        if ct._ring_buf is not None:
            if ct._ring_widx > 0:
                frame = ct._ring_buf[(ct._ring_widx - 1) % ct._ring_n]
                self._last_frame = frame.copy()
                if not self._buf_recording:
                    self._render(frame)
                    self._tick_disp_fps()
        elif self._buf_saving and ct._latest_frame is not None:
            self._last_frame = ct._latest_frame
            self._render(ct._latest_frame)
            self._tick_disp_fps()

        # Capture FPS
        fps = ct._cap_fps
        fps_str = f"{fps:.2f}" if fps < 10 else f"{fps:.1f}"
        self.lbl_fps_cap.config(text=f"{fps_str} fps")

        # Sync gain slider when auto-gain is active
        if self.var_gain_auto.get():
            cur = round(ct._current_gain, 2)
            self.var_gain.set(cur)
            self._slider_gain.set(cur)
            self._entry_var_gain.set(str(cur))

        # Normal recording status
        if self.recording and self._disk_thread is not None:
            used_gb = self._disk_thread.count * self._cam_w * self._cam_h * 3 / 1024 ** 3
            free_gb = disk_free_gb(self._record_dir)
            self._status(
                f"Recording: {self._disk_thread.count} frames / "
                f"{used_gb:.2f} GB used  ({free_gb:.1f} GB free)"
            )

        # Buffered recording status
        if self._buf_recording:
            used_gb = ct._buf_bytes_c / 1024 ** 3
            rem_gb  = (BUF_MAX_BYTES - ct._buf_bytes_c) / 1024 ** 3
            self._status(
                f"Buffer recording: {ct._buf_count} frames / "
                f"{used_gb:.2f} GB used  ({rem_gb:.2f} GB remaining)"
            )
            if ct._buf_done:
                ct._buf_done        = False
                self._buf_recording = False
                self._finalize_buf_record()

        if self.cam_thread is not None:
            self.root.after(DISPLAY_REFRESH_MS, self._update_frame)

    def _tick_disp_fps(self):
        """Update the display FPS label once per second."""
        self._disp_fps_count += 1
        now = time.time()
        if now - self._disp_fps_tick >= 1.0:
            self.lbl_fps_disp.config(text=f"{self._disp_fps_count} fps")
            self._disp_fps_count = 0
            self._disp_fps_tick  = now

    def _render(self, frame: np.ndarray):
        resized = cv2.resize(frame, (DISPLAY_W, self._display_h))
        rgb     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        img     = ImageTk.PhotoImage(Image.fromarray(rgb))
        self._tk_img = img   # keep a reference to prevent GC
        self.canvas.delete("all")
        self.canvas.create_image(0, 0, anchor=tk.NW, image=img)
        if self.recording:
            self.canvas.create_rectangle(8, 8, 80, 30, fill="#c0392b", outline="")
            self.canvas.create_text(44, 19, text="⏺ REC",
                                    fill="white", font=("Helvetica", 10, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # Snapshot
    # ══════════════════════════════════════════════════════════════════════════

    def _take_photo(self):
        if self.cam_thread is None:
            self._status("Snapshot failed: camera not connected")
            return
        if self._last_frame is None:
            self._status("Snapshot failed: no frame received yet")
            return

        fmt    = self.var_fmt.get()
        ext    = ".jpg" if fmt == "JPG" else ".bmp"
        now    = datetime.now()
        subdir = os.path.join(CAPTURE_DIR, now.strftime("%Y%m%d"))
        os.makedirs(subdir, exist_ok=True)
        path = os.path.join(subdir, f"photo_{now.strftime('%Y%m%d_%H%M%S')}{ext}")
        imwrite_fmt(path, self._last_frame, fmt)

        size_mb = os.path.getsize(path) / 1024 ** 2
        self._status(
            f"Saved: {now.strftime('%Y%m%d')}/{os.path.basename(path)}"
            f"  ({size_mb:.2f} MB, {disk_free_gb(path):.1f} GB free)"
        )

    # ══════════════════════════════════════════════════════════════════════════
    # Normal recording (Ping-Pong → disk thread)
    # ══════════════════════════════════════════════════════════════════════════

    def _toggle_record(self):
        if self.recording:
            self._stop_record()
        elif self.cam_thread is None:
            self._status("Recording failed: camera not connected")
        elif self._buf_recording:
            self._status("Buffer recording in progress — cannot record simultaneously")
        else:
            self._start_record()

    def _start_record(self):
        now = datetime.now()
        self._record_dir = os.path.join(
            CAPTURE_DIR, now.strftime("%Y%m%d"), f"video_{now.strftime('%Y%m%d_%H%M%S')}"
        )
        os.makedirs(self._record_dir, exist_ok=True)

        self._disk_thread          = DiskWriteThread()
        self._disk_thread.buf      = self._pp_buf
        self._disk_thread.ts_buf   = self._pp_ts
        self._disk_thread.save_dir = self._record_dir
        self._disk_thread.fmt      = self.var_fmt.get()
        self._disk_thread.start()

        ct              = self.cam_thread
        ct._pp_buf      = self._pp_buf
        ct._pp_ts       = self._pp_ts
        ct._pp_wslot    = 0
        ct._disk_thread = self._disk_thread
        ct._recording   = True

        self.recording = True
        self.btn_record.config(text="⏹  Stop Recording  (r)", bg=self.C_RECORD_ON)
        self._status(f"Recording: {os.path.relpath(self._record_dir, ROOT_DIR)}")

    def _stop_record(self):
        if self.cam_thread:
            self.cam_thread._recording = False
        self.recording = False

        n = 0
        if self._disk_thread:
            self._disk_thread.stop()
            self._disk_thread.join(timeout=DISK_JOIN_TIMEOUT)
            n = self._disk_thread.count
            self._disk_thread = None

        self.btn_record.config(text="⏺  Start Recording  (r)", bg=self.C_RECORD)
        self._status(f"Saved {n} frames → {os.path.relpath(self._record_dir, ROOT_DIR)}")

    # ══════════════════════════════════════════════════════════════════════════
    # Buffered recording (RAM → disk)
    # ══════════════════════════════════════════════════════════════════════════

    def _toggle_buf_record(self):
        if self._buf_saving:
            self._status("Saving in progress, please wait...")
        elif self._buf_recording:
            self._stop_buf_record()
        elif self.cam_thread is None:
            self._status("Buffer recording failed: camera not connected")
        elif self.recording:
            self._status("Stop normal recording first")
        else:
            self._start_buf_record()

    def _start_buf_record(self):
        ct = self.cam_thread

        # Release the Ring Buffer to avoid holding two large allocations at once
        ct._ring_buf  = None
        ct._ring_n    = 0
        ct._ring_widx = 0
        self._ring_buf = None

        self._buf_frames = []
        self._buf_ts     = []
        ct._buf_list     = self._buf_frames
        ct._buf_ts_list  = self._buf_ts
        ct._buf_count    = 0
        ct._buf_bytes_c  = 0
        ct._buf_max      = BUF_MAX_BYTES
        ct._buf_done     = False
        ct._buf_active   = True

        self._buf_recording = True
        self.btn_buf.config(text="⏹  Stop Buffer Record", bg=self.C_BUF_ON)
        self._status("Buffer recording...")

    def _stop_buf_record(self):
        if self.cam_thread:
            self.cam_thread._buf_active = False
        self._buf_recording = False
        self._finalize_buf_record()

    def _finalize_buf_record(self):
        frames, self._buf_frames = self._buf_frames, []
        ts,     self._buf_ts     = self._buf_ts,     []
        if not frames:
            self.btn_buf.config(text="⏺  Buffer Record (RAM)", bg=self.C_BUF)
            self._status("Buffer recording stopped: no frames to save")
            return
        self._buf_saving = True
        self.btn_buf.config(text="💾  Saving...", bg="#888888")
        self._status(f"Buffer stopped — {len(frames)} frames, saving in background...")
        fmt = self.var_fmt.get()   # cache format so mid-save changes don't affect this batch
        threading.Thread(
            target=self._save_buf_frames, args=(frames, ts, fmt), daemon=True
        ).start()

    def _save_buf_frames(self, frames: list, timestamps: list, fmt: str):
        now = datetime.now()
        save_dir = os.path.join(
            CAPTURE_DIR, now.strftime("%Y%m%d"), f"buf_{now.strftime('%Y%m%d_%H%M%S')}"
        )
        os.makedirs(save_dir, exist_ok=True)
        ext      = ".jpg" if fmt == "JPG" else ".bmp"
        start_ts = timestamps[0] if timestamps else 0

        def write_one(args):
            i, frame, ts_ns = args
            tc   = elapsed_str(start_ts, ts_ns)
            path = os.path.join(save_dir, f"frame_{i:06d}_{tc}{ext}")
            imwrite_fmt(path, frame, fmt)

        with ThreadPoolExecutor(max_workers=BUF_SAVE_WORKERS) as pool:
            pool.map(write_one, ((i, f, t) for i, (f, t) in enumerate(zip(frames, timestamps))))

        self.root.after(0, self._on_buf_save_done, len(frames), save_dir)

    def _on_buf_save_done(self, n: int, save_dir: str):
        self._buf_saving = False

        # Re-allocate Ring Buffer
        if self.cam_thread is not None:
            frame_bytes = self._cam_w * self._cam_h * 3
            n_frames    = min(RING_MAX_BYTES // frame_bytes, RING_MAX_FRAMES)
            self._ring_buf = np.zeros(
                (n_frames, self._cam_h, self._cam_w, 3), dtype=np.uint8
            )
            self.cam_thread._ring_buf  = self._ring_buf
            self.cam_thread._ring_n    = n_frames
            self.cam_thread._ring_widx = 0

        self.btn_buf.config(text="⏺  Buffer Record (RAM)", bg=self.C_BUF)
        self._status(f"Saved {n} frames → {os.path.relpath(save_dir, ROOT_DIR)}")

    # ══════════════════════════════════════════════════════════════════════════
    # Hotkeys / status bar / close
    # ══════════════════════════════════════════════════════════════════════════

    def _bind_keys(self):
        self.root.bind("<KeyPress-s>", lambda _: self._take_photo())
        self.root.bind("<KeyPress-r>", lambda _: self._toggle_record())
        self.root.bind("<KeyPress-q>", lambda _: self._on_close())

    def _status(self, msg: str):
        self.lbl_status.config(text=msg)

    def _on_close(self):
        from tkinter import messagebox
        saving = self._buf_saving or (
            self._disk_thread is not None and self._disk_thread.is_alive()
        )
        if saving:
            if not messagebox.askyesno(
                "Confirm Close",
                "A save thread is still running. Force-closing may result in "
                "incomplete data.\nClose anyway?"
            ):
                return

        if self._buf_recording and self.cam_thread:
            self.cam_thread._buf_active = False
            self._buf_recording = False

        if self._disk_thread:
            if self.cam_thread:
                self.cam_thread._recording = False
            self._disk_thread.stop()
            self._disk_thread = None

        if self.cam_thread:
            self.cam_thread.stop()
            self.cam_thread.join(timeout=CAM_JOIN_TIMEOUT)
            self.cam_thread = None

        self.root.destroy()

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.mainloop()


# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    LiveViewer().run()
