#  Camera USB3 Vision Live Stream Viewer
## u3v_viewer_v2
— Hikvision Industrial
Drives a Hikvision USB3 Vision industrial camera via Aravis, providing live
preview, parameter adjustment, snapshot, and recording features.
Supports both BMP (lossless) and JPG (configurable quality) output formats.

---

## System Requirements

| Item | Requirement |
|------|-------------|
| OS | Ubuntu 20.04 / 22.04 (or any Linux with Aravis support) |
| Python | 3.9 or later |
| Camera interface | USB 3.0 |
| RAM | Should exceed RING_MAX_BYTES + BUF_MAX_BYTES (default 4 GB each, 8 GB total) |

---

## Installation

### 1. System packages

All Python dependencies are installed via apt (pip is locked on newer systems
by PEP 668):

```bash
sudo apt-get install -y \
    python3-opencv \
    python3-numpy \
    python3-pil python3-pil.imagetk \
    gir1.2-aravis-0.8 aravis-tools-cli \
    python3-gi python3-gi-cairo \
    python3-tk
```

### 2. udev rules (camera access permissions, one-time setup)

Allow non-root users to access the camera without `sudo`:

```bash
sudo bash -c 'cat > /etc/udev/rules.d/99-hikvision.rules << EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", MODE="0666", GROUP="plugdev"
EOF'

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Add your user to the `plugdev` group:

```bash
sudo usermod -aG plugdev $USER
```

> **Re-login** for the group change to take effect. To apply it in the current
> session without re-logging in:
> ```bash
> newgrp plugdev
> ```

---

## Running

```bash
python3 scripts/test/en/u3v_viewer_v2.py
```

---

## Interface

```
┌───────────────────────────────┬──────────────────────────────┐
│                               │  Camera Info                  │
│                               │    Model / Serial / Res       │
│                               │    Capture FPS / Display FPS  │
│                               │  ──────────────────────────   │
│       Live image feed         │  [ Open Camera ]              │
│       (860 px wide,           │                               │
│        aspect-ratio scaled)   │  Exposure (µs)  [log slider]  │
│                               │  [──────────────] [ entry ]   │
│                               │                               │
│   ┌──────┐                    │  Gain (dB)                    │
│   │⏺ REC│  shown while        │  □ Auto Gain                  │
│   └──────┘  recording         │  [──────────────] [ entry ]   │
│                               │                               │
│                               │  Frame Rate Limit (fps) [log] │
│                               │  [──────────────] [ entry ]   │
│                               │  ──────────────────────────   │
│                               │  Capture                      │
│                               │  Save format: ◉ BMP  ○ JPG   │
│                               │  [ 📷 Snapshot (s) ]          │
│                               │  [ ⏺ Start Recording (r) ]   │
│                               │  [ ⏺ Buffer Record (RAM) ]   │
│                               │  ──────────────────────────   │
│                               │  Status message               │
└───────────────────────────────┴──────────────────────────────┘
```

---

## Operation

| Action | How |
|--------|-----|
| Open / close camera | Click "Open Camera" button |
| Select save format | Click "BMP (lossless)" or "JPG (Q85)" radio button |
| Snapshot | Click "📷 Snapshot" or press `s` |
| Start / stop recording | Click "⏺ Start Recording" or press `r` |
| Buffered recording | Click "⏺ Buffer Record (RAM)", click again to stop and save |
| Adjust exposure / gain / FPS | Drag the log-scale slider or type a value and press Enter |
| Auto gain | Check "Auto Gain" — the slider tracks the camera's live reading |
| Quit | Close the window or press `q` |

> Exposure and FPS sliders use a **logarithmic scale** for fine control across
> a wide range. The entry box always shows the physical value.

> Slider ranges for exposure and gain are read from hardware after the camera
> opens and are updated automatically.

> The save format can be changed at any time. During a buffered recording save,
> the format is locked to whatever was selected when recording stopped.

---

## Recording Modes

| Mode | Mechanism | Best for |
|------|-----------|----------|
| Normal recording | Ping-Pong Buffer → write each frame immediately | Long recordings; throughput limited by disk write speed |
| Buffered recording | Fill RAM up to BUF_MAX_BYTES, then flush to disk in background | Short high-speed bursts; not limited by disk write speed during capture |

> During buffered recording the Ring Buffer is released to avoid occupying
> two large memory regions simultaneously.

---

## Output Location

All files are saved under `captures/`, organised by date.
Timestamps in filenames (`HH_MM_SS_mmm`) are derived from the camera's
hardware clock and represent the offset from the start of the recording.
The file extension (`.bmp` or `.jpg`) follows the format selector.

```
captures/
└── YYYYMMDD/
    ├── photo_YYYYMMDD_HHMMSS.bmp/.jpg              # snapshot
    ├── video_YYYYMMDD_HHMMSS/                       # normal recording
    │   ├── frame_000000_00_00_00_000.bmp/.jpg
    │   ├── frame_000001_00_00_00_005.bmp/.jpg
    │   └── ...
    └── buf_YYYYMMDD_HHMMSS/                         # buffered recording
        ├── frame_000000_00_00_00_000.bmp/.jpg
        └── ...
```

---

## Tunable Constants

All constants are grouped at the top of `u3v_viewer_v2.py` for easy
maintenance:

| Constant | Default | Description |
|----------|---------|-------------|
| `DEFAULT_EXPOSURE` | 30000 µs | Startup exposure time |
| `DEFAULT_GAIN` | 0.0 dB | Startup gain |
| `DEFAULT_FPS` | 200 fps | Startup frame rate limit |
| `JPEG_QUALITY` | 85 | JPG save quality (1–100) |
| `RING_MAX_BYTES` | 4 GB | Ring Buffer memory cap (adjust to suit available RAM) |
| `BUF_MAX_BYTES` | 4 GB | Buffered recording memory cap (adjust to suit available RAM) |
| `RING_MAX_FRAMES` | 5000 | Ring Buffer max frame count (see note below) |
| `DISPLAY_W` | 860 px | Image canvas width |
| `DISPLAY_REFRESH_MS` | 16 ms | Display update interval (~60 Hz) |
| `STREAM_PREBUF` | 10 | Pre-allocated Aravis stream buffers |
| `BUF_TIMEOUT_US` | 1 000 000 µs | Aravis buffer pop timeout |
| `FPS_SAMPLE_FRAMES` | 30 | Sliding-window size for capture FPS |
| `BUF_SAVE_WORKERS` | None | Parallel workers for buffered save (None = auto) |
| `FALLBACK_FPS_MIN/MAX` | 1 / 1000 fps | Fixed FPS slider range (not read from hardware) |

**`RING_MAX_FRAMES` note:** actual frame count =
`min(RING_MAX_BYTES ÷ bytes_per_frame, RING_MAX_FRAMES)`. At low resolutions
the memory limit alone could allow tens of thousands of frames; this constant
prevents an excessively large one-time `np.zeros` allocation.

**`BUF_SAVE_WORKERS` note:** BMP saves are I/O-bound — extra workers help
mainly on fast SSDs. JPG saves are CPU-bound — workers equal to physical core
count give the best speedup.

---

## Camera Compatibility

This program was developed and tested with **Hikvision USB3 Vision cameras**.
The transport layer is Aravis implementing the USB3 Vision standard, so other
brands can connect and stream in principle, but the following four areas may
need adjustment depending on the brand:

### 1. udev rule

The rule installed in the setup step only covers the Hikvision VID (`2bdf`),
so other brands will lack device access. Find your camera's VID with:

```bash
lsusb | grep <brand keyword>
```

A rule that covers all USB3 Vision devices:

```
SUBSYSTEM=="usb", ATTR{bDeviceClass}=="ef", MODE="0666", GROUP="plugdev"
```

### 2. GenICam node names

Affected code: `CameraThread.open()`, `run()`, `_apply_pending()`, `_query_bounds()`

| Feature | This program | Other vendors may use |
|---------|-------------|----------------------|
| Exposure | `ExposureTime` | `ExposureTimeAbs`, `ExposureTimeRaw` |
| Gain | `Gain` | `AnalogGain`, `GainRaw` |
| Frame rate | `AcquisitionFrameRate` | `AcquisitionFrameRateAbs` |

Failed writes print a warning and fall back to defaults — the program will not
crash, but that parameter cannot be controlled. List supported nodes with:

```bash
arv-tool-0.8 --name <camera name> features
```

### 3. Frame rate enable flag

Affected code: `CameraThread.open()`, `run()`

Some brands (e.g. Basler, FLIR) require an enable flag before the frame rate
can be written. Add this line before the `AcquisitionFrameRate` call:

```python
self.cam.set_boolean("AcquisitionFrameRateEnable", True)
```

### 4. Pixel format

Affected code: `CameraThread._decode_frame()`

Currently handled: `MONO_8` / `MONO_16` / `BAYER_RG_8` / `BAYER_GB_8`

If the camera outputs a different format (`BAYER_BG_8`, `BAYER_GR_8`, `RGB8`,
`YUV422`, etc.), add the corresponding `if`-branch and `cv2` colour conversion
inside `_decode_frame()`. Unhandled formats currently fall back to Mono8
interpretation and may display incorrectly.

### Compatibility summary

| Scenario | Compatibility |
|----------|---------------|
| Other Hikvision U3V cameras | Full |
| Major vendors (Basler, FLIR, Allied Vision) | Connects; parameter control depends on node names |
| Non-standard or proprietary-protocol cameras | Not supported |
