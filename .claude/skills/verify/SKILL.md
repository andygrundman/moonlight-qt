---
name: verify
description: Verify Moonlight changes end-to-end by streaming from the local Sunshine host (loopback, RX 9070 XT box)
---

# Verify via loopback stream against local Sunshine

Build: `cd /home/azr/src/moonlight-qt && make -j$(nproc)`. Gotcha: qmake does NOT
track `moonlight-common-c/libmoonlight-common-c.a` as a link dependency — after
touching common-c sources, `rm app/moonlight && make` to force the relink.

Host: `/home/azr/src/sunshine/build/sunshine` (config `~/.config/sunshine/sunshine.conf`:
capture=kwin, encoder=vulkan, pyrowave_mode=2; log `~/.config/sunshine/sunshine.log`).
Start it in background if not running; it needs the live KDE Wayland session.

Drive the client headlessly via CLI (already paired with localhost):

```bash
timeout 25 app/moonlight stream localhost Test \
  --video-codec PyroWave --yuv444 --performance-overlay --1080 --bitrate 150000 > /tmp/ml.log 2>&1
```

- **Stream the app named "Test", not "Desktop"** — Desktop launch fails on this host.
- `--video-codec` choices: auto, H.264, HEVC, AV1, PyroWave. `--yuv444`/`--no-yuv444`.
- Key evidence lines: client `Video stream is WxHxFPS (format 0xNN)` (0x10=PyroWave 420,
  0x20=PyroWave 444) and `PyroWave GPU zero-copy decoder ready: WxH 4:x:x`;
  host log `PyroWave encoder ready (GPU): WxH 4:x:x budget N bytes/frame`.
- Screenshot mid-stream: `spectacle -b -n -o out.png` (KDE Wayland; grim not available).
- Server codec advertisement check (no auth needed):
  `curl -s http://localhost:47989/serverinfo` → `ServerCodecModeSupport` decimal
  (SCM_PYROWAVE=0x00800000, SCM_PYROWAVE_444=0x01000000).
- Benign noise to ignore: `amdgpu_query_info(ACCEL_WORKING) failed (-13)` (vaapi probe),
  portal `Could not register app ID`, one `connect() failed: 111` (address fallback).
