# Moonlight PC — Audio Jitter Buffer Fork

This is a fork of [moonlight-stream/moonlight-qt](https://github.com/moonlight-stream/moonlight-qt) (v6.1.0) with a targeted fix for audio crackling and micro-stuttering on wireless connections.

## What's different

A single setting has been added: **Audio Jitter Buffer**, visible in **Settings → Audio Settings**.

### The problem

On WiFi, audio packets from the host arrive in bursts rather than at a perfectly steady rate. The original client drops decoded audio frames whenever more than 30 ms of audio is queued in the pre-decode buffer — regardless of whether the output buffer is empty. On a congested or variable-latency wireless connection this fires constantly, creating short gaps in the audio stream that sound like crackling or micro-stuttering. Video is unaffected because it has a separate pipeline.

### The fix

The 30 ms drop threshold is now a slider. Raising it allows more packets to queue up during a burst before any are dropped, giving the decoder time to work through the backlog without punching holes in the audio stream.

## Audio Jitter Buffer slider

Found in **Settings → Audio Settings → Audio jitter buffer**.

| Value | Effect |
|---|---|
| **30 ms** (default) | Original behaviour — most aggressive drop threshold |
| **60–80 ms** | Good starting point for most WiFi connections |
| **100–150 ms** | For high-jitter or congested networks |

**What to expect when increasing the slider:**
- Crackling and micro-stutters should reduce or disappear
- A small amount of additional audio latency is introduced (equal to the extra buffer)
- Video is unaffected — A/V sync is maintained by the underlying stream

**The change takes effect the next time you start a stream.** Changing the slider mid-stream has no effect on the current session.

**If audio is still crackling** at higher values, your network jitter may be exceeding the buffer. Try 100–150 ms, or improve your wireless connection (5 GHz band, closer to the router, wired if possible).

---

## Downloads

macOS (Apple Silicon) builds are available on the [Releases](https://github.com/AmanBhardwaj25/moonlight-qt/releases) page.

For other platforms (Windows, Linux, Intel Mac, Steam Deck), use the official [moonlight-stream/moonlight-qt](https://github.com/moonlight-stream/moonlight-qt/releases) releases. The audio fix in this fork affects all platforms equally and could be applied to any Qt build.

---

## Original project

[Moonlight PC](https://moonlight-stream.org) is an open source client for NVIDIA GameStream and [Sunshine](https://github.com/LizardByte/Sunshine).

Moonlight also has mobile versions for [Android](https://github.com/moonlight-stream/moonlight-android) and [iOS](https://github.com/moonlight-stream/moonlight-ios).

### Original features
- Hardware accelerated video decoding on Windows, Mac, and Linux
- H.264, HEVC, and AV1 codec support (AV1 requires Sunshine and a supported host GPU)
- YUV 4:4:4 support (Sunshine only)
- HDR streaming support
- 7.1 surround sound audio support
- 10-point multitouch support (Sunshine only)
- Gamepad support with force feedback and motion controls for up to 16 players
- Support for both pointer capture (for games) and direct mouse control (for remote desktop)
- Support for passing system-wide keyboard shortcuts like Alt+Tab to the host

---

## Building

### macOS Build Requirements
* Qt 6.7 SDK or later
* Xcode 14 or later
* [create-dmg](https://github.com/sindresorhus/create-dmg) (only if building DMGs)

### Build steps
```bash
git submodule update --init --recursive
python3 setup-deps.py  # macOS only
# then either open in Qt Creator, or:
qmake6 moonlight-qt.pro QMAKE_APPLE_DEVICE_ARCHS=arm64
make -j$(sysctl -n hw.logicalcpu) release
```

For a distributable DMG, use `scripts/generate-dmg.sh` from the repo root with Qt's `bin` in your `$PATH`.

See the [upstream README](https://github.com/moonlight-stream/moonlight-qt) for Windows, Linux, and Steam Link build instructions.
