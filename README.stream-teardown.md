# Stream shutdown and overlay input ownership

The macOS stream-quit reports detected corruption of a freed heap block on
different threads, including network and AppKit threads. That identifies where
the allocator noticed the damage, rather than the original access.

A background reproduction using the production Metal renderer and frame pacer
caught a use-after-free in `ImGui::UpdateInputEvents()`. The main SDL event thread
grew and freed ImGui's input vector while the rendering thread was reading it.
The stream's keyboard and mouse handlers also read ImGui's context without
synchronizing with context destruction.

`ImGuiInput` now copies supported SDL events into a mutex-protected queue. The
context owner drains that queue before each overlay frame. It publishes capture
flags through atomics, so the input thread never reads or changes ImGui's context.
Context shutdown disables event delivery, discards pending events and resets
capture flags before destroying the backend. The input handler is destroyed after
the decoder has stopped rendering, allowing the overlay backend to close its
gamepad handles before SDL's input subsystems are stopped.

The shared input path is consumed by Metal, D3D11 and the Vulkan PyroWave overlay.
The macOS release and both architectures have been built. Windows and Linux
runtime testing was not performed in this macOS session.

## Background regression tests

```sh
scripts/test-macos-stream-teardown.sh
scripts/test-macos-hdr.sh
```

The input test uses SDL's dummy driver and Address Sanitizer. It checks keyboard
presses and releases, shortcut modifiers, capture publication and reset, 440,000
concurrent events, context destruction while events arrive, and 20 restarts.
It creates no native window and connects to no streaming host.

An additional isolated probe under `build/teardown-20261002` links production
application objects with instrumented renderer, frame-pacing, input, overlay,
audio and PyroWave application code, plus ImGui. Prebuilt Qt, SDL, FFmpeg and
codec libraries are not instrumented. It uses tiny hidden Metal windows,
synthetic HDR frames and an in-process virtual controller, with physical
controller drivers disabled. It exercises the production Ctrl+Alt+Shift+Q
handler and normal render/input/window cleanup for 20 cycles. The original
event-delivery code fails under Address Sanitizer; the corrected code passes.

Tests run at reduced priority, do not capture device input or alter display,
HDR or audio settings, and do not open or change a Jakey-PC stream. Test settings
are isolated from the installed client's preferences. The native probe's input
constructor can fetch Moonlight's standard controller mapping database.

The packaged candidate and evidence are under
`installer-macos-metal-vrr-pyrowave/hdr-candidate/stream-quit-fix`. A foreground
stream on the installed update remains the final acceptance check. The existing
HDR verification measures signal conversion before display composition, rather
than physical screen luminance.
