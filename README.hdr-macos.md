# HDR color verification on macOS

The native Metal renderer receives the host desktop's encoded color values.
SDR applications composited into an HDR desktop are already represented in
that desktop's PQ signal. Their values must be preserved, without adding a
second SDR conversion or a contrast adjustment on the client.

The ordinary HDR path keeps 10-bit PQ values and tags the layer with the
corresponding RGB primaries and transfer function. The optional linear EDR
path decodes PQ to absolute luminance, normalizes by its reference white, and
lets Core Animation perform the only tone mapping. The shader does not
compress highlights before Core Animation. Missing mastering metadata uses
a documented 1000-nit luminance fallback in this optional path. This cannot
recover mastering information the host did not send.

Color conversion uses float precision. P010 and software 10-bit planes are
normalized according to their storage packing; native PyroWave textures are
already normalized. Explicit primaries and transfer tags are handled
independently of YUV matrix coefficients. A transfer, bit-depth or display
reference-white change updates the layer configuration. A pixel-format
change discards the cached drawable before rendering the next frame.

Frame-synchronized HDR metadata takes priority over the host's asynchronous
fallback. Only the render thread owns CoreFoundation metadata. SDR frames
clear HDR metadata, including attachments on recycled CoreVideo buffers.
SDR overlays are converted to the video's gamut and reference-white level
before being rendered in PQ, linear EDR or HLG targets.

## Background checks

```sh
scripts/test-macos-hdr.sh
```

The test creates no window, drawable, NSApplication or streaming session. It
does not capture input, change settings, change display brightness or modes,
or contact the host. It submits small offscreen Metal draws and reads back
their pixels. Compilation runs with two workers at reduced priority by
default. `QT_BIN`, `HDR_TEST_FOLDER` and `JOBS` can override the local defaults.

It verifies production shaders and color policy using independent PQ and
Kr/Kb reference calculations, SDR/HDR color patches, 8/10-bit limited/full
range, planar/biplanar textures, native/P010/software storage, 100/203-nit
reference white, display-headroom independence, overlay conversion, metadata
precedence/serialization/removal and SDR/PQ/HLG format transitions. The HDR
test is also included in the opt-in macOS PyroWave test project.

The separate native PyroWave GPU round trip covers codec negotiation,
framing, decoding, GPU synchronization and retained surface lifetime.
These tests verify the client signal before display composition. They do
not measure physical display luminance or validate every host application's
capture/color management. Those require a controlled display and live host
content. A second hidden stream is not a safe substitute because it can
change the host's session, resolution, HDR state or active application.

Apple documents the two display paths in
[Using color spaces to display HDR content](https://developer.apple.com/documentation/metal/using-color-spaces-to-display-hdr-content)
and [Using system tone mapping on video content](https://developer.apple.com/documentation/metal/using-system-tone-mapping-on-video-content).
