# Native Metal dependency

`pyrowave` is an unmodified submodule of [Themaister/pyrowave](https://github.com/Themaister/pyrowave), pinned to `186f0393b77f7755953b5ecde994bb1cec2e4155`. Its MIT license is in `pyrowave/LICENSE`.

The existing top-level PyroWave submodule is kept at its original revision for the Vulkan backend. That revision does not contain the native Metal API, so this separate checkout avoids changing its Vulkan C API or Granite dependencies.

The qmake target builds the upstream Metal sources as a static library with ARC. The codec compiles its committed Metal shader source at runtime; no shader conversion tools, Vulkan loader, Granite checkout, or PyroWave dylib are needed for the native path. The encoder is included to run GPU round-trip tests.
