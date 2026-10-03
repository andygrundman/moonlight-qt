#!/bin/bash
set -euo pipefail
repo=$(cd "$(dirname "$0")/.." && pwd)
qmake_bin=${QMAKE:-"$repo/build/Qt/6.11.1/macos/bin/qmake"}
test_build=${MOONLIGHT_TEARDOWN_TEST_BUILD:-"$repo/build/tests-imgui-input-asan"}
mkdir -p "$test_build"
cd "$test_build"
"$qmake_bin" "$repo/tests/imgui/input.pro" CONFIG+=release CONFIG+=asan QMAKE_APPLE_DEVICE_ARCHS=arm64
nice -n 15 make -j2
nice -n 15 env DYLD_LIBRARY_PATH="$repo/libs/mac/lib" ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 ./tst_imguiinput
