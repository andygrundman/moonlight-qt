#!/bin/bash
# Background HDR verification. No streaming session or display-mode changes.
set -euo pipefail
SOURCE_ROOT=$(cd "$(dirname "$0")/.." && pwd)
QT_BIN=${QT_BIN:-"$SOURCE_ROOT/build/Qt/6.11.1/macos/bin"}
TEST_FOLDER=${HDR_TEST_FOLDER:-"$SOURCE_ROOT/build/tests-hdr"}
mkdir -p "$TEST_FOLDER"
"$QT_BIN/qmake" "$SOURCE_ROOT/tests/hdr/hdr.pro" -o "$TEST_FOLDER/Makefile" \
    CONFIG+=release QMAKE_APPLE_DEVICE_ARCHS=arm64 QMAKE_MACOSX_DEPLOYMENT_TARGET=13.0 \
    > "$TEST_FOLDER/qmake.log" 2>&1
nice -n 15 make -C "$TEST_FOLDER" -j"${JOBS:-2}" > "$TEST_FOLDER/build.log" 2>&1
# Set DYLD after launching nice; macOS strips DYLD variables from system tools.
nice -n 15 env DYLD_LIBRARY_PATH="$SOURCE_ROOT/libs/mac/lib" \
    "$TEST_FOLDER/tst_vthdr" "$SOURCE_ROOT/app/shaders/vt_renderer.metal" \
    | tee "$TEST_FOLDER/results.log"
