# Match qmake's macOS target selection, including native and universal builds.
macx {
    PYROWAVE_TARGET_ARCHS = $$QMAKE_APPLE_DEVICE_ARCHS
    isEmpty(PYROWAVE_TARGET_ARCHS) {
        only_active_arch: PYROWAVE_TARGET_ARCHS = $$system(uname -m)
        else: PYROWAVE_TARGET_ARCHS = $$QT_ARCHS
    }

    contains(PYROWAVE_TARGET_ARCHS, arm64): CONFIG += pyrowave-metal
}
