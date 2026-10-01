TEMPLATE = subdirs
include(pyrowave/build-config.pri)

SUBDIRS = \
    moonlight-common-c \
    qmdnsengine \
    app \
    h264bitstream \
    imgui

# Build the dependencies in parallel before the final app
app.depends = qmdnsengine moonlight-common-c h264bitstream imgui
macx:pyrowave-metal {
    SUBDIRS += pyrowave
    app.depends += pyrowave
}
win32:!winrt {
    SUBDIRS += AntiHooking
    app.depends += AntiHooking
}

# Support debug and release builds from command line for CI
CONFIG += debug_and_release

# Run our compile tests
load(configure)
qtCompileTest(SL)
qtCompileTest(EGL)
