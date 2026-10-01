QT -= core gui

include(build-config.pri)
!pyrowave-metal: error("PyroWave Metal requires an arm64 macOS target")

TARGET = pyrowave-metal
TEMPLATE = lib

CONFIG += staticlib c++14

include(../globaldefs.pri)

# Universal Moonlight builds link this archive only into their arm64 slice.
QMAKE_APPLE_DEVICE_ARCHS = arm64

# Keep ARC local to this library; Moonlight's Metal renderer uses manual ownership.
QMAKE_OBJECTIVE_CFLAGS += -fobjc-arc
QMAKE_CXXFLAGS += -Wshadow -fvisibility=hidden
DEFINES += PYROWAVE_EXPORT_SYMBOLS

PYROWAVE_METAL_DIR = $$PWD/pyrowave/metal
INCLUDEPATH += $$PYROWAVE_METAL_DIR

SOURCES += $$PYROWAVE_METAL_DIR/pyrowave_bitstream.cpp

OBJECTIVE_SOURCES += \
    $$PYROWAVE_METAL_DIR/pyrowave_common.mm \
    $$PYROWAVE_METAL_DIR/pyrowave_decoder.mm

# Shader sources are embedded in this committed header; no shader tools are needed.
HEADERS += \
    $$PYROWAVE_METAL_DIR/pyrowave_metal.h \
    $$PYROWAVE_METAL_DIR/pyrowave_common.hpp \
    $$PYROWAVE_METAL_DIR/pyrowave_bitstream.hpp \
    $$PYROWAVE_METAL_DIR/shaders/pyrowave_msl.h

LIBS += -framework Foundation -framework Metal
