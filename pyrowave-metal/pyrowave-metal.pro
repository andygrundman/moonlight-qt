# Native macOS Metal codec. Upstream revision and license: README.md.
QT -= core gui

TARGET = pyrowave-metal
TEMPLATE = lib

# Build a static library
CONFIG += staticlib c++17

# Third-party code: keep its warnings out of our build logs
CONFIG += warn_off

# Include global qmake defs
include(../globaldefs.pri)

macx {
    PW_METAL = $$PWD/pyrowave/metal
    INCLUDEPATH += $$PW_METAL
    OBJECTIVE_SOURCES += $$PW_METAL/pyrowave_common.mm $$PW_METAL/pyrowave_decoder.mm $$PW_METAL/pyrowave_encoder.mm
    SOURCES += $$PW_METAL/pyrowave_bitstream.cpp
    QMAKE_CXXFLAGS += -fobjc-arc
    LIBS += -framework Metal -framework IOSurface
}
