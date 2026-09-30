TEMPLATE = app
TARGET = tst_pyrowavemetal
QT = core
CONFIG += console c++17
CONFIG -= app_bundle
INCLUDEPATH += $$PWD/../../app $$PWD/../../pyrowave-metal/pyrowave/metal \
               $$PWD/../../libs/mac/include $$PWD/../../libs/mac/include/SDL2
OBJECTIVE_SOURCES += $$PWD/tst_pyrowavemetal.mm $$PWD/../../app/streaming/video/pyrowave/pyrowavemetal.mm
SOURCES += $$PWD/../../app/streaming/video/pyrowave/pyrowaveframing.cpp $$PWD/../../app/path.cpp
LIBS += -L$$OUT_PWD -lpyrowave-metal -L$$PWD/../../libs/mac/lib -lSDL2 -lavutil.60 -framework Metal -framework IOSurface -framework Foundation -lobjc
PRE_TARGETDEPS += $$OUT_PWD/libpyrowave-metal.a
QMAKE_RPATHDIR += $$PWD/../../libs/mac/lib
RESOURCES += $$PWD/metaltest.qrc

DEFINES += PYROWAVE_METAL_TEST
