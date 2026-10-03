TEMPLATE = app
TARGET = tst_vthdr
QT = core qml
CONFIG += console c++17
CONFIG -= app_bundle
INCLUDEPATH += $$PWD/../../app $$PWD/../../libs/mac/include $$PWD/../../libs/mac/include/SDL2 \
               $$PWD/../../moonlight-common-c/moonlight-common-c/src
OBJECTIVE_SOURCES += $$PWD/tst_vthdr.mm
LIBS += -L$$PWD/../../libs/mac/lib -lavutil.60 -lSDL2 -framework Foundation \
        -framework CoreGraphics -framework CoreVideo -framework Metal -framework QuartzCore
QMAKE_RPATHDIR += $$PWD/../../libs/mac/lib
