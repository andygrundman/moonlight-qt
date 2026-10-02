TEMPLATE = app
TARGET = tst_framequeue
QT -= gui core
CONFIG += console c++17
CONFIG -= app_bundle
INCLUDEPATH += $$PWD/../../app $$PWD/../../moonlight-common-c/moonlight-common-c/src
SOURCES += $$PWD/tst_framequeue.cpp \
           $$PWD/../../app/streaming/video/ffmpeg-renderers/framepacing/framequeue.cpp
macx {
    INCLUDEPATH += $$PWD/../../libs/mac/include $$PWD/../../libs/mac/include/SDL2
    LIBS += -L$$PWD/../../libs/mac/lib -lavutil.60 -lSDL2
    QMAKE_RPATHDIR += $$PWD/../../libs/mac/lib
} else {
    CONFIG += link_pkgconfig
    PKGCONFIG += libavutil sdl2
}
