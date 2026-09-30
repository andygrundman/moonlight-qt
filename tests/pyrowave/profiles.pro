TEMPLATE = app
TARGET = tst_pyrowaveprofiles
QT = core gui quick network
CONFIG += console c++17
CONFIG -= app_bundle
INCLUDEPATH += $$PWD/../../app $$PWD/../../libs/mac/include $$PWD/../../libs/mac/include/SDL2 \
               $$PWD/../../moonlight-common-c/moonlight-common-c/src
SOURCES += $$PWD/tst_pyrowaveprofiles.cpp
LIBS += -L$$PWD/../../libs/mac/lib -lSDL2
QMAKE_RPATHDIR += $$PWD/../../libs/mac/lib
INCLUDEPATH += $$PWD/../../qmdnsengine/qmdnsengine/src/include $$PWD/../../qmdnsengine
