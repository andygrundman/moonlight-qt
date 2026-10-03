TEMPLATE = app
TARGET = tst_imguiinput
QT -= gui core
CONFIG += console c++17
CONFIG -= app_bundle
INCLUDEPATH += $$PWD/../../app $$PWD/../../imgui/imgui $$PWD/../../imgui/imgui/backends
SOURCES += $$PWD/tst_imguiinput.cpp $$PWD/../../app/imgui/imgui_input.cpp \
           $$PWD/../../imgui/imgui/imgui.cpp $$PWD/../../imgui/imgui/imgui_draw.cpp \
           $$PWD/../../imgui/imgui/imgui_tables.cpp $$PWD/../../imgui/imgui/imgui_widgets.cpp \
           $$PWD/../../imgui/imgui/backends/imgui_impl_sdl2.cpp
macx {
    INCLUDEPATH += $$PWD/../../libs/mac/include $$PWD/../../libs/mac/include/SDL2
    LIBS += -L$$PWD/../../libs/mac/lib -lSDL2
    QMAKE_RPATHDIR += $$PWD/../../libs/mac/lib
}
unix:!macx {
    CONFIG += link_pkgconfig
    PKGCONFIG += sdl2
    LIBS += -lpthread
}
asan {
    QMAKE_CXXFLAGS += -fsanitize=address -fno-omit-frame-pointer
    QMAKE_CXXFLAGS_RELEASE = -O1 -g
    QMAKE_LFLAGS += -fsanitize=address
}
