TEMPLATE = subdirs
CONFIG += ordered
framing.file = $$PWD/framing.pro
profiles.file = $$PWD/profiles.pro
SUBDIRS = profiles framing
macx {
    metalcodec.file = $$PWD/metalcodec.pro
    metalcodec.makefile = Makefile.metalcodec
    metalroundtrip.file = $$PWD/metalroundtrip.pro
    metalroundtrip.depends = metalcodec
    hdr.file = $$PWD/../hdr/hdr.pro
    SUBDIRS += metalcodec metalroundtrip hdr
}
