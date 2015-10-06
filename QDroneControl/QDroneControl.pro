#-------------------------------------------------
#
# Project created by QtCreator 2015-01-07T21:21:01
#
#-------------------------------------------------
QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QDroneControl
TEMPLATE = lib
CONFIG += staticlib
DEFINES += BUILD_LIB

#TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
    #qdronecontrol.cpp \
    3rdParty/cvdrone/src/ardrone.cpp \
    3rdParty/cvdrone/src/command.cpp \
    3rdParty/cvdrone/src/config.cpp \
    3rdParty/cvdrone/src/navdata.cpp \
    3rdParty/cvdrone/src/udp.cpp \
    3rdParty/cvdrone/src/version.cpp \
    3rdParty/cvdrone/src/video.cpp \
    3rdParty/cvdrone/src/tcp.cpp

HEADERS  += mainwindow.h \
    #qdronecontrol.h \
    #idronecontrol.h \
    3rdParty/cvdrone/src/ardrone.h
    #consts.h

FORMS    += mainwindow.ui


LIBS          = -lm                     \
                -lpthread               \
                -lavutil                \
                -lavformat              \
                -lavcodec               \
                -lswscale               \
                -lopencv_calib3d        \
                -lopencv_contrib        \
                -lopencv_core           \
                -lopencv_features2d     \
                -lopencv_flann          \
                -lopencv_highgui        \
                -lopencv_imgproc        \
                -lopencv_legacy         \
                -lopencv_ml             \
                -lopencv_objdetect      \
                -lopencv_photo          \
                -lopencv_stitching      \
                -lopencv_superres       \
                -lopencv_video          \
                -lopencv_videostab
