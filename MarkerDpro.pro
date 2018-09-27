#-------------------------------------------------
#
# Project created by QtCreator 2016-11-05T21:58:43
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = MarkerD
TEMPLATE = app


SOURCES += main.cpp\
    markerdetection.cpp

HEADERS  += \
    markerdetection.h

FORMS    += mainwindow.ui
LIBS+= -lopencv_core
LIBS+= -lopencv_highgui
LIBS+= -lopencv_imgproc
LIBS+= -lopencv_features2d
LIBS+= -lopencv_calib3d
LIBS+= -lopencv_video
