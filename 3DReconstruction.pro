#-------------------------------------------------
#
# Project created by QtCreator 2018-02-18T14:55:14
#
#-------------------------------------------------

QT       += core gui xml opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 3DReconstruction
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

QMAKE_CXXFLAGS = -fpermissive -O3 -msse4.2

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    nicptrackerapp.cpp

HEADERS += \
        mainwindow.h \
    standardcamera.h \
    gluwrapper.h \
    nicptrackerappviewer.h \
    nicptrackerapp.h

FORMS += \
        mainwindow.ui

INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /home/thiago/Libraries/nicp/nicp
INCLUDEPATH += /usr/include/QGLViewer
LIBS += -lglut -lGLU
LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui
LIBS += -L/home/thiago/Libraries/nicp/lib -lnicp -lnicp_viewer
LIBS += -lQGLViewer
