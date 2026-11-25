QT += core gui network charts multimedia

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    telemetrymodel.cpp \
    mavlinkmessageparser.cpp \
    mavlinkmanager.cpp \
    uavsimulator.cpp

HEADERS += \
    mainwindow.h \
    telemetrymodel.h \
    mavlinkmessageparser.h \
    mavlinkmanager.h \
    uavsimulator.h

FORMS += \
    mainwindow.ui

RESOURCES += station.qrc
