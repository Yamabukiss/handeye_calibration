QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    delegate.cpp \
    imageproc.cpp \
    main.cpp \
    handeye.cpp \
    sensor.cpp \
    CallOneTimes.cpp \
    utils.cpp \
    verification.cpp \
    searcher.cpp \
    solver.cpp

HEADERS += \
    delegate.h \
    handeye.h \
    imageproc.h \
    sensor.h \
    CallOneTimes.h \
    utils.h \
    verification.h \
    searcher.h \
    solver.h

FORMS += \
    handeye.ui \
    verification.ui

INCLUDEPATH += D:\opencv\opencv\build\include
INCLUDEPATH +=  $$PWD/SR_SdkDllx64
INCLUDEPATH += D:\eigen\eigen-3.4.0

LIBS += D:\opencv\opencv\build\x64\vc14\lib\opencv*.lib
LIBS        +=  -L$$PWD/SR_SdkDllx64 \
                -lSR7Link

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
