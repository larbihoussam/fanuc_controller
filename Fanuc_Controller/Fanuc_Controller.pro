QT -= gui qt

CONFIG += c++17 console
CONFIG -= app_bundle

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH += "C:\Program Files\Webots\include\controller\c"
INCLUDEPATH += "C:\Program Files\Webots\include\controller\cpp"
INCLUDEPATH += "C:\eigen\include\Eigen"

LIBS += "C:\Program Files\Webots\lib\controller\Controller.lib"
INCLUDEPATH +="C:\Program Files\Webots\lib\controller"
HEADERS +=\
        ControlRobot.h \
 \        #track_gen.h
    track_gen.h

SOURCES += \
        ControlRobot.cpp \
        Fanuc_Controller.cpp \
        track_gen.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

LIBS += -L$$PWD/../../../../../../../ruckig/build/debug/ -lruckig
#LIBS += "C:\Program Files\Webots\lib\controller\Controller.lib"

INCLUDEPATH += $$PWD/../../../../../../../ruckig/include/

#LIBS += "C:\ruckig\build\Debug\ruckig.lib"

INCLUDEPATH += $$PWD/../../../../../../../ruckig/build/Debug
DEPENDPATH += $$PWD/../../../../../../../ruckig/build/Debug



