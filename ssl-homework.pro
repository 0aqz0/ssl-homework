QT += gui network

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp \
    udpreceiver.cpp \
    vision_detection.pb.cc \
    grSim_Commands.pb.cc \
    grSim_Packet.pb.cc \
    grSim_Replacement.pb.cc \
    udpsender.cpp

HEADERS += \
    udpreceiver.h \
    vision_detection.pb.h \
    grSim_Commands.pb.h \
    grSim_Packet.pb.h \
    grSim_Replacement.pb.h \
    udpsender.h

INCLUDEPATH += \
    C:\Users\0AQZ0\Desktop\homework\protobuf\protobuf\include \
#    ..\protobuf\protobuf\include

LIBS += \
    C:\Users\0AQZ0\Desktop\homework\protobuf\protobuf\lib\x64\libprotobuf.lib \
#    ..\protobuf\protobuf\lib\x64\libprotobuf.lib
