QT += gui network
QT += serialport

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
    communication\udpreceiver.cpp \
    proto\vision_detection.pb.cc \
    proto\grSim_Commands.pb.cc \
    proto\grSim_Packet.pb.cc \
    proto\grSim_Replacement.pb.cc \
    communication\udpsender.cpp \
    utils/datamanager.cpp \
    algorithm/pathplanner.cpp \
    proto/zss_debug.pb.cc \
    communication\serialsender.cpp \
    algorithm/artifical_potential.cpp \
    utils/mymath.cpp

HEADERS += \
    communication\udpreceiver.h \
    proto\vision_detection.pb.h \
    proto\grSim_Commands.pb.h \
    proto\grSim_Packet.pb.h \
    proto\grSim_Replacement.pb.h \
    communication\udpsender.h \
    utils/singleton.hpp \
    utils/datamanager.h \
    algorithm/pathplanner.h \
    proto/zss_debug.pb.h \
    communication\serialsender.h \
    algorithm/artifical_potential.h \
    utils/mymath.h \
    utils/params.h

INCLUDEPATH += \
#    C:\Users\0AQZ0\Desktop\homework\protobuf\protobuf\include \
    ..\protobuf\include

LIBS += \
#    C:\Users\0AQZ0\Desktop\homework\protobuf\protobuf\lib\x64\libprotobuf.lib \
    ..\protobuf\lib\x86\libprotobuf.lib
