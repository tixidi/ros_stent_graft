TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++11

CONFIG(debug, debug|release) {
    DESTDIR = ./debug
    OBJECTS_DIR = ./debug/obj
    MOC_DIR = ./debug/moc
    UI_DIR = $$MOC_DIR
} else {
    DESTDIR = ./release
    OBJECTS_DIR = ./release/obj
    MOC_DIR = ./release/moc
    UI_DIR = $$MOC_DIR
}
CONFIG(debug, release|debug):   DEFINES += _DEBUG
CONFIG(release, release|debug): DEFINES += _RELEASE

SOURCES += main.cpp

LIBS += -lpthread

include(/home/yang/workspace/IIWa/IIWA_Sunrise/IIWA_Sunrise.pri)
include(/home/yang/workspace/IIWa/PathPlanners/PathPlanners.pri)

INCLUDEPATH += /home/yang/workspace/IIWa/Erl
INCLUDEPATH += /home/yang/workspace/IIWa/Eigen-3.3.3
INCLUDEPATH += /home/yang/workspace/IIWa/asio/include
INCLUDEPATH += /home/yang/workspace/IIWa/ReflexxesTypeII/include
LIBS += -L/home/yang/workspace/IIWa/ReflexxesTypeII/Linux/x64/release/lib/shared -lReflexxesTypeII
