
cmake_minimum_required(VERSION 2.6)

message(STATUS " ------- Building HuboQt QWidgets! -------- ")

find_package(Qt4 COMPONENTS QtCore QtGui Qt3Support REQUIRED)

if(Qt4_NOT_FOUND)
  message(FATAL_ERROR "Unable to find Qt4!! We cannot compile HuboQt!")
endif(Qt4_NOT_FOUND)

add_definitions(${QT_DEFINITIONS})
include(${QT_USE_FILE})

FILE(GLOB ui_files "ui/*.ui")
QT4_WRAP_UI(ui ${ui_files})

FILE(GLOB moc_files "*.h")
QT4_WRAP_CPP(moc ${moc_files})

file(GLOB rc_files "*.qrc")
QT4_ADD_RESOURCES(resources ${rc_files})

file(GLOB huboqt_sources "src/*.cpp" "*.h")

add_library(HuboQt SHARED ${huboqt_sources} ${ui} ${moc} ${resources})


