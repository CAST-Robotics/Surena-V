##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(surena_eth)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS  roscpp sensor_msgs interactive_markers  visualization_msgs  tf   message_generation message_runtime robot_teleop)
set(LIBUSB_1_LIBRARIES "/usr/lib/x86_64-linux-gnu/libusb-1.0.so")
set(QSLOG_LIBRARIES  ${PROJECT_SOURCE_DIR}/src/QsLog/build-QsLogShared/libQsLog.so)
include_directories(include ${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything.

#add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msg FILES num.msg)
add_service_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/srv FILES active_csp.srv)
add_service_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/srv FILES reset_node.srv)
add_service_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/srv FILES  home.srv)
### Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
  # std_srvs
 )



catkin_package()
message(${CMAKE_CURRENT_SOURCE_DIR})
##############################################################################
# Qt Environment
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
#set(CMAKE_VERBOSE_MAKEFILE ON)
#SET(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport Network Qt5Test)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)


QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Network_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5PrintSupport_INCLUDE_DIRS}
    ${Qt5Test_INCLUDE_DIRS}
    )

add_definitions( -std=c++11 -fPIC)
##############################################################################
# Sources
##############################################################################

set( QT_SOURCES
    src/main.cpp
    src/VisualizeControl.cpp
    src/visualize.cpp
    src/visualize.h
    src/can.cpp
    src/can.h
    src/epos.cpp
    src/epos.h
    src/udphandler.cpp
    src/udphandler.h
    src/robot.cpp
    src/robot.h
    src/qnode.h
    src/qnode.cpp
    src/pingmodel.cpp
    src/pingmodel.h
    src/pidcontroller.cpp
    src/pidcontroller.h
        )
##############################################################################
# Binaries
##############################################################################

add_executable(surena_eth ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(surena_eth
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Network
    Qt5::Widgets
    Qt5::PrintSupport
    ${LIBUSB_1_LIBRARIES}
    ${QSLOG_LIBRARIES}
)
install(TARGETS surena_eth RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


