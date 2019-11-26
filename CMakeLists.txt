##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.1.0)

add_compile_options(-std=c++14)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_PREFIX_PATH /usr/lib/x86_64-linux-gnu/cmake/Qt5/Qt5Widgets)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

project(ros_logger_gui)
##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS
message_generation 
roscpp
geometry_msgs
sensor_msgs
mavros
nav_msgs
std_msgs
std_srvs
tf2_ros
tf2_eigen
mavros_msgs
rosbag
Qt5Core
Qt5Widgets
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
# catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/ros_logger_gui/*.hpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(ros_logger_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(ros_logger_gui ${QT_LIBRARIES} ${catkin_LIBRARIES} Qt5::Widgets)
install(TARGETS ros_logger_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

