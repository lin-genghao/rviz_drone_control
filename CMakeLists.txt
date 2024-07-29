cmake_minimum_required(VERSION 3.0.2)
project(rviz_drone_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS true)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  mavros
  mavros_msgs
  libmavconn
)

find_package(OGRE QUIET COMPONENTS Overlay)
if(OGRE_FOUND)
  if(${OGRE_VERSION} VERSION_LESS 1.11)
    set(OGRE_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS} ${OGRE_Overlay_INCLUDE_DIRS})
    set(OGRE_LIBRARIES ${OGRE_LIBRARIES} ${OGRE_Overlay_LIBRARIES})
  endif()
else()
  # OGRE doesn't come with correctly installed cmake files on Linux. Use pkg-config instead.
  # We need both, OGRE and OGRE-Overlay. Look for both simulatenously and use prefix X_
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(X REQUIRED OGRE OGRE-Overlay)

  # Set OGRE_* variables as cmake-based find_package would do
  set(OGRE_VERSION ${X_OGRE_VERSION})
  set(OGRE_INCLUDE_DIRS ${X_INCLUDE_DIRS})
  message(STATUS "OGRE_INCLUDE_DIRS=${OGRE_INCLUDE_DIRS}")

  # Find absolute path of OGRE libraries.
  # This is stored in the cache to save time on cmake re-run.
  # Using absolute paths is necessary if pkg-config finds Ogre in a different
  # location than the default.  This can happen when Ogre is built from source,
  # or when 2 versions of Ogre are installed.  Using absolute paths ensures that
  # components that link against Ogre (rviz and any packages depending on rviz)
  # all find the same Ogre shared library.
  if(NOT DEFINED OGRE_LIBRARIES)
    set(_OGRE_LIBRARIES)
    foreach(_lib ${X_LIBRARIES})
      set(_lib_tag "OGRE_RVIZ_${_lib}")

      find_library(${_lib_tag}
              NAMES ${_lib}
              HINTS ${X_LIBRARY_DIRS}
              PATHS ${X_LIBRARY_DIRS}
      )

      list(APPEND _OGRE_LIBRARIES ${${_lib_tag}})
    endforeach(_lib)

    set(OGRE_LIBRARIES ${_OGRE_LIBRARIES} CACHE STRING "Absolute paths to OGRE libs")
  endif(NOT DEFINED OGRE_LIBRARIES)
  message(STATUS "OGRE_LIBRARIES=${OGRE_LIBRARIES}")

  ## Fetch OGRE_PLUGIN_DIR variable from pkg-config
  if(NOT DEFINED OGRE_PLUGIN_DIR)
    execute_process(COMMAND
      ${PKG_CONFIG_EXECUTABLE} --variable=plugindir OGRE
      OUTPUT_VARIABLE OGRE_PLUGIN_DIR
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
  endif(NOT DEFINED OGRE_PLUGIN_DIR)
  message(STATUS "OGRE_PLUGIN_DIR=${OGRE_PLUGIN_DIR}")
endif()

# TODO: adapt version after upgrade to newer OGRE release
# Consider deprecation issues as warnings, not as errors for OGRE >= 1.10
if(NOT OGRE_VERSION VERSION_LESS "1.10.0" AND NOT MSVC)
  add_compile_options(-Wno-error=deprecated-declarations)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rviz_drone_control
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

# 添加 uav_button 库
# add_library(uav_button src/uav_button.cpp)
# add_dependencies(uav_button ${catkin_EXPORTED_TARGETS})
# target_link_libraries(uav_button ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_library(rviz_drone_control src/rviz_drone_control.cpp include/rviz_drone_control/rviz_drone_control.h include/rviz_drone_control/httplib.h)
add_dependencies(rviz_drone_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rviz_drone_control ${QT_LIBRARIES} ${catkin_LIBRARIES})
