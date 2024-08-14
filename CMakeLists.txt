cmake_minimum_required(VERSION 3.7)

project(plotjuggler_apbin_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#-------  Include Qt dependencies -------
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS
    Core
    Widgets
    Xml
    Svg)

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Xml_INCLUDE_DIRS}
    ${Qt5Svg_INCLUDE_DIRS} )

set(QT_LIBRARIES
    Qt5::Core
    Qt5::Widgets
    Qt5::Xml
    Qt5::Svg )

add_definitions( ${QT_DEFINITIONS} -DQT_PLUGIN )
set( PJ_LIBRARIES ${QT_LIBRARIES} )

#-------------- Switch units support ----------------
OPTION(ADD_UNITS "Add units support" OFF)
IF(ADD_UNITS)
    add_compile_definitions("LABEL_WITH_UNIT")
    message(STATUS "Enabling field units define.")
ENDIF(ADD_UNITS)

#--------------------------------------------------------
#-------------- Build with CATKIN (ROS1) ----------------
if( CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)

    set(COMPILING_WITH_CATKIN 1)
    message(STATUS "COMPILING_WITH_CATKIN")
    add_definitions(-DCOMPILED_WITH_CATKIN)

    find_package(catkin REQUIRED COMPONENTS plotjuggler )
    catkin_package(
        CATKIN_DEPENDS plotjuggler
        )
    include_directories(${catkin_INCLUDE_DIRS})
    list(APPEND ${PJ_LIBRARIES} ${catkin_LIBRARIES} )
    set(PJ_PLUGIN_INSTALL_DIRECTORY ${CATKIN_PACKAGE_BIN_DESTINATION} )

    #--------------------------------------------------------
    #-------------- Build with AMENT (ROS2) -----------------
elseif( DEFINED ENV{AMENT_PREFIX_PATH})

    set(COMPILING_WITH_AMENT 1)
    message(STATUS "COMPILING_WITH_CATKIN")
    add_definitions(-DCOMPILED_WITH_AMENT)

    find_package(ament_cmake REQUIRED)
    find_package(plotjuggler REQUIRED)
    set(PJ_PLUGIN_INSTALL_DIRECTORY lib/${PROJECT_NAME} )

    #--------------------------------------------------------
    #------------- Build without any ROS support ------------
else()

    find_package(plotjuggler REQUIRED)
    message(STATUS "PlotJuggler FOUND")
    message(STATUS "plotjuggler_INCLUDE_DIR: ${plotjuggler_INCLUDE_DIR}")
    message(STATUS "plotjuggler_LIBRARIES: ${plotjuggler_LIBRARIES}")

    include_directories(${plotjuggler_INCLUDE_DIR})
    list(APPEND PJ_LIBRARIES ${plotjuggler_LIBRARIES} )
    set(PJ_PLUGIN_INSTALL_DIRECTORY bin )

endif()

#------- Create the libraries -------
add_library(DataAPBin SHARED
    DataLoadAPBin/dataload_apbin.h
    DataLoadAPBin/dataload_apbin.cpp )

target_link_libraries(DataAPBin
    ${PJ_LIBRARIES})

if (COMPILING_WITH_AMENT)
    ament_target_dependencies(DataAPBin plotjuggler)
endif()

#------- Install the libraries -------
install(
    TARGETS
        DataAPBin
    DESTINATION
        ${PJ_PLUGIN_INSTALL_DIRECTORY}  )
