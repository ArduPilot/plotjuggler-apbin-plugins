cmake_minimum_required(VERSION 3.7)
cmake_policy(SET CMP0068 NEW)
project(plotjuggler_apbin_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#--- Basic Qt setup
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5 REQUIRED COMPONENTS Core Concurrent Widgets Xml Svg)

#--- Find PlotJuggler and its dependencies, and set paths
if(CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    # --- Build with CATKIN (ROS1) ---
    message(STATUS "COMPILING_WITH_CATKIN")
    find_package(catkin REQUIRED COMPONENTS plotjuggler)
    include_directories(${catkin_INCLUDE_DIRS})
    set(PJ_LIBS ${catkin_LIBRARIES})

elseif(DEFINED ENV{AMENT_PREFIX_PATH})
    # --- Build with AMENT (ROS2) ---
    message(STATUS "COMPILING_WITH_AMENT")
    find_package(ament_cmake REQUIRED)
    find_package(plotjuggler REQUIRED)
    include_directories(${plotjuggler_INCLUDE_DIR})
    set(PJ_LIBS ${plotjuggler_LIBRARIES})
else()
    # --- Build without any ROS support (standalone) ---
    message(STATUS "COMPILING STANDALONE")
    find_package(plotjuggler REQUIRED)
    # This is the critical line for moc to find the PJ::DataLoader interface
    include_directories(${plotjuggler_INCLUDE_DIR})
    set(PJ_LIBS ${plotjuggler_LIBRARIES})
endif()


#-------------- Switch units support ----------------
OPTION(ADD_UNITS "Add units support" OFF)
IF(ADD_UNITS)
    add_compile_definitions("LABEL_WITH_UNIT")
    message(STATUS "Enabling field units define.")
ENDIF(ADD_UNITS)


#------- Create the library -------
add_library(DataAPBin SHARED
    DataLoadAPBin/dataload_apbin.h
    DataLoadAPBin/dataload_apbin.cpp
)

# Link against both plotjuggler libraries and the required Qt modules.
target_link_libraries(DataAPBin PRIVATE
    ${PJ_LIBS}
    Qt5::Core
    Qt5::Concurrent
    Qt5::Widgets
    Qt5::Xml
    Qt5::Svg
)

if (COMPILING_WITH_AMENT)
    ament_target_dependencies(DataAPBin plotjuggler)
endif()

# We need to tell Qt this is a plugin.
target_compile_definitions(DataAPBin PRIVATE QT_PLUGIN)


#------- Installation and RPATH for macOS -------
set(CMAKE_MACOSX_RPATH ON)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "@executable_path/../Frameworks")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)

set_target_properties(DataAPBin PROPERTIES
    INSTALL_NAME_DIR "@rpath"
)

if(CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    set(PJ_PLUGIN_INSTALL_DIRECTORY ${CATKIN_PACKAGE_BIN_DESTINATION})
elseif(DEFINED ENV{AMENT_PREFIX_PATH})
    set(PJ_PLUGIN_INSTALL_DIRECTORY lib/${PROJECT_NAME})
else()
    set(PJ_PLUGIN_INSTALL_DIRECTORY bin)
endif()

install(
    TARGETS
        DataAPBin
    DESTINATION
        ${PJ_PLUGIN_INSTALL_DIRECTORY}  )

if (COMPILING_WITH_AMENT)
    ament_package()
endif()
    TARGETS DataAPBin
    DESTINATION ${PJ_PLUGIN_INSTALL_DIRECTORY}
)
