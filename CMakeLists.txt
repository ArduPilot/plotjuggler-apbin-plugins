cmake_minimum_required(VERSION 3.22)
project(plotjuggler_apbin_plugins VERSION 0.2.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Qt6 REQUIRED COMPONENTS Core)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
include(CTest)

# Preserve ROS workspace registration while consuming the standalone SDK.
# PlotJuggler itself is a runtime application, not a plugin build dependency.
if(CATKIN_DEVEL_PREFIX OR catkin_FOUND OR CATKIN_BUILD_BINARY_PACKAGE)
    find_package(catkin REQUIRED)
    catkin_package()
    set(PJ_PLUGIN_INSTALL_DIRECTORY "${CATKIN_PACKAGE_BIN_DESTINATION}")
elseif(DEFINED ENV{AMENT_PREFIX_PATH})
    find_package(ament_cmake REQUIRED)
    set(COMPILING_WITH_AMENT TRUE)
    set(PJ_PLUGIN_INSTALL_DIRECTORY "lib/${PROJECT_NAME}")
else()
    set(PJ_PLUGIN_INSTALL_DIRECTORY lib)
endif()

# Consume the same SDK as PlotJuggler 4.1 (conanfile.txt), or build a local
# SDK checkout. No link to the PlotJuggler application or datastore is needed.
set(PJ_SDK_SOURCE_DIR "" CACHE PATH "Optional plotjuggler_sdk 0.40.0 source checkout")
if(PJ_SDK_SOURCE_DIR)
    set(PJ_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(PJ_BUILD_PORTED_PLUGINS OFF CACHE BOOL "" FORCE)
    add_subdirectory("${PJ_SDK_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/sdk")
else()
    if(BUILD_TESTING)
        find_package(plotjuggler_sdk 0.40.0 REQUIRED COMPONENTS plugin_sdk plugin_host)
    else()
        find_package(plotjuggler_sdk 0.40.0 REQUIRED COMPONENTS plugin_sdk)
    endif()
endif()

option(ADD_UNITS "Append units to field names" OFF)
option(ADD_RCOU_FUNCTION_LABELS "Append servo functions to RCOU/RCO2 field names" OFF)

file(GLOB LOGMSG_XML CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/DataLoadAPBin/LogMessageDescriptions/*.xml")
set(GEN_LOGMSG_HDR_OUT "${CMAKE_CURRENT_BINARY_DIR}/LogMessageDescriptions.h")
add_custom_command(OUTPUT "${GEN_LOGMSG_HDR_OUT}"
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/DataLoadAPBin/scripts/generate_LogMessageDescriptions_header.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/DataLoadAPBin/LogMessageDescriptions" "${GEN_LOGMSG_HDR_OUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/DataLoadAPBin/scripts/generate_LogMessageDescriptions_header.py" ${LOGMSG_XML}
    COMMENT "Embedding ArduPilot log message descriptions"
    VERBATIM)

add_library(DataAPBin SHARED DataLoadAPBin/dataload_apbin.cpp DataLoadAPBin/dataload_apbin.h "${GEN_LOGMSG_HDR_OUT}")
target_include_directories(DataAPBin PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_link_libraries(DataAPBin PRIVATE plotjuggler_sdk::plugin_sdk Qt6::Core)
target_compile_options(DataAPBin PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)
target_compile_definitions(DataAPBin PRIVATE
    $<$<BOOL:${ADD_UNITS}>:LABEL_WITH_UNIT>
    $<$<BOOL:${ADD_RCOU_FUNCTION_LABELS}>:LABEL_RCOU_FUNCTION>)
# The SDK installs the manifest using CMAKE_INSTALL_LIBDIR. Keep it beside the
# plugin in standalone, catkin and ament install trees.
set(CMAKE_INSTALL_LIBDIR "${PJ_PLUGIN_INSTALL_DIRECTORY}")
pj_configure_plugin(DataAPBin
    FAMILIES data_source
    MANIFEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/DataLoadAPBin/manifest.json"
    MANIFEST_HEADER generated/apbin_manifest.hpp
    MANIFEST_VAR kAPBinManifest)
install(TARGETS DataAPBin
    RUNTIME DESTINATION "${PJ_PLUGIN_INSTALL_DIRECTORY}"
    LIBRARY DESTINATION "${PJ_PLUGIN_INSTALL_DIRECTORY}")

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

if(COMPILING_WITH_AMENT)
    ament_package()
endif()
