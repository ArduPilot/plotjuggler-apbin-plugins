cmake_minimum_required(VERSION 3.16)
project(pj_plugin_smoketest CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Deliberately no find_package(plotjuggler): the plugin under test is dlopened, so
# it carries its own copy of plotjuggler_base. All this harness needs is the
# headers that declare the DataLoader interface. Avoiding the package config also
# avoids PlotJuggler 3.16's exported target, whose link interface references
# fmt::fmt and lua::lua without ever looking them up.
set(PJ_INCLUDE_DIR "" CACHE PATH "PlotJuggler install include directory")
if(NOT PJ_INCLUDE_DIR)
    message(FATAL_ERROR "Set -DPJ_INCLUDE_DIR=<plotjuggler_install>/include")
endif()

find_package(Qt5 REQUIRED COMPONENTS Core Widgets Xml)

add_executable(pj_plugin_smoketest pj_plugin_smoketest.cpp)
target_include_directories(pj_plugin_smoketest PRIVATE ${PJ_INCLUDE_DIR})
target_link_libraries(pj_plugin_smoketest PRIVATE Qt5::Core Qt5::Widgets Qt5::Xml)
