# - try to find PlotJuggler library
#
# Cache Variables: (probably not for direct use in your scripts)
#  PlotJuggler_INCLUDE_DIR
#  PlotJuggler_LIBRARY
#
# Non-cache variables you might use in your CMakeLists.txt:
#  PlotJuggler_FOUND
#  PlotJuggler_INCLUDE_DIRS
#  PlotJuggler_LIBRARIES
#  PlotJuggler_DEFINITIONS
#  PlotJuggler_RUNTIME_LIBRARIES - aka the dll for installing
#  PlotJuggler_RUNTIME_LIBRARY_DIRS
#
# Requires these CMake modules:
#  FindPackageHandleStandardArgs (known included with CMake >=2.6.2)
#

set(PlotJuggler_ROOT_DIR
    "${PlotJuggler_ROOT_DIR}"
    CACHE
    PATH
    "Directory to search")

if(CMAKE_SIZEOF_VOID_P MATCHES "8")
    set(_LIBSUFFIXES /lib64 /lib)
else()
    set(_LIBSUFFIXES /lib)
endif()

# For old plotjuggler releases (up to version 3.2)
find_library(PlotJuggler_LIBRARY
    NAMES
    plotjuggler_plugin_base
    PATHS
    "${PlotJuggler_ROOT_DIR}"
    PATH_SUFFIXES
    "${_LIBSUFFIXES}")

# For new plotjuggler releases (from version 3.3)
if(NOT PlotJuggler_LIBRARY)
    find_library(PlotJuggler_LIBRARY
        NAMES
        plotjuggler_base
        PATHS
        "${PlotJuggler_ROOT_DIR}"
        PATH_SUFFIXES
        "${_LIBSUFFIXES}")
endif()
    
# Might want to look close to the library first for the includes.
get_filename_component(_libdir "${PlotJuggler_LIBRARY}" PATH)

find_path(PlotJuggler_INCLUDE_DIR
    NAMES
    pj_plugin.h
    plotdata.h
    plotdatabase.h
    HINTS
    "${_libdir}" # the library I based this on was sometimes bundled right next to its include
    "${_libdir}/.."
    PATHS
    "${PlotJuggler_ROOT_DIR}"
    PATH_SUFFIXES
    include/PlotJuggler)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PlotJuggler
    DEFAULT_MSG
    PlotJuggler_LIBRARY
    PlotJuggler_INCLUDE_DIR
    ${_deps_check})

if(PlotJuggler_FOUND)
    set(PlotJuggler_LIBRARIES "${PlotJuggler_LIBRARY}")
    set(PlotJuggler_INCLUDE_DIRS "${PlotJuggler_INCLUDE_DIR}/..")
    mark_as_advanced(PlotJuggler_ROOT_DIR)

endif()

mark_as_advanced(
    PlotJuggler_INCLUDE_DIR
    PlotJuggler_LIBRARY
    PlotJuggler_RUNTIME_LIBRARY
    PlotJuggler_DEFINITIONS)

if(APPLE AND EXISTS /usr/local/opt/qt5)
    # Homebrew installs Qt5 (up to at least 5.9.1) in
    # /usr/local/qt5, ensure it can be found by CMake since
    # it is not in the default /usr/local prefix.
    # source: https://github.com/Homebrew/homebrew-core/issues/8392#issuecomment-325226494
    list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt5")
    set(CMAKE_MACOSX_RPATH 1)
endif()
