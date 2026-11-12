#===============================================================================
# DYNAMIXEL driver: build settings
#===============================================================================
# Read by the root CMakeLists.txt in both of its modes (plain CMake and the
# ESP-IDF component): target name, version, include directories, sources and
# ESP-IDF requirements. This file is the single source
# of the version; tests/CMakeLists.txt checks every other copy against it.
#===============================================================================

include_guard(GLOBAL)

set(DYNAMIXEL_TARGET_NAME "dynamixel")

#===============================================================================
# Version (single source of truth)
#===============================================================================
set(DYNAMIXEL_VERSION_MAJOR 0)
set(DYNAMIXEL_VERSION_MINOR 1)
set(DYNAMIXEL_VERSION_PATCH 0)
set(DYNAMIXEL_VERSION "${DYNAMIXEL_VERSION_MAJOR}.${DYNAMIXEL_VERSION_MINOR}.${DYNAMIXEL_VERSION_PATCH}")

#===============================================================================
# Generated version header (build tree, never the source tree)
#===============================================================================
set(DYNAMIXEL_VERSION_TEMPLATE "${CMAKE_CURRENT_LIST_DIR}/../inc/dynamixel_version.h.in")
set(DYNAMIXEL_VERSION_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/dynamixel_generated")
set(DYNAMIXEL_VERSION_HEADER "${DYNAMIXEL_VERSION_HEADER_DIR}/dynamixel_version.h")

# The directory must exist before ESP-IDF's idf_component_register() checks the include paths.
file(MAKE_DIRECTORY "${DYNAMIXEL_VERSION_HEADER_DIR}")
configure_file("${DYNAMIXEL_VERSION_TEMPLATE}" "${DYNAMIXEL_VERSION_HEADER}" @ONLY)

#===============================================================================
# Public include directories, sources, ESP-IDF requirements
#===============================================================================
set(DYNAMIXEL_PUBLIC_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../inc"
    "${DYNAMIXEL_VERSION_HEADER_DIR}"
)

# Header-only: the template implementation is src/dynamixel.ipp, included by inc/dynamixel.hpp.
set(DYNAMIXEL_SOURCE_FILES)

set(DYNAMIXEL_IDF_REQUIRES)
