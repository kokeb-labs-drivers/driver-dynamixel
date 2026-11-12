#===============================================================================
# DYNAMIXEL driver: root CMake
#===============================================================================
# One file, two build systems:
#
# - ESP-IDF (ESP_PLATFORM): this folder is an ESP-IDF component. Point
#   EXTRA_COMPONENT_DIRS at it, check it out under components/, or add it as a
#   component-manager git dependency. menuconfig reads _config/Kconfig.
# - Any other CMake build (host, STM32, RP2040, nRF, ...): one INTERFACE target,
#   dynamixel, consumed as dynamixel::dynamixel both through add_subdirectory() and
#   through find_package(dynamixel) after install.
#
#   cmake -S . -B build -DDYNAMIXEL_BUILD_TESTS=ON
#   cmake --build build && ctest --test-dir build --output-on-failure
#===============================================================================
cmake_minimum_required(VERSION 3.16)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/dynamixel_build_settings.cmake)

#===============================================================================
# ESP-IDF component
#===============================================================================
if(ESP_PLATFORM)
    idf_component_register(
        SRCS ${DYNAMIXEL_SOURCE_FILES}
        INCLUDE_DIRS ${DYNAMIXEL_PUBLIC_INCLUDE_DIRS}
        REQUIRES ${DYNAMIXEL_IDF_REQUIRES}
        KCONFIG ${CMAKE_CURRENT_LIST_DIR}/_config/Kconfig
    )
    if(DYNAMIXEL_SOURCE_FILES)
        target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_20)
    else()
        target_compile_features(${COMPONENT_LIB} INTERFACE cxx_std_20)
    endif()
    return()
endif()

project(dynamixel_driver
    VERSION ${DYNAMIXEL_VERSION}
    DESCRIPTION "C++20 driver for the ROBOTIS DYNAMIXEL servo protocols 1.0 and 2.0 over a half-duplex UART."
    LANGUAGES CXX)

include(GNUInstallDirs)

# ISO C++20 for every target this project builds (tests, fuzzers, examples), never the GNU dialect
# that CMake picks by default (handbook KB-CPP-001, KB-CPP20-001). CXX_EXTENSIONS does not travel
# through an INTERFACE target, so each folder that builds a target sets it as well.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The installed layout keeps the source tree's inc/ + src/ pair, because dynamixel.hpp includes
# "../src/dynamixel.ipp" relative to itself:
#   <prefix>/include/dynamixel/inc/*.hpp, dynamixel_version.h
#   <prefix>/include/dynamixel/src/dynamixel.ipp
set(DYNAMIXEL_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}/dynamixel/inc")

#===============================================================================
# Target
#===============================================================================
add_library(${DYNAMIXEL_TARGET_NAME} INTERFACE)
target_include_directories(${DYNAMIXEL_TARGET_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/inc>
        $<BUILD_INTERFACE:${DYNAMIXEL_VERSION_HEADER_DIR}>
        $<INSTALL_INTERFACE:${DYNAMIXEL_INSTALL_INCLUDEDIR}>
)
target_compile_features(${DYNAMIXEL_TARGET_NAME} INTERFACE cxx_std_20)

# dynamixel::dynamixel everywhere: the alias for add_subdirectory(), EXPORT_NAME for the installed package.
add_library(dynamixel::dynamixel ALIAS ${DYNAMIXEL_TARGET_NAME})
set_target_properties(${DYNAMIXEL_TARGET_NAME} PROPERTIES EXPORT_NAME dynamixel)

option(DYNAMIXEL_ENABLE_WARNINGS "Add -Wall -Wextra -Wpedantic to every target that links dynamixel::dynamixel" OFF)
if(DYNAMIXEL_ENABLE_WARNINGS)
    target_compile_options(${DYNAMIXEL_TARGET_NAME} INTERFACE -Wall -Wextra -Wpedantic)
endif()

#===============================================================================
# Host tests, documentation snippets, install check (tests/)
#===============================================================================
option(DYNAMIXEL_BUILD_TESTS "Build the host tests in tests/" OFF)
option(DYNAMIXEL_BUILD_FUZZERS "Also build the libFuzzer targets in tests/fuzz/ (Clang only)" OFF)
if(DYNAMIXEL_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

#===============================================================================
# Install and package export
#===============================================================================
include(CMakePackageConfigHelpers)

install(TARGETS ${DYNAMIXEL_TARGET_NAME} EXPORT dynamixelTargets
    INCLUDES DESTINATION ${DYNAMIXEL_INSTALL_INCLUDEDIR})

install(DIRECTORY inc/ DESTINATION ${DYNAMIXEL_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(FILES "${DYNAMIXEL_VERSION_HEADER}" DESTINATION ${DYNAMIXEL_INSTALL_INCLUDEDIR})
install(DIRECTORY src/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dynamixel/src
    FILES_MATCHING PATTERN "*.ipp")

install(EXPORT dynamixelTargets
    FILE dynamixelTargets.cmake
    NAMESPACE dynamixel::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dynamixel)

configure_package_config_file(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/dynamixelConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/dynamixelConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dynamixel)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/dynamixelConfigVersion.cmake"
    VERSION ${DYNAMIXEL_VERSION}
    COMPATIBILITY SameMajorVersion
    ARCH_INDEPENDENT)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/dynamixelConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/dynamixelConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/dynamixel)
