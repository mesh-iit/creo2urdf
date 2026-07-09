# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

cmake_minimum_required(VERSION 3.13)
project(creo2urdf LANGUAGES C CXX
                  VERSION 0.5.18)

include(GNUInstallDirs)
include(FeatureSummary)

find_package(Eigen3 REQUIRED)
find_package(iDynTree 15.0.0 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(LibXml2 REQUIRED)

find_path(RAPIDCSV_INCLUDE_DIRS "rapidcsv.h")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")


# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(BUILD_SHARED_LIBS ON)
# Build position independent code.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Disable C and C++ compiler extensions.
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set manifest mode to use vcpkg.json to resolve dependencies
option(VCPKG_MANIFEST_MODE "Enable manifest mode to use vcpkg.json" ON)

# When ON, vcpkg install is ran every time configure is ran
option(VCPKG_MANIFEST_INSTALL "Install dependencies in manifest mode" ON)

# Force use of following triplet since thats the only config supported by Creo
set(VCPKG_TARGET_TRIPLET "x64-windows-static-md")

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.19)
  cmake_policy(SET CMP0111 NEW)
endif()

# Encourage user to specify a build type (e.g. Release, Debug, etc.), otherwise set it to Release.
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

# We cannot compile in debug CREO Object Toolkit does not supports flags '/MDd' and '/MTd'
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message( FATAL_ERROR "creo2urdf can't be compiled in Debug" )
endif()


#### Dependencies

# FIXME add find_package CREO9

if (NOT "$ENV{CREO_INSTALL_PATH}" STREQUAL "")
  set(CREO_INSTALL_PATH "$ENV{CREO_INSTALL_PATH}" CACHE INTERNAL "Copied from environment variable CREO_INSTALL_PATH")
else()
  if (NOT "$ENV{CREO9_INSTALL_PATH}" STREQUAL "")
    message(WARNING "CREO9_INSTALL_PATH is deprecated. Please use CREO_INSTALL_PATH instead.")
    set(CREO_INSTALL_PATH "$ENV{CREO9_INSTALL_PATH}" CACHE INTERNAL "Copied from deprecated environment variable CREO9_INSTALL_PATH")
  else()
    message(FATAL_ERROR "CREO_INSTALL_PATH not set")
  endif()
endif()

message("CREO_INSTALL_PATH = ${CREO_INSTALL_PATH}")


#### Optional Dependencies

feature_summary(WHAT ALL INCLUDE_QUIET_PACKAGES)

add_subdirectory(src)

option(BUILD_EXAMPLES "Build the examples" ON)
option(BUILD_TESTING "Create tests using CMake" OFF)

##############################
########### Test #############
##############################

set_property(GLOBAL PROPERTY USE_FOLDERS 1)

if (BUILD_TESTING)
  include( CTest )
  enable_testing()
  add_subdirectory(tests)
endif()
