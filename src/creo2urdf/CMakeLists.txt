# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

add_library(creo2urdf SHARED)
add_library(creo2urdf::creo2urdf ALIAS creo2urdf)

set(CREO2URDF_HDRS include/creo2urdf/Creo2Urdf.h
                   include/creo2urdf/ComponentIdentity.h
                   include/creo2urdf/Validator.h
                   include/creo2urdf/Sensorizer.h
                   include/creo2urdf/Utils.h
                   include/creo2urdf/ElementTreeManager.h
)
set(CREO2URDF_SRCS src/main.cpp
                   src/Creo2Urdf.cpp
                   src/Validator.cpp
                   src/Sensorizer.cpp
                   src/Utils.cpp
                   src/ElementTreeManager.cpp
)

set(CREO2URDF_IMPL_HDRS )
set(CREO2URDF_IMPL_SRCS )

source_group(
  TREE "${CMAKE_CURRENT_SOURCE_DIR}"
  PREFIX "Source Files"
  FILES
    ${CREO2URDF_SRCS}
    ${CREO2URDF_IMPL_SRCS}
)
source_group(
  TREE "${CMAKE_CURRENT_SOURCE_DIR}"
  PREFIX "Header Files"
  FILES
    ${CREO2URDF_HDRS}
    ${CREO2URDF_IMPL_HDRS}
)

target_sources(creo2urdf
  PRIVATE
    ${CREO2URDF_SRCS}
    ${CREO2URDF_HDRS}
    ${CREO2URDF_IMPL_SRCS}
    ${CREO2URDF_IMPL_HDRS}
)


target_include_directories(creo2urdf PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                            ${RAPIDCSV_INCLUDE_DIRS}
                                            ${CREO_INSTALL_PATH}/protoolkit/includes
                                            ${CREO_INSTALL_PATH}/otk/otk_cpp/include/
                                            ${CREO_INSTALL_PATH}/protoolkit/protk_appls/includes)

target_link_directories(creo2urdf PUBLIC    ${CREO_INSTALL_PATH}/otk/otk_cpp/x86e_win64/obj/
                                            ${CREO_INSTALL_PATH}/protoolkit/x86e_win64/obj/)

## FIXME C++ 17 triggers "byte ambigous symbol" probably std::byte clashes with PTC defines
target_compile_features(creo2urdf PUBLIC cxx_std_14)


# FIXME investigate what they are
add_compile_definitions(creo2urdf PUBLIC PRO_MACHINE=36
                                         PRO_OS=4)

# Link dependencies
target_link_libraries(creo2urdf PRIVATE iDynTree::idyntree-modelio
                                        iDynTree::idyntree-high-level
                                        iDynTree::idyntree-model
                                        yaml-cpp::yaml-cpp
                                        LibXml2::LibXml2
                                        Eigen3::Eigen
                                        protk_dllmd_NU
                                        otk_cpp_md
                                        otk_no222_md
                                        ucore
                                        udata
                                        wsock32
                                        ws2_32
                                        advapi32
                                        mpr
                                        netapi32)

# Useful global defines
add_compile_definitions(_USE_MATH_DEFINES)

# FIXME all these win32 libraries that are used by protk dll have to be set as dependencies of the target
## FIXME
list(APPEND CREO2URDF_PUBLIC_DEPS)
list(APPEND CREO2URDF_PRIVATE_DEPS)

set_target_properties(creo2urdf PROPERTIES DEFINE_SYMBOL CREO2URDF_EXPORTS)

set_property(TARGET creo2urdf PROPERTY PUBLIC_HEADER ${CREO2URDF_HDRS})
set_property(TARGET creo2urdf PROPERTY PRIVATE_HEADER ${CREO2URDF_IMPL_HDRS})
set_property(TARGET creo2urdf PROPERTY VERSION ${${CMAKE_PROJECT_NAME}_VERSION})
set_property(TARGET creo2urdf PROPERTY SOVERSION 0)
set_property(TARGET creo2urdf PROPERTY FOLDER "Libraries")

#install(
#  TARGETS creo2urdf
#  EXPORT creo2urdf
#  RUNTIME
#    DESTINATION "${CMAKE_INSTALL_BINDIR}"
#    COMPONENT creo2urdf
#  LIBRARY
#    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
#    COMPONENT creo2urdf
#    NAMELINK_COMPONENT creo2urdf-dev
#  ARCHIVE
#    DESTINATION "${CMAKE_INSTALL_LIBDIR}"
#    COMPONENT creo2urdf-dev
#  PUBLIC_HEADER
#    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/creo2urdf"
#    COMPONENT creo2urdf-dev
#  PRIVATE_HEADER
#    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/creo2urdf/impl"
#    COMPONENT creo2urdf-priv-dev
#)

set(CREO2URDF_PUBLIC_DEPS ${CREO2URDF_PUBLIC_DEPS} PARENT_SCOPE)
set(CREO2URDF_PRIVATE_DEPS ${CREO2URDF_PRIVATE_DEPS} PARENT_SCOPE)

## Generate protk.dat with correct paths
file(WRITE ${PROJECT_BINARY_DIR}/protk.dat "")
file(APPEND ${PROJECT_BINARY_DIR}/protk.dat "name creo2urdf\n")
file(APPEND ${PROJECT_BINARY_DIR}/protk.dat "Startup dll\n")
file(APPEND ${PROJECT_BINARY_DIR}/protk.dat "Allow_stop True\n")
file(APPEND ${PROJECT_BINARY_DIR}/protk.dat "Delay_start False\n")
file(APPEND ${PROJECT_BINARY_DIR}/protk.dat "exec_file ${CMAKE_BINARY_DIR}/bin/creo2urdf.dll\n")
file(APPEND ${PROJECT_BINARY_DIR}/protk.dat "text_dir ${CMAKE_CURRENT_SOURCE_DIR}/text\n")
file(APPEND ${PROJECT_BINARY_DIR}/protk.dat "END\n")
