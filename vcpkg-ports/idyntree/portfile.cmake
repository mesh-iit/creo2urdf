# Based on the vcpkg idyntree port. Keep a local overlay until the registry
# provides a release containing gbionics/idyntree#1295 with JSON I/O enabled.
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO gbionics/idyntree
    REF "v${VERSION}"
    SHA512 284ca15eb9242b6e9a32d37d0ccbc06ef2e24fde218a3375920c60e9b938484c8c943780b103b5ced86fe9efe0516a9045ab76afa14906a993491c25e5990341
    HEAD_REF master
)

# 15.0.1 uses this header-only dependency through FetchContent. Download its
# sources and CMake helpers through vcpkg so configure needs no network access.
vcpkg_from_github(
    OUT_SOURCE_PATH RRUC_SOURCE_PATH
    REPO gbionics/resolve-robotics-uri-cpp
    REF v0.1.0
    SHA512 0e624ba54acbb7f0f7c695d6df17e9d97da1dbbb1e9b6918f678cef8bdb1fb3d44240ec5e07cfb341d243aa6ba4bf7f0cd894a3bf3a1d5acd1709de90591848a
)
vcpkg_from_github(
    OUT_SOURCE_PATH YCM_SOURCE_PATH
    REPO robotology/ycm
    REF v0.18.2
    SHA512 2df4554bf9cd104ddc98ea8437e265fc5098c8da9724d1cab5b850791d53118ae01818d14161093becf88adfded53fddaf0dd3f09a25a3d256794380a6ebccc2
)

# The URI resolver is only used in a .cpp file. Do not export a reference to
# its bundled, non-installed INTERFACE target in the static iDynTree package.
vcpkg_replace_string("${SOURCE_PATH}/src/core/CMakeLists.txt"
    "ResolveRoboticsURICpp::ResolveRoboticsURICpp"
    "$<BUILD_INTERFACE:ResolveRoboticsURICpp::ResolveRoboticsURICpp>"
)
# JSON is likewise a private header-only implementation dependency. Upstream
# 15.0.1 does not find nlohmann_json in its installed static package config.
vcpkg_replace_string("${SOURCE_PATH}/src/model_io/codecs/CMakeLists.txt"
    "nlohmann_json::nlohmann_json"
    "$<BUILD_INTERFACE:nlohmann_json::nlohmann_json>"
)

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        "assimp" IDYNTREE_USES_ASSIMP
        "irrlicht" IDYNTREE_USES_IRRLICHT
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        ${FEATURE_OPTIONS}
        -DIDYNTREE_USES_NLOHMANN_JSON:BOOL=ON
        -DIDYNTREE_USES_SYSTEM_ResolveRoboticsURICpp:BOOL=OFF
        "-DFETCHCONTENT_SOURCE_DIR_RESOLVEROBOTICSURICPP=${RRUC_SOURCE_PATH}"
        "-DFETCHCONTENT_SOURCE_DIR_YCM=${YCM_SOURCE_PATH}"
        -DRRUP_USE_SYSTEM_YCM:BOOL=OFF
        -DRRUC_INSTALL:BOOL=OFF
        -DRRUC_BUILD_TOOL:BOOL=OFF
        -DIDYNTREE_USES_IPOPT:BOOL=OFF
        -DIDYNTREE_USES_OSQPEIGEN:BOOL=OFF
        -DIDYNTREE_USES_MATLAB:BOOL=OFF
        -DIDYNTREE_USES_PYTHON:BOOL=OFF
        -DIDYNTREE_USES_OCTAVE:BOOL=OFF
        -DIDYNTREE_USES_LUA:BOOL=OFF
        -DIDYNTREE_USES_YARP:BOOL=OFF
        -DIDYNTREE_USES_ICUB_MAIN:BOOL=OFF
        -DIDYNTREE_USES_ALGLIB:BOOL=OFF
        -DIDYNTREE_USES_WORHP:BOOL=OFF
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup(
    PACKAGE_NAME iDynTree
    CONFIG_PATH lib/cmake/iDynTree
)
vcpkg_copy_pdbs()

set(TOOL_NAMES_LIST idyntree-model-info idyntree-model-convert)
if ("assimp" IN_LIST FEATURES)
    list(APPEND TOOL_NAMES_LIST idyntree-model-simplify-shapes)
endif()
if ("irrlicht" IN_LIST FEATURES)
    list(APPEND TOOL_NAMES_LIST idyntree-model-view)
endif()
vcpkg_copy_tools(TOOL_NAMES ${TOOL_NAMES_LIST} AUTO_CLEAN)

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

vcpkg_install_copyright(FILE_LIST
    "${SOURCE_PATH}/LICENSE"
    "${RRUC_SOURCE_PATH}/LICENSE"
    "${YCM_SOURCE_PATH}/LICENSE"
)
