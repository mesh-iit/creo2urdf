# Copyright (C) 2025 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.


add_executable(creo2urdf-model-test creo2urdf-model-test.cpp)
target_link_libraries(creo2urdf-model-test ${iDynTree_LIBRARIES})

macro(add_creo2urdf_model_test modelName)
    if (COMPILE_creo2urdfModels)
        add_test(NAME ${modelName}ConsistencyCheck
             COMMAND creo2urdf-model-test ${CMAKE_BINARY_DIR}/${BUILD_PREFIX}/robots/${modelName}/model.urdf)
    endif()
endmacro()

# Model generated with simmechanics
#add_creo2urdf_model_test()
