cmake_minimum_required(VERSION 3.13)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(creo2urdf_identity_tests LANGUAGES CXX)
  enable_testing()
endif()

add_executable(component_identity_test ComponentIdentityTest.cpp)
target_compile_features(component_identity_test PRIVATE cxx_std_14)
target_include_directories(component_identity_test PRIVATE ../src/creo2urdf/include)
add_test(NAME component_identity COMMAND component_identity_test)

# Available in the full project build; standalone identity tests need no SDKs.
if(TARGET iDynTree::idyntree-model)
  add_executable(component_model_test ComponentModelTest.cpp)
  target_compile_features(component_model_test PRIVATE cxx_std_14)
  target_include_directories(component_model_test PRIVATE ../src/creo2urdf/include)
  target_link_libraries(component_model_test PRIVATE iDynTree::idyntree-model)
  add_test(NAME component_model COMMAND component_model_test)
endif()
