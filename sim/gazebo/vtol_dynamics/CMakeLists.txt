cmake_minimum_required(VERSION 3.16)
project(vtol_dynamics CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(nlohmann_json 3 REQUIRED)

# Force model, no Gazebo dependency.
add_library(vtol_model STATIC src/model.cpp)
target_include_directories(vtol_model PUBLIC include)
target_link_libraries(vtol_model PUBLIC nlohmann_json::nlohmann_json)
target_compile_options(vtol_model PRIVATE -Wall -Wextra)

add_executable(test_model test/test_model.cpp)
target_link_libraries(test_model PRIVATE vtol_model)
enable_testing()
add_test(NAME model_matches_python
         COMMAND test_model ${CMAKE_CURRENT_SOURCE_DIR}/../../../models/aero/p1
                            ${CMAKE_CURRENT_SOURCE_DIR}/test/vectors.json)

# Gazebo system plugin (Harmonic = gz-sim8).
find_package(gz-sim8 QUIET)
if(gz-sim8_FOUND)
  find_package(gz-plugin2 REQUIRED COMPONENTS register)
  find_package(gz-transport13 REQUIRED)
  add_library(VtolDynamics SHARED src/VtolDynamics.cpp)
  target_link_libraries(VtolDynamics PRIVATE vtol_model gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13)
  install(TARGETS VtolDynamics LIBRARY DESTINATION lib)
else()
  message(WARNING "gz-sim8 not found: building the force model and its test only")
endif()
