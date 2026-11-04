cmake_minimum_required(VERSION 3.12)
project(a2_controller)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)
find_package(unitree_sdk2 REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-x64-1.22.0")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../thirdparty/onnxruntime-linux-aarch64-1.22.0")
endif()

include_directories(
  /usr/include/eigen3
  include
  ${ONNXRUNTIME_DIR}/include
  ${PROJECT_SOURCE_DIR}/../../include/
)

set(A2_DEPLOY_LIBRARIES
  unitree_sdk2 ddsc ddscxx rt pthread
  libboost_program_options.a libyaml-cpp.a fmt
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.22.0
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${A2_DEPLOY_LIBRARIES})

add_executable(a2_ctrl main.cpp)
target_link_libraries(a2_ctrl PRIVATE ${PROJECT_NAME}_lib)

add_executable(real_joint_test real_joint_test.cpp)
target_link_libraries(real_joint_test PRIVATE
  unitree_sdk2 ddsc ddscxx rt pthread
  libboost_program_options.a libyaml-cpp.a
)

option(BUILD_OFFLINE_TESTS "Build policy checks that do not create DDS channels" OFF)
if(BUILD_OFFLINE_TESTS)
  add_executable(a2_policy_probe tests/policy_probe.cpp)
  target_link_libraries(a2_policy_probe PRIVATE ${PROJECT_NAME}_lib)
  add_executable(a2_joystick_test tests/joystick_test.cpp)
  target_link_libraries(a2_joystick_test PRIVATE ${PROJECT_NAME}_lib)
  add_executable(a2_joint_test tests/joint_test.cpp)
  target_link_libraries(a2_joint_test PRIVATE ddsc ddscxx libyaml-cpp.a)
  add_executable(a2_fsm_shutdown_test tests/fsm_shutdown_test.cpp)
  target_link_libraries(a2_fsm_shutdown_test PRIVATE ${PROJECT_NAME}_lib)
  enable_testing()
  add_test(NAME a2_joystick COMMAND a2_joystick_test "${PROJECT_SOURCE_DIR}/../../..")
  add_test(NAME a2_joint_test COMMAND a2_joint_test "${PROJECT_SOURCE_DIR}/../../..")
  add_test(NAME a2_fsm_shutdown COMMAND a2_fsm_shutdown_test "${PROJECT_SOURCE_DIR}/../../..")
  set_tests_properties(a2_fsm_shutdown PROPERTIES TIMEOUT 10)
endif()
