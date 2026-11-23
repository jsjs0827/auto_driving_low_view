cmake_minimum_required(VERSION 3.21)
project(perception_dual_entry_host_contracts LANGUAGES CXX)
include(CTest)
find_package(Threads REQUIRED)

set(DUAL_ENTRY_PROJECT_ROOT "" CACHE PATH "Read-only canonical source tree for public SDK headers and JSON parser")
get_filename_component(_default_overlay "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(DUAL_ENTRY_OVERLAY_ROOT "${_default_overlay}" CACHE PATH "Reviewed work patch-tree")
set(DUAL_ENTRY_SDK_LIBRARY "" CACHE FILEPATH "Optional existing real perception SDK shared library; never a mock")
if(NOT EXISTS "${DUAL_ENTRY_PROJECT_ROOT}/sdk/include/perception_full_sdk.h")
  message(FATAL_ERROR "Set DUAL_ENTRY_PROJECT_ROOT to the canonical source tree")
endif()
set(_bridge "${DUAL_ENTRY_OVERLAY_ROOT}/features/sdk_bridge")
set(_board "${DUAL_ENTRY_OVERLAY_ROOT}/features/v0_host")
if(NOT EXISTS "${_bridge}/include/perception_entry/session.hpp")
  message(FATAL_ERROR "Shared session overlay is missing")
endif()

add_library(dual_entry_contract_bridge STATIC "${_bridge}/src/sdk_caller.cpp")
target_compile_features(dual_entry_contract_bridge PUBLIC cxx_std_17)
target_include_directories(dual_entry_contract_bridge PUBLIC
  "${_bridge}/include" "${DUAL_ENTRY_PROJECT_ROOT}/sdk/include")
target_link_libraries(dual_entry_contract_bridge PUBLIC Threads::Threads)
add_library(dual_entry_contract_board STATIC "${_board}/src/input.cpp" "${_board}/src/controller.cpp")
target_compile_features(dual_entry_contract_board PUBLIC cxx_std_17)
target_include_directories(dual_entry_contract_board PUBLIC "${_board}/include")
target_link_libraries(dual_entry_contract_board PUBLIC dual_entry_contract_bridge)

add_executable(dual_entry_session_mock_contract session_mock_contract.cpp)
target_link_libraries(dual_entry_session_mock_contract PRIVATE dual_entry_contract_board)
add_test(NAME dual_entry_session_mock_C_ABI_NOT_hardware COMMAND dual_entry_session_mock_contract)
set_tests_properties(dual_entry_session_mock_C_ABI_NOT_hardware PROPERTIES TIMEOUT 20 LABELS "synthetic;host;mock")

# input.cpp alone is intentionally independent of the SDK implementation.
add_executable(dual_entry_board_input_contract board_input_contract.cpp "${_board}/src/input.cpp")
target_compile_features(dual_entry_board_input_contract PRIVATE cxx_std_17)
target_include_directories(dual_entry_board_input_contract PRIVATE
  "${_board}/include" "${DUAL_ENTRY_PROJECT_ROOT}/sdk/include")
add_test(NAME dual_entry_board_input_synthetic_descriptor_contract COMMAND dual_entry_board_input_contract)
set_tests_properties(dual_entry_board_input_synthetic_descriptor_contract PROPERTIES TIMEOUT 20 LABELS "synthetic;host;input")

if(DUAL_ENTRY_SDK_LIBRARY)
  if(NOT EXISTS "${DUAL_ENTRY_SDK_LIBRARY}")
    message(FATAL_ERROR "DUAL_ENTRY_SDK_LIBRARY does not exist")
  endif()
  add_library(dual_entry_actual_sdk SHARED IMPORTED)
  set_target_properties(dual_entry_actual_sdk PROPERTIES IMPORTED_LOCATION "${DUAL_ENTRY_SDK_LIBRARY}")
  add_executable(dual_entry_session_real_sdk_contract session_real_sdk_contract.cpp)
  target_include_directories(dual_entry_session_real_sdk_contract PRIVATE "${DUAL_ENTRY_PROJECT_ROOT}/scheduling/config/include")
  target_link_libraries(dual_entry_session_real_sdk_contract PRIVATE dual_entry_contract_board dual_entry_actual_sdk)
  add_test(NAME dual_entry_actual_SDK_temporal_DAG_entry_parity_NOT_camera_NOT_model
    COMMAND dual_entry_session_real_sdk_contract "${CMAKE_CURRENT_LIST_DIR}/synthetic_temporal.json")
  set_tests_properties(dual_entry_actual_SDK_temporal_DAG_entry_parity_NOT_camera_NOT_model
    PROPERTIES TIMEOUT 30 LABELS "synthetic;host;real_sdk;temporal")
endif()

foreach(_target dual_entry_session_mock_contract dual_entry_board_input_contract dual_entry_contract_bridge dual_entry_contract_board)
  if(MSVC)
    target_compile_options(${_target} PRIVATE /utf-8 /EHsc)
  else()
    target_compile_options(${_target} PRIVATE -Wall -Wextra -Wpedantic -fno-fast-math -ffp-contract=off)
  endif()
endforeach()
# There are no install, packaging, ADB, Docker, camera or producer-start actions.
