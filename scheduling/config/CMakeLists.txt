cmake_minimum_required(VERSION 3.20)
get_filename_component(PERCEPTION_NATIVE_SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
include("${PERCEPTION_NATIVE_SOURCE_ROOT}/cmake/algorithms.cmake")
add_library(perception_native_config STATIC
  src/native_json.cpp src/native_features.cpp src/native_pipeline.cpp src/native_dag.cpp)
target_compile_features(perception_native_config PUBLIC cxx_std_17)
target_link_libraries(perception_native_config PUBLIC r36_motion_features)
target_link_libraries(perception_native_config PUBLIC perception_algorithm_catalog)
target_include_directories(perception_native_config PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${PERCEPTION_NATIVE_SOURCE_ROOT}/scheduling/dag/include"
  "${PERCEPTION_NATIVE_SOURCE_ROOT}/core/include")
target_compile_definitions(perception_native_config PRIVATE
  PERCEPTION_NATIVE_PROJECT_ROOT="${PERCEPTION_NATIVE_SOURCE_ROOT}")
add_executable(perception_config "${PERCEPTION_NATIVE_SOURCE_ROOT}/scheduling/launch/src/perception_config.cpp")
target_link_libraries(perception_config PRIVATE perception_native_config)
if(MSVC)
  target_compile_options(perception_native_config PRIVATE /utf-8 /EHsc)
  target_compile_options(perception_config PRIVATE /utf-8 /EHsc)
else()
  target_compile_options(perception_native_config PRIVATE -Wall -Wextra -Wpedantic)
endif()
install(TARGETS perception_config RUNTIME DESTINATION bin)
if(BUILD_TESTING)
  add_executable(algorithm_config_contract_test
    "${PERCEPTION_NATIVE_SOURCE_ROOT}/validation/algorithm_config_test.cpp")
  target_link_libraries(algorithm_config_contract_test PRIVATE perception_native_config)
  set(_algorithm_fixture "${CMAKE_BINARY_DIR}/algorithm-contract-fixture")
  add_test(NAME algorithm_plugin_configuration COMMAND algorithm_config_contract_test
    "${PERCEPTION_NATIVE_SOURCE_ROOT}" "${_algorithm_fixture}")
  set_tests_properties(algorithm_plugin_configuration PROPERTIES FIXTURES_SETUP algorithm_fixture TIMEOUT 60)
  if(TARGET perception_sdk AND R29_WITH_ORT)
    add_executable(algorithm_runtime_contract_test
      "${PERCEPTION_NATIVE_SOURCE_ROOT}/validation/algorithm_runtime_test.cpp")
    target_include_directories(algorithm_runtime_contract_test PRIVATE
      "${PERCEPTION_NATIVE_SOURCE_ROOT}/scheduling/config/include")
    target_link_libraries(algorithm_runtime_contract_test PRIVATE perception_sdk)
    if(WIN32)
      find_file(_algorithm_ort_runtime onnxruntime.dll
        PATHS "${ORT_ROOT}/lib" "${ORT_ROOT}/bin" NO_DEFAULT_PATH REQUIRED)
      add_custom_command(TARGET algorithm_runtime_contract_test POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          $<TARGET_RUNTIME_DLLS:algorithm_runtime_contract_test>
          $<TARGET_FILE_DIR:algorithm_runtime_contract_test>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
          "${_algorithm_ort_runtime}" $<TARGET_FILE_DIR:algorithm_runtime_contract_test>
        COMMAND_EXPAND_LISTS)
    endif()
    add_test(NAME algorithm_plugin_real_onnx_runtime COMMAND algorithm_runtime_contract_test
      "${_algorithm_fixture}/effective_full.json")
    set_tests_properties(algorithm_plugin_real_onnx_runtime PROPERTIES FIXTURES_REQUIRED algorithm_fixture TIMEOUT 60)
  endif()
  if(TARGET dag_framework)
    set(_dag_contract_runtime dag_framework)
  else()
    find_package(Threads REQUIRED)
    add_library(product_dag_contract_runtime STATIC
      "${PERCEPTION_NATIVE_SOURCE_ROOT}/scheduling/dag/src/task_graph.cpp")
    target_include_directories(product_dag_contract_runtime PUBLIC
      "${PERCEPTION_NATIVE_SOURCE_ROOT}/scheduling/dag/include")
    target_link_libraries(product_dag_contract_runtime PUBLIC Threads::Threads)
    set(_dag_contract_runtime product_dag_contract_runtime)
  endif()
  add_executable(native_dag_contract_test
    "${PERCEPTION_NATIVE_SOURCE_ROOT}/validation/native_dag_test.cpp")
  target_link_libraries(native_dag_contract_test PRIVATE
    perception_native_config ${_dag_contract_runtime})
  add_test(NAME native_structured_dag_contracts COMMAND native_dag_contract_test
    "${PERCEPTION_NATIVE_SOURCE_ROOT}")
  set_tests_properties(native_structured_dag_contracts PROPERTIES TIMEOUT 30)
  add_executable(motion_config_contract_test "${PERCEPTION_NATIVE_SOURCE_ROOT}/validation/motion_config_test.cpp")
  target_link_libraries(motion_config_contract_test PRIVATE perception_native_config)
  add_test(NAME v8_motion_configuration_contracts COMMAND motion_config_contract_test "${PERCEPTION_NATIVE_SOURCE_ROOT}")
  enable_testing()
  add_executable(native_config_contract_test "${PERCEPTION_NATIVE_SOURCE_ROOT}/validation/native_config_test.cpp")
  target_link_libraries(native_config_contract_test PRIVATE perception_native_config)
  if(MSVC)
    target_compile_options(native_config_contract_test PRIVATE /utf-8 /EHsc)
  endif()
  add_test(NAME native_config_contracts COMMAND native_config_contract_test "${PERCEPTION_NATIVE_SOURCE_ROOT}")
  add_executable(offline_image_contract_test
    "${PERCEPTION_NATIVE_SOURCE_ROOT}/validation/offline_image_test.cpp"
    "${PERCEPTION_NATIVE_SOURCE_ROOT}/features/offline/src/offline_image.cpp")
  target_include_directories(offline_image_contract_test PRIVATE
    "${PERCEPTION_NATIVE_SOURCE_ROOT}/features/offline/include")
  add_test(NAME offline_image_q8_contracts COMMAND offline_image_contract_test)
  add_executable(camera_benchmark_config_test "${PERCEPTION_NATIVE_SOURCE_ROOT}/validation/camera_benchmark_config_test.cpp")
  target_link_libraries(camera_benchmark_config_test PRIVATE perception_native_config)
  add_test(NAME camera_benchmark_config_contracts COMMAND camera_benchmark_config_test "${PERCEPTION_NATIVE_SOURCE_ROOT}")
  if(UNIX AND NOT CMAKE_CROSSCOMPILING)
    add_test(NAME native_bash_entrypoints COMMAND bash
      "${PERCEPTION_NATIVE_SOURCE_ROOT}/validation/native_entrypoints_test.sh" "${PERCEPTION_NATIVE_SOURCE_ROOT}")
    add_test(NAME camera_benchmark_bash_entrypoints COMMAND bash
      "${PERCEPTION_NATIVE_SOURCE_ROOT}/validation/camera_benchmark_entrypoints_test.sh" "${PERCEPTION_NATIVE_SOURCE_ROOT}")
  endif()
endif()

if(BUILD_TESTING AND CMAKE_SYSTEM_NAME STREQUAL "Linux" AND EXISTS "${PERCEPTION_NATIVE_SOURCE_ROOT}/local_only/camera_shm/tests/config_test.cpp")
  add_executable(camera_shm_config_test "${PERCEPTION_NATIVE_SOURCE_ROOT}/local_only/camera_shm/tests/config_test.cpp")
  target_link_libraries(camera_shm_config_test PRIVATE perception_native_config)
  add_test(NAME camera_shm_native_configuration_contract
    COMMAND camera_shm_config_test "${PERCEPTION_NATIVE_SOURCE_ROOT}")
endif()
