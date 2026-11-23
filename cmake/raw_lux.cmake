# Preserve the existing RAW feature kernel's arithmetic independently of the
# surrounding DAG's strict-FP flags. This feature has no shared-memory dependency.
set(_raw_lux_backend "${PERCEPTION_PROJECT_ROOT}/features/raw_lux/backend/src/raw_lux_runner.cpp")
if(MSVC)
  set_source_files_properties("${_raw_lux_backend}" PROPERTIES COMPILE_OPTIONS "/O2;/fp:precise")
else()
  set_source_files_properties("${_raw_lux_backend}" PROPERTIES
    COMPILE_OPTIONS "-O3;-fno-fast-math;-ffp-contract=fast")
endif()
if(BUILD_TESTING)
  add_executable(raw_input_contract_test
    "${PERCEPTION_PROJECT_ROOT}/validation/raw_input_test.cpp"
    "${PERCEPTION_PROJECT_ROOT}/validation/raw_input_header_c.c")
  target_include_directories(raw_input_contract_test PRIVATE
    "${PERCEPTION_PROJECT_ROOT}/sdk/include"
    "${PERCEPTION_PROJECT_ROOT}/scheduling/runtime/include"
    "${PERCEPTION_PROJECT_ROOT}/core/runtime/include")
  target_link_libraries(raw_input_contract_test PRIVATE dag_framework)
  add_test(NAME raw_stereo_optional_sensor_C_ABI_contract COMMAND raw_input_contract_test)

  add_executable(raw_lux_plugin_test
    "${PERCEPTION_PROJECT_ROOT}/validation/raw_lux_plugin_test.cpp")
  target_include_directories(raw_lux_plugin_test PRIVATE
    "${PERCEPTION_PROJECT_ROOT}" "${PERCEPTION_PROJECT_ROOT}/sdk/include"
    "${PERCEPTION_PROJECT_ROOT}/features/raw_lux/backend/include"
    "${PERCEPTION_PROJECT_ROOT}/core/runtime/include"
    "${PERCEPTION_PROJECT_ROOT}/scheduling/config/include"
    "${PERCEPTION_PROJECT_ROOT}/scheduling/dag/include"
    "${PERCEPTION_PROJECT_ROOT}/scheduling/algorithms/include")
  target_link_libraries(raw_lux_plugin_test PRIVATE perception_algorithm_runtime dag_framework)
  add_executable(raw_lux_skew_test "${PERCEPTION_PROJECT_ROOT}/validation/raw_lux_skew_test.cpp")
  target_include_directories(raw_lux_skew_test PRIVATE "${PERCEPTION_PROJECT_ROOT}"
    "${PERCEPTION_PROJECT_ROOT}/sdk/include" "${PERCEPTION_PROJECT_ROOT}/core/runtime/include"
    "${PERCEPTION_PROJECT_ROOT}/scheduling/config/include" "${PERCEPTION_PROJECT_ROOT}/scheduling/dag/include"
    "${PERCEPTION_PROJECT_ROOT}/scheduling/algorithms/include")
  target_link_libraries(raw_lux_skew_test PRIVATE perception_algorithm_runtime dag_framework)
  set(RAW_LUX_TEST_FIXTURES "/home/st/ini_code/raw_lux1_cpp/examples" CACHE PATH "Optional real RAW examples for native plugin regression")
  if(EXISTS "${RAW_LUX_TEST_FIXTURES}/left.raw" AND EXISTS "${RAW_LUX_TEST_FIXTURES}/right.raw")
    add_test(NAME raw_lux_real_features_and_lane_isolation
      COMMAND raw_lux_plugin_test "${RAW_LUX_TEST_FIXTURES}")
    add_test(NAME raw_lux_explicit_cross_stream_policy
      COMMAND raw_lux_skew_test "${RAW_LUX_TEST_FIXTURES}")
  endif()
endif()

if(BUILD_TESTING AND UNIX AND NOT CMAKE_CROSSCOMPILING)
  add_test(NAME local_payload_release_guard COMMAND bash
    "${PERCEPTION_PROJECT_ROOT}/validation/local_payload_guard_test.sh" "${PERCEPTION_PROJECT_ROOT}")
endif()

# C ABI end-to-end tests exercise engine/result ownership independently of the
# pure feature factory test. SDK is defined later in sdk/CMakeLists.txt.
function(perception_add_raw_lux_sdk_test)
  if(BUILD_TESTING AND TARGET perception_sdk)
    add_executable(raw_lux_sdk_test "${PERCEPTION_PROJECT_ROOT}/validation/raw_lux_sdk_test.cpp"
      "${PERCEPTION_PROJECT_ROOT}/features/raw_lux/backend/src/raw_lux_runner.cpp")
    target_include_directories(raw_lux_sdk_test PRIVATE "${PERCEPTION_PROJECT_ROOT}"
      "${PERCEPTION_PROJECT_ROOT}/sdk/include" "${PERCEPTION_PROJECT_ROOT}/scheduling/config/include")
    target_link_libraries(raw_lux_sdk_test PRIVATE perception_sdk Threads::Threads)
    if(EXISTS "${RAW_LUX_TEST_FIXTURES}/left.raw" AND EXISTS "${RAW_LUX_TEST_FIXTURES}/right.raw")
      add_test(NAME raw_lux_full_C_ABI_results_and_legacy_frames
        COMMAND raw_lux_sdk_test "${RAW_LUX_TEST_FIXTURES}")
    endif()
  endif()
endfunction()
