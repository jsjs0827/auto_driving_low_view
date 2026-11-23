add_library(perception_sdk SHARED
  "${PERCEPTION_PROJECT_ROOT}/sdk/src/perception_sdk.cpp"
  "${PERCEPTION_PROJECT_ROOT}/sdk/src/perception_full_sdk.cpp"
  "${PERCEPTION_PROJECT_ROOT}/scheduling/inference/src/inference_session.cpp")
target_sources(perception_sdk PRIVATE "${PERCEPTION_PROJECT_ROOT}/scheduling/pipeline/src/full_pipeline.cpp"
  "${PERCEPTION_PROJECT_ROOT}/scheduling/config/src/full_config.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/dag/src/full_graph.cpp" "${PERCEPTION_PROJECT_ROOT}/features/temporal/backend/src/temporal.cpp")
target_include_directories(perception_sdk PRIVATE "${PERCEPTION_PROJECT_ROOT}/scheduling/config/include"
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/original" "${PERCEPTION_PROJECT_ROOT}/features/pointcloud/backend/include"
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend" "${PERCEPTION_PROJECT_ROOT}/scheduling/product/include")
target_link_libraries(perception_sdk PRIVATE r33_stereo_inputs r36_motion_features)
target_link_libraries(perception_sdk PRIVATE perception_algorithm_runtime)
if(R33_WITH_MAPPING)
  target_compile_definitions(perception_sdk PRIVATE R33_WITH_MAPPING=1)
  target_link_libraries(perception_sdk PRIVATE r33_mapping_core)
endif()
if(R33_WITH_RECHARGE)
  target_compile_definitions(perception_sdk PRIVATE R33_WITH_RECHARGE=1)
  target_link_libraries(perception_sdk PRIVATE r33_recharge_core)
  # R36_RECHARGE_LINK_BEGIN
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Consumers link only the public SDK. GNU cross ld also needs to locate
    # its private DT_NEEDED backend in the sibling recharge build directory.
    # Keep this build path out of the installed SDK's runtime search paths.
    target_link_options(perception_sdk INTERFACE
      "$<BUILD_INTERFACE:-Wl,-rpath-link,$<TARGET_FILE_DIR:r35_recharge_backend>>")
  endif()
  # R36_RECHARGE_LINK_END
endif()
target_include_directories(perception_sdk
  PUBLIC $<BUILD_INTERFACE:${PERCEPTION_PROJECT_ROOT}/sdk/include> $<INSTALL_INTERFACE:include>
  PRIVATE "${PERCEPTION_PROJECT_ROOT}/features/inference/preprocess/include" "${PERCEPTION_PROJECT_ROOT}/features/camera/backend/include"
          "${PERCEPTION_PROJECT_ROOT}/features/inference/legacy_support")
target_compile_definitions(perception_sdk PRIVATE PERCEPTION_SDK_BUILD=1)
target_link_libraries(perception_sdk PRIVATE dag_exact_features app_support)
if(WIN32 AND R33_WITH_RECHARGE)
  r33_copy_opencv_runtime(perception_sdk)
  install(FILES ${R33_OPENCV_RUNTIME_FILES} DESTINATION bin COMPONENT SharedSDK)
endif()
set_target_properties(perception_sdk PROPERTIES
  VERSION 1.0.36 SOVERSION 1 CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES)
if(R20_WITH_BPU)
  target_compile_definitions(perception_sdk PRIVATE R20_WITH_BPU=1)
  target_link_libraries(perception_sdk PRIVATE original_runner)
  # The shared library's private runner does not propagate its link options
  # to consumers. They also need the vendor DSOs' transitive link directories.
  foreach(directory IN LISTS PERCEPTION_LINK_DEPENDENCY_DIRS)
    target_link_options(perception_sdk PUBLIC
      "$<BUILD_INTERFACE:-Wl,-rpath-link,${directory}>")
  endforeach()
endif()
if(R29_WITH_ORT)
  target_compile_definitions(perception_sdk PRIVATE R29_WITH_ORT=1)
  target_link_libraries(perception_sdk PRIVATE cpu_onnx_stages)
  if(WIN32)
    add_custom_command(TARGET perception_sdk POST_BUILD COMMAND ${CMAKE_COMMAND} -E
      copy_if_different "${ORT_RUNTIME}" "$<TARGET_FILE_DIR:perception_sdk>")
    install(FILES "${ORT_RUNTIME}" DESTINATION bin COMPONENT SharedSDK)
  else()
    file(GLOB _shared_ort_runtime "${ORT_ROOT}/lib/libonnxruntime.so*")
    install(FILES ${_shared_ort_runtime} DESTINATION lib COMPONENT SharedSDK)
  endif()
  foreach(_notice IN ITEMS LICENSE ThirdPartyNotices.txt VERSION_NUMBER)
    if(EXISTS "${ORT_ROOT}/${_notice}")
      install(FILES "${ORT_ROOT}/${_notice}" DESTINATION licenses/onnxruntime COMPONENT SharedSDK)
    endif()
  endforeach()
endif()
if(MSVC)
  target_compile_options(perception_sdk PRIVATE /utf-8 /EHsc /fp:strict)
else()
  target_compile_options(perception_sdk PRIVATE -fno-fast-math -ffp-contract=off)
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(_math_linked "")
  foreach(_math IN LISTS PCL_LIBRARIES OpenCV_LIBS)
    if(TARGET "${_math}")
      get_target_property(_math_type "${_math}" TYPE)
      if(NOT _math_type STREQUAL "INTERFACE_LIBRARY")
        string(APPEND _math_linked "$<TARGET_FILE:${_math}>\n")
      endif()
    elseif(IS_ABSOLUTE "${_math}" AND EXISTS "${_math}")
      string(APPEND _math_linked "${_math}\n")
    endif()
  endforeach()
  file(GENERATE OUTPUT "${PROJECT_BINARY_DIR}/r33_math_linked.txt" CONTENT "${_math_linked}")
  target_link_options(perception_sdk PRIVATE
    "-Wl,--no-undefined" "-Wl,--exclude-libs,ALL"
    "-Wl,--version-script=${PERCEPTION_PROJECT_ROOT}/sdk/perception_sdk.map")
  set_property(TARGET perception_sdk APPEND PROPERTY LINK_DEPENDS
    "${PERCEPTION_PROJECT_ROOT}/sdk/perception_sdk.map")
  set_target_properties(perception_sdk PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()
add_executable(perception_sdk_demo "${PERCEPTION_PROJECT_ROOT}/sdk/examples/demo.cpp")
target_link_libraries(perception_sdk_demo PRIVATE perception_sdk)
add_executable(perception_full_demo "${PERCEPTION_PROJECT_ROOT}/sdk/examples/full_demo.cpp")
target_compile_definitions(perception_full_demo PRIVATE R33_DEMO_WITH_CAMERA=1)
target_link_libraries(perception_full_demo PRIVATE perception_sdk r33_stereo_inputs)
if(MSVC)
  target_compile_options(perception_full_demo PRIVATE /utf-8 /EHsc)
  target_link_options(perception_full_demo PRIVATE "/MANIFEST:EMBED"
    "/MANIFESTINPUT:${PROJECT_SOURCE_DIR}/cmake/windows_utf8.manifest")
endif()
if(UNIX)
  set_target_properties(perception_full_demo PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
endif()
if(MSVC)
  target_compile_options(perception_sdk_demo PRIVATE /utf-8 /EHsc)
  target_link_options(perception_sdk_demo PRIVATE "/MANIFEST:EMBED"
    "/MANIFESTINPUT:${PROJECT_SOURCE_DIR}/cmake/windows_utf8.manifest")
endif()
if(UNIX)
  set_target_properties(perception_sdk_demo PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
endif()
install(TARGETS perception_sdk perception_sdk_demo perception_full_demo
  LIBRARY DESTINATION lib COMPONENT SharedSDK
  RUNTIME DESTINATION bin COMPONENT SharedSDK
  ARCHIVE DESTINATION lib COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/sdk/include/perception_sdk.h" DESTINATION include COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/sdk/include/perception_slip_sdk.h" DESTINATION include COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/sdk/include/perception_sensor_input.h" DESTINATION include COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/sdk/include/perception_raw_input.h" DESTINATION include COMPONENT SharedSDK)
install(DIRECTORY "${PERCEPTION_PROJECT_ROOT}/core/runtime/include/" DESTINATION include COMPONENT SharedSDK)
install(DIRECTORY "${PERCEPTION_PROJECT_ROOT}/scheduling/dag/include/perception/" DESTINATION include/perception COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/sdk/include/perception_full_sdk.h" "${PERCEPTION_PROJECT_ROOT}/sdk/include/perception_temporal_sdk.h" "${PERCEPTION_PROJECT_ROOT}/features/temporal/backend/include/perception_temporal.hpp" DESTINATION include COMPONENT SharedSDK)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${PERCEPTION_PROJECT_ROOT}/sdk/cmake/perception_full_shared_sdkConfig.cmake.in"
  "${PROJECT_BINARY_DIR}/perception_full_shared_sdkConfig.cmake"
  INSTALL_DESTINATION lib/cmake/perception_full_shared_sdk)
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/perception_full_shared_sdkConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}" COMPATIBILITY SameMajorVersion)
install(FILES
  "${PROJECT_BINARY_DIR}/perception_full_shared_sdkConfig.cmake"
  "${PROJECT_BINARY_DIR}/perception_full_shared_sdkConfigVersion.cmake"
  DESTINATION lib/cmake/perception_full_shared_sdk COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/sdk/package.xml"
  DESTINATION share/perception_full_shared_sdk COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/sdk/cmake/perception_full_shared_sdk.resource"
  DESTINATION share/ament_index/resource_index/packages
  RENAME perception_full_shared_sdk COMPONENT SharedSDK)
install(DIRECTORY "${PERCEPTION_SDK_RUNTIME_TEMPLATES_DIR}/" DESTINATION config COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/sdk/examples/demo.cpp" "${PERCEPTION_PROJECT_ROOT}/sdk/examples/CMakeLists.txt" DESTINATION examples COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/sdk/examples/full_demo.cpp" "${PERCEPTION_PROJECT_ROOT}/sdk/examples/full/CMakeLists.txt"
  DESTINATION examples/full COMPONENT SharedSDK)
install(DIRECTORY "${PERCEPTION_SDK_RUNTIME_PROFILES_DIR}/" DESTINATION profiles COMPONENT SharedSDK)
install(DIRECTORY "${PERCEPTION_PROJECT_ROOT}/models/" DESTINATION models COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/README_SHARED_zh.md" DESTINATION . COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/docs/r12_v32_shared_zh.md"
              "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/docs/r12_v32_shared_zh.html" DESTINATION docs COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/docs/r12_v33_full_zh.md"
  "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/docs/r12_v33_full_zh.html"
  "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/FUNCTIONAL_SCOPE_R33.json"
  DESTINATION docs COMPONENT SharedSDK)
if(BUILD_TESTING)
  add_executable(slip_sdk_contract_test "${PERCEPTION_PROJECT_ROOT}/validation/slip_sdk_contract_test.cpp")
  target_link_libraries(slip_sdk_contract_test PRIVATE perception_sdk)
  add_test(NAME v8_slip_full_SDK_ownership_transaction_contract COMMAND slip_sdk_contract_test
    "${PERCEPTION_PROJECT_ROOT}/validation/slip_sdk_contract.json")
  add_executable(frame_resources_tests "${PERCEPTION_PROJECT_ROOT}/validation/sdk/tests/frame_resources_tests.cpp")
  target_link_libraries(frame_resources_tests PRIVATE dag_framework Threads::Threads)
  add_test(NAME per_frame_RGB_cache_once_and_retry COMMAND frame_resources_tests)
  add_executable(input_bundle_test "${PERCEPTION_PROJECT_ROOT}/validation/input_bundle_test.cpp")
  target_include_directories(input_bundle_test PRIVATE "${PERCEPTION_PROJECT_ROOT}/sdk/include")
  target_link_libraries(input_bundle_test PRIVATE dag_framework)
  add_test(NAME v4_input_bundle_contract COMMAND input_bundle_test)
  add_executable(input_sensor_slots_test
    "${PERCEPTION_PROJECT_ROOT}/validation/input_sensor_slots_test.cpp"
    "${PERCEPTION_PROJECT_ROOT}/validation/input_sensor_slots_header_c.c")
  target_include_directories(input_sensor_slots_test PRIVATE "${PERCEPTION_PROJECT_ROOT}/sdk/include")
  target_link_libraries(input_sensor_slots_test PRIVATE dag_framework)
  add_test(NAME v6_optional_sensor_input_slots COMMAND input_sensor_slots_test)
  add_executable(inference_task_tests "${PERCEPTION_PROJECT_ROOT}/validation/sdk/tests/inference_task_tests.cpp")
  target_link_libraries(inference_task_tests PRIVATE dag_framework)
  add_test(NAME v4_shared_staged_inference_contract COMMAND inference_task_tests)
  if(MSVC)
    target_compile_options(frame_resources_tests PRIVATE /utf-8 /EHsc)
    target_compile_options(input_bundle_test PRIVATE /utf-8 /EHsc)
    target_compile_options(input_sensor_slots_test PRIVATE /utf-8 /EHsc)
    target_compile_options(inference_task_tests PRIVATE /utf-8 /EHsc)
  endif()
  add_executable(v3_temporal_json_contract "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/temporal_json_contract.cpp")
  target_link_libraries(v3_temporal_json_contract PRIVATE perception_sdk)
  if(MSVC)
    target_compile_options(v3_temporal_json_contract PRIVATE /utf-8 /EHsc)
  endif()
  add_test(NAME v3_temporal_analysis_JSON_without_extension COMMAND v3_temporal_json_contract
    "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/temporal_no_extension.json")
  add_executable(v3_disparity_only_contract "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/disparity_only_contract.cpp")
  target_link_libraries(v3_disparity_only_contract PRIVATE perception_sdk)
  if(MSVC)
    target_compile_options(v3_disparity_only_contract PRIVATE /utf-8 /EHsc)
  endif()
  add_test(NAME v3_independent_disparity_C_ABI_contract COMMAND v3_disparity_only_contract
    "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/disparity_only.json")
  add_executable(v2_sdk_forwarding_contract
    "${PERCEPTION_PROJECT_ROOT}/validation/sdk_forwarding_contract.cpp"
    "${PERCEPTION_PROJECT_ROOT}/sdk/src/perception_sdk.cpp"
    "${PERCEPTION_PROJECT_ROOT}/sdk/src/perception_full_sdk.cpp")
  target_include_directories(v2_sdk_forwarding_contract PRIVATE
    "${PERCEPTION_PROJECT_ROOT}/sdk/include" "${PERCEPTION_PROJECT_ROOT}/sdk/internal")
  target_compile_definitions(v2_sdk_forwarding_contract PRIVATE PERCEPTION_SDK_BUILD=1)
  add_test(NAME v2_sdk_forwarding_C_ABI_contract COMMAND v2_sdk_forwarding_contract)
  add_subdirectory("${PERCEPTION_PROJECT_ROOT}/validation/sdk/full/tests" "${PROJECT_BINARY_DIR}/full-tests")
  add_executable(perception_full_sdk_contract "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/full_sdk_contract.cpp" "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/full_header_c_contract.c")
  target_link_libraries(perception_full_sdk_contract PRIVATE perception_sdk)
  if(MSVC)
    target_compile_options(perception_full_sdk_contract PRIVATE /utf-8 /EHsc)
  endif()
  enable_language(C)
  add_executable(perception_sdk_contract_tests
    "${PERCEPTION_PROJECT_ROOT}/sdk/src/perception_sdk.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/inference/src/inference_session.cpp" "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/sdk_contract_tests.cpp" "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/header_c_contract.c")
  target_include_directories(perception_sdk_contract_tests PRIVATE "${PERCEPTION_PROJECT_ROOT}/sdk/include" "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests"
    "${PERCEPTION_PROJECT_ROOT}/features/inference/preprocess/include" "${PERCEPTION_PROJECT_ROOT}/features/camera/backend/include")
  target_compile_definitions(perception_sdk_contract_tests PRIVATE
    PERCEPTION_SDK_BUILD=1 PSDK_INTERNAL_TESTS=1)
  target_link_libraries(perception_sdk_contract_tests PRIVATE dag_exact_features app_support)
  if(MSVC)
    target_compile_options(perception_sdk_contract_tests PRIVATE /utf-8
      $<$<COMPILE_LANGUAGE:CXX>:/EHsc> $<$<COMPILE_LANGUAGE:CXX>:/fp:strict>)
  else()
    target_compile_options(perception_sdk_contract_tests PRIVATE -fno-fast-math -ffp-contract=off)
  endif()
  add_test(NAME shared_C_ABI_ownership_errors_png_NOT_hardware
    COMMAND perception_sdk_contract_tests "${PERCEPTION_PROJECT_ROOT}/scheduling/config/profiles")
endif()
if(BUILD_TESTING AND R33_WITH_MAPPING AND R33_WITH_RECHARGE)
  install(TARGETS r33_mapping_tests r33_recharge_contract_tests r33_stereo_input_tests
    r33_full_graph_contract
    RUNTIME DESTINATION tests COMPONENT SharedSDK)
  if(UNIX)
    set_target_properties(r33_mapping_tests r33_recharge_contract_tests r33_stereo_input_tests
      r33_full_graph_contract PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  endif()
endif()

if(BUILD_TESTING AND R33_WITH_RECHARGE)
  add_executable(r35_recharge_cabi "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/r35_recharge_cabi.cpp")
  target_link_libraries(r35_recharge_cabi PRIVATE perception_sdk)
  target_include_directories(r35_recharge_cabi PRIVATE "${R33_RECHARGE_RAPIDJSON_INCLUDE_DIR}")
  if(MSVC)
    target_compile_options(r35_recharge_cabi PRIVATE /utf-8 /EHsc)
  endif()
  r33_copy_opencv_runtime(r35_recharge_cabi)
  add_test(NAME r35_full_C_ABI_recharge_DAG_ownership_identity COMMAND r35_recharge_cabi
    "${PERCEPTION_PROJECT_ROOT}/validation/sdk/recharge/tests/full_recharge.json" "${PROJECT_BINARY_DIR}/recharge/fixtures/dual.ppm")
  set_tests_properties(r35_full_C_ABI_recharge_DAG_ownership_identity PROPERTIES FIXTURES_REQUIRED aruco_frames)
endif()

add_executable(perception_temporal_demo "${PERCEPTION_PROJECT_ROOT}/sdk/examples/temporal_demo.cpp")
target_link_libraries(perception_temporal_demo PRIVATE perception_sdk)
if(MSVC)
  target_compile_options(perception_temporal_demo PRIVATE /utf-8 /EHsc)
else()
  set_target_properties(perception_temporal_demo PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
endif()
install(TARGETS perception_temporal_demo RUNTIME DESTINATION bin COMPONENT SharedSDK)
install(FILES "${PERCEPTION_PROJECT_ROOT}/sdk/examples/temporal_demo.cpp" DESTINATION examples COMPONENT SharedSDK)
option(R36_BUILD_TEMPORAL_TESTS "Build owned temporal history contract tests" ON)
if(BUILD_TESTING OR R36_BUILD_TEMPORAL_TESTS)
  enable_language(C)
  add_executable(r36_temporal_unit "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/temporal_unit.cpp"
    "${PERCEPTION_PROJECT_ROOT}/features/temporal/backend/src/temporal.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/config/src/full_config.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/dag/src/full_graph.cpp")
  target_include_directories(r36_temporal_unit PRIVATE "${PERCEPTION_PROJECT_ROOT}/sdk/include" "${PERCEPTION_PROJECT_ROOT}/scheduling/config/include")
  target_link_libraries(r36_temporal_unit PRIVATE dag_framework runtime_status r36_motion_features)
  add_executable(r36_temporal_contract "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/temporal_contract.cpp" "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/temporal_header_c.c")
  target_link_libraries(r36_temporal_contract PRIVATE perception_sdk)
  if(MSVC)
    target_compile_options(r36_temporal_unit PRIVATE /utf-8 /EHsc)
    target_compile_options(r36_temporal_contract PRIVATE /utf-8 $<$<COMPILE_LANGUAGE:CXX>:/EHsc>)
  endif()
  if(UNIX)
    set_target_properties(r36_temporal_unit r36_temporal_contract PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  endif()
  if(WIN32)
    # Windows loads the SDK/OpenCV/ORT DLLs beside the executable.
    install(TARGETS r36_temporal_unit r36_temporal_contract RUNTIME DESTINATION bin COMPONENT SharedSDK)
  else()
    install(TARGETS r36_temporal_unit r36_temporal_contract RUNTIME DESTINATION tests COMPONENT SharedSDK)
  endif()
  install(FILES "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/temporal_only.json" DESTINATION config RENAME temporal_test.json COMPONENT SharedSDK)
  add_test(NAME r36_temporal_sampling_memory_DAG_contract COMMAND r36_temporal_unit)
  add_test(NAME r36_temporal_public_C_ABI_ownership_contract COMMAND r36_temporal_contract "${PERCEPTION_PROJECT_ROOT}/validation/sdk/shared/tests/temporal_only.json")
  set_tests_properties(r36_temporal_sampling_memory_DAG_contract r36_temporal_public_C_ABI_ownership_contract PROPERTIES TIMEOUT 30)
endif()

# Real public C ABI smoke for the aligned geometry path, with models disabled.
if(BUILD_TESTING AND R33_WITH_MAPPING)
  add_executable(r33_v0_mapping_cabi "${PERCEPTION_PROJECT_ROOT}/validation/sdk/mapping/tests/v0_mapping_cabi.cpp")
  target_link_libraries(r33_v0_mapping_cabi PRIVATE perception_sdk ${CMAKE_DL_LIBS})
  target_include_directories(r33_v0_mapping_cabi PRIVATE "${PERCEPTION_PROJECT_ROOT}/scheduling/config/include")
  target_compile_features(r33_v0_mapping_cabi PRIVATE cxx_std_17)
  add_test(NAME r33_v0_maps_only_C_ABI COMMAND r33_v0_mapping_cabi)
endif()
