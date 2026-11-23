# Included from the mapping backend after r33_mapping_core is defined.
add_executable(r33_v0_alignment_tests
  "${PERCEPTION_PROJECT_ROOT}/validation/sdk/mapping/tests/v0_alignment_tests.cpp")
target_link_libraries(r33_v0_alignment_tests PRIVATE r33_mapping_core)
target_include_directories(r33_v0_alignment_tests PRIVATE
  "${PERCEPTION_PROJECT_ROOT}/validation/sdk/mapping/tests"
  "${PERCEPTION_PROJECT_ROOT}/features/fusion/backend/include"
  "${PERCEPTION_PROJECT_ROOT}/features/fusion/vendor/include"
  "${PERCEPTION_PROJECT_ROOT}/features/pointcloud/vendor/include"
  "${PERCEPTION_PROJECT_ROOT}/features/pointcloud/backend/src"
  "${PERCEPTION_PROJECT_ROOT}/features/inference/backend/include"
  ${R33_RAPIDJSON_INCLUDE_DIR})
target_include_directories(r33_v0_alignment_tests SYSTEM PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(r33_v0_alignment_tests PRIVATE PCL_NO_PRECOMPILE)
target_compile_features(r33_v0_alignment_tests PRIVATE cxx_std_17)
if(MSVC)
  target_compile_options(r33_v0_alignment_tests PRIVATE /fp:precise /bigobj)
else()
  target_compile_options(r33_v0_alignment_tests PRIVATE -fno-fast-math -ffp-contract=off)
endif()
add_test(NAME r33_v0_1104_alignment_oracle COMMAND r33_v0_alignment_tests)
