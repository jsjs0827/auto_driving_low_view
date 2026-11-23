get_filename_component(R36_MOTION_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
file(GLOB_RECURSE _motion_sources CONFIGURE_DEPENDS
  "${R36_MOTION_ROOT}/features/visual_motion/src/*.cpp"
  "${R36_MOTION_ROOT}/features/motion_reference/src/*.cpp"
  "${R36_MOTION_ROOT}/features/slip_detection/src/*.cpp")
add_library(r36_motion_features STATIC ${_motion_sources})
set_target_properties(r36_motion_features PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(r36_motion_features PUBLIC cxx_std_17)
target_include_directories(r36_motion_features PUBLIC
  "${R36_MOTION_ROOT}/features/visual_motion/include"
  "${R36_MOTION_ROOT}/features/motion_reference/include"
  "${R36_MOTION_ROOT}/features/slip_detection/include"
  "${R36_MOTION_ROOT}/features/temporal/backend/include"
  "${R36_MOTION_ROOT}/sdk/include")
if(MSVC)
  target_compile_options(r36_motion_features PRIVATE /utf-8 /EHsc /fp:strict)
else()
  target_compile_options(r36_motion_features PRIVATE -Wall -Wextra -Wpedantic -fno-fast-math -ffp-contract=off)
endif()
if(BUILD_TESTING)
  add_executable(motion_algorithms_test "${R36_MOTION_ROOT}/validation/motion_algorithms_test.cpp")
  target_link_libraries(motion_algorithms_test PRIVATE r36_motion_features)
  add_test(NAME v8_visual_motion_reference_slip_contract COMMAND motion_algorithms_test)
endif()
