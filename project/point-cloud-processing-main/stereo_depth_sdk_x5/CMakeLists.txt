cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(stereo_depth_sdk_x5_binary LANGUAGES NONE)
endif()
set(_stereo_depth_root "${CMAKE_CURRENT_LIST_DIR}")
foreach(file include/percep_main/stereo_depth_runner.hpp src/libstereo_depth.so)
  if(NOT EXISTS "${_stereo_depth_root}/${file}")
    message(FATAL_ERROR "Incomplete stereo depth binary SDK: ${_stereo_depth_root}/${file}")
  endif()
endforeach()
add_library(StereoDepth::SDK SHARED IMPORTED GLOBAL)
set_target_properties(StereoDepth::SDK PROPERTIES
  IMPORTED_LOCATION "${_stereo_depth_root}/src/libstereo_depth.so"
  INTERFACE_INCLUDE_DIRECTORIES "${_stereo_depth_root}/include"
  INTERFACE_COMPILE_FEATURES cxx_std_17
  INTERFACE_LINK_OPTIONS "-Wl,-rpath-link,${_stereo_depth_root}/../det_seg_cloud_sdk_x5/runtime")
install(FILES "$<TARGET_FILE:StereoDepth::SDK>" DESTINATION lib/ai_node)
install(DIRECTORY "${_stereo_depth_root}/include/" DESTINATION include)
install(DIRECTORY "${_stereo_depth_root}/config/" DESTINATION share/stereo_depth_sdk_x5/config
  FILES_MATCHING PATTERN "*.json")
