cmake_minimum_required(VERSION 3.16)

# This directory imports prebuilt X5 libraries; it never builds SDK sources.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(det_seg_cloud_sdk_x5_binary LANGUAGES NONE)
endif()

set(_dsc_binary_root "${CMAKE_CURRENT_LIST_DIR}")
foreach(_dsc_required
    include/det_seg.h include/det_cloud.h
    include/percep_main/det_seg_cloud_adapter.hpp
    include/percep_main/horizon_dnn_runner.hpp
    src/libdet_seg_cloud.so src/libdet_seg_cloud_adapter.so)
  if(NOT EXISTS "${_dsc_binary_root}/${_dsc_required}")
    message(FATAL_ERROR "Incomplete X5 binary SDK: ${_dsc_binary_root}/${_dsc_required}")
  endif()
endforeach()

if(TARGET DetSegCloud::SDK OR TARGET DetSegCloud::Adapter)
  message(FATAL_ERROR "DetSegCloud targets already exist; add this SDK directory only once")
endif()

add_library(DetSegCloud::SDK SHARED IMPORTED GLOBAL)
set_target_properties(DetSegCloud::SDK PROPERTIES
  IMPORTED_LOCATION "${_dsc_binary_root}/src/libdet_seg_cloud.so"
  INTERFACE_INCLUDE_DIRECTORIES "${_dsc_binary_root}/include"
  INTERFACE_LINK_OPTIONS "-Wl,-rpath-link,${_dsc_binary_root}/runtime")

add_library(DetSegCloud::Adapter SHARED IMPORTED GLOBAL)
set_target_properties(DetSegCloud::Adapter PROPERTIES
  IMPORTED_LOCATION "${_dsc_binary_root}/src/libdet_seg_cloud_adapter.so"
  INTERFACE_INCLUDE_DIRECTORIES "${_dsc_binary_root}/include"
  INTERFACE_COMPILE_FEATURES cxx_std_17
  INTERFACE_LINK_LIBRARIES DetSegCloud::SDK)

# Keep this layout: core SDK RPATH is $ORIGIN/../runtime; adapter RPATH is
# $ORIGIN:$ORIGIN/../runtime. Application and both SDK libraries are installed together.
# Imported targets cannot use install(TARGETS) on CMake 3.16, so install their
# exact TARGET_FILE paths here rather than repeating these rules in callers.
install(FILES
  "$<TARGET_FILE:DetSegCloud::SDK>"
  "$<TARGET_FILE:DetSegCloud::Adapter>"
  DESTINATION lib/ai_node)
install(DIRECTORY "${_dsc_binary_root}/runtime/"
  DESTINATION lib/runtime
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
  DIRECTORY_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(DIRECTORY "${_dsc_binary_root}/include/" DESTINATION include)
install(DIRECTORY "${_dsc_binary_root}/config/"
  DESTINATION share/det_seg_cloud_sdk_x5/config
  FILES_MATCHING PATTERN "*.json")
