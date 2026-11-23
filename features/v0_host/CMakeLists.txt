if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.20)
  project(perception_v0_host_input LANGUAGES CXX)
endif()

set(PERCEPTION_V0_SDK_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../../sdk/include" CACHE PATH "SDK C ABI headers for standalone input-contract builds")
add_library(perception_v0_host_input STATIC src/input.cpp)
target_compile_features(perception_v0_host_input PUBLIC cxx_std_17)
target_include_directories(perception_v0_host_input PUBLIC include "${PERCEPTION_V0_SDK_INCLUDE}")
set_target_properties(perception_v0_host_input PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(TARGET sdk_bridge)
  add_library(perception_v0_host_core STATIC src/controller.cpp)
  target_compile_features(perception_v0_host_core PUBLIC cxx_std_17)
  target_link_libraries(perception_v0_host_core PUBLIC perception_v0_host_input sdk_bridge)
  set_target_properties(perception_v0_host_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

option(PERCEPTION_BUILD_V0_HOST_PLUGIN "Build the genuine V0 daye/ROS SHM host plugin" OFF)
if(NOT PERCEPTION_BUILD_V0_HOST_PLUGIN)
  return()
endif()
if(NOT TARGET perception_v0_host_core OR NOT TARGET stereo_provider)
  message(FATAL_ERROR "V0 host plugin requires the common sdk_bridge Session and existing stereo_provider targets")
endif()
set(PERCEPTION_V0_FRAMEWORK_ROOT "" CACHE PATH "Matched real V0 thirdparty root (include/base_framework and lib/<arch>)")
set(PERCEPTION_V0_INTERFACE_INCLUDE "" CACHE PATH "Matched V0 common_interface include root (shm_struct, topic_macros, common_interface/msg)")
set(PERCEPTION_V0_BASE_FRAMEWORK_LIBRARY "" CACHE FILEPATH "Matched real V0 libbase_framework shared library")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/require_v0_dependencies.cmake")
perception_require_v0_dependencies()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(common_interface REQUIRED)

# This checks actual declarations; symbol names alone cannot validate SHM layout.
# In particular, the locally found old PicRgdData ABI is not accepted.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES "${PERCEPTION_V0_FRAMEWORK_ROOT}/include" "${PERCEPTION_V0_INTERFACE_INCLUDE}" "${PERCEPTION_V0_MESSAGE_INCLUDE}" ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${common_interface_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES rclcpp::rclcpp)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
unset(PERCEPTION_V0_MATCHED_HEADER_CONTRACT CACHE)
check_cxx_source_compiles([=[
#include "base_framework/function_factory.h"
#include "base_framework/daye_shm.h"
#include "shm_struct/camera_struct.h"
#include "topic_macros.h"
#include "common_interface/msg/camera_param.hpp"
#include "common_interface/msg/grid_map_info.hpp"
#include "common_interface/msg/dock_pose.hpp"
#include "common_interface/srv/to_percep_cmd.hpp"
#include "common_interface/srv/get_percep_status.hpp"
#include <type_traits>
static_assert(std::is_array_v<decltype(CameraData::left_data)>);
static_assert(std::is_array_v<decltype(CameraData::disp_data)>);
static_assert(std::is_array_v<decltype(CameraGdcData::left_data)>);
int probe() {
  CameraData rgbd{}; CameraGdcData gdc{}; common_interface::msg::CameraParam p;
  daye::ChannelConfig c{kCameraGdcFrameShm,kGdcFrameShmSize,false};
  return int(rgbd.left.time_ns+rgbd.left.frame_id+rgbd.disp.bytes_used+
    rgbd.disp.width+rgbd.disp.height+rgbd.disp.frame_id+rgbd.disp.time_ns+
    gdc.left.width+p.base_line+p.bxf+p.fx+p.fy+p.cx+p.cy+p.width+p.height+
    kFrameShmSize+sizeof(kCameraFrameShm)+sizeof(kCameraParam)+sizeof(kToPercepCmd)+
    sizeof(kGetPercepStatus)+sizeof(kGridMapInfo)+sizeof(kToApplicationCmd));
}
]=] PERCEPTION_V0_MATCHED_HEADER_CONTRACT)
unset(CMAKE_REQUIRED_INCLUDES)
unset(CMAKE_REQUIRED_LIBRARIES)
unset(CMAKE_TRY_COMPILE_TARGET_TYPE)
if(NOT PERCEPTION_V0_MATCHED_HEADER_CONTRACT)
  message(FATAL_ERROR "V0 dependency ABI gate failed: supply the real headers/messages matching the 1104 CameraData + CameraGdcData producer. Old headers and synthetic ABI substitutes are unsupported. See CMakeError.log.")
endif()

add_library(perception_v0_host_plugin SHARED src/percep_main_plugin.cpp)
set_target_properties(perception_v0_host_plugin PROPERTIES OUTPUT_NAME percep_main)
target_compile_features(perception_v0_host_plugin PRIVATE cxx_std_17)
target_include_directories(perception_v0_host_plugin PRIVATE "${PERCEPTION_V0_FRAMEWORK_ROOT}/include" "${PERCEPTION_V0_INTERFACE_INCLUDE}" "${PERCEPTION_V0_MESSAGE_INCLUDE}")
target_link_libraries(perception_v0_host_plugin PRIVATE perception_v0_host_core stereo_provider "${PERCEPTION_V0_BASE_FRAMEWORK_LIBRARY}")
target_link_options(perception_v0_host_plugin PRIVATE "$<$<PLATFORM_ID:Linux>:-Wl,--no-undefined>")
ament_target_dependencies(perception_v0_host_plugin rclcpp std_msgs common_interface)
# Installation belongs to the host integration build. No V0 dependency payload
# or private local_only transport is copied into the ordinary SDK installation.
