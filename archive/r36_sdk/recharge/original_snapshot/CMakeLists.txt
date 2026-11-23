cmake_minimum_required(VERSION 3.16)
project(recharge_mark)

# 独立二维码识别动态库：产出 librecharge_mark.so
# 感知仓入口：Aiper_Recharge::RechargeMarkHost（createRechargeMarkHost）
# 离线/nocam 仍可用 Aiper_Recharge::RechargeMarkApi

add_compile_options(-frtti)
add_compile_options(-Wno-psabi)
if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# ---------- sysroot（X5 交叉）：优先环境变量，再按相对路径探测 ----------
set(HORIZON_SYSROOT "")
if(DEFINED ENV{HORIZON_SYSROOT} AND EXISTS "$ENV{HORIZON_SYSROOT}")
  set(HORIZON_SYSROOT "$ENV{HORIZON_SYSROOT}")
elseif(DEFINED ENV{USR_X5} AND EXISTS "$ENV{USR_X5}")
  set(HORIZON_SYSROOT "$ENV{USR_X5}")
else()
  foreach(_cand
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../sysroot_docker/usr_x5"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../../sysroot_docker/usr_x5"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../sysroot_docker/usr_x5"
      "/home/st/code/ai_root/sysroot_docker/usr_x5")
    get_filename_component(_abs "${_cand}" REALPATH)
    if(EXISTS "${_abs}/lib/aarch64-linux-gnu")
      set(HORIZON_SYSROOT "${_abs}")
      break()
    endif()
  endforeach()
endif()

if(NOT HORIZON_SYSROOT)
  message(FATAL_ERROR
    "[recharge_mark] cannot locate X5 sysroot (usr_x5). "
    "Set HORIZON_SYSROOT or USR_X5, or sync this package under ai_ws/src/.")
endif()
message(STATUS "[recharge_mark] HORIZON_SYSROOT=${HORIZON_SYSROOT}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(common_interface REQUIRED)

# 与 percep_demo 头文件同一套 OpenCV 4.5.4；实现静态打进本 so（见下方）。
# 跨 so 只传 NV12/POD，不传 cv::Mat，避免和 percep_demo 动态 OpenCV 混用。
set(OpenCV_DIR "${HORIZON_SYSROOT}/lib/aarch64-linux-gnu/cmake/opencv4")
find_package(OpenCV REQUIRED)
message(STATUS "[recharge_mark] OpenCV ${OpenCV_VERSION}: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "[recharge_mark] OpenCV_LIBS=${OpenCV_LIBS}")

# SHM 帧布局（仅 .cpp 私用，不进对外头）
set(_RM_SHM_INC "")
foreach(_cand
    "${CMAKE_CURRENT_SOURCE_DIR}/../common_interface/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../ai_root/ai_ws/src/common_interface/include"
    "/home/st/code/ai_root/ai_ws/src/common_interface/include")
  if(EXISTS "${_cand}/shm_struct/camera_struct.h")
    set(_RM_SHM_INC "${_cand}")
    break()
  endif()
endforeach()
if(NOT _RM_SHM_INC)
  message(FATAL_ERROR
    "[recharge_mark] missing shm_struct/camera_struct.h (common_interface). "
    "在 ai_ws 中与 common_interface 同级编译，或设置该头所在 include 路径。")
endif()
message(STATUS "[recharge_mark] SHM include=${_RM_SHM_INC}")

add_library(recharge_mark SHARED
  "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/src/mark_recharge_detect.cpp"
  "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/src/mark_aruco_detect.cpp"
  "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/src/recharge_mark_config.cpp"
  "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/src/recharge_mark_logger.cpp"
  "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/src/api_recharge_mark.cpp"
  "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/src/recharge_mark_host.cpp"
)
# 源码头必须在 install/include 之前：后者可能残留旧拆分版/旧单头文件，
# 会盖掉 host LogSink 等新声明。
target_include_directories(recharge_mark BEFORE PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_options(recharge_mark PRIVATE -O3 -frtti)
target_compile_definitions(recharge_mark PRIVATE RECHARGE_MARK_WITH_AIPER_AI=0)
# Humble 的 ament_target_dependencies 走无关键字 target_link_libraries，
# 不能和下面 PRIVATE 混用。只链 cpp typesupport：板端 ai_node 的
# LD_LIBRARY_PATH 含 /app/daye/lib 与 humble，不含 /app/install/lib；
# 链 ${common_interface_TARGETS} 会把 fastrtps_c / generator_py 写进 NEEDED，
# dlopen libpercep_demo.so 直接失败。
target_include_directories(recharge_mark PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${common_interface_INCLUDE_DIRS}
)
target_link_libraries(recharge_mark PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  common_interface::common_interface__rosidl_typesupport_cpp
)
target_include_directories(recharge_mark
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/src"
    ${HORIZON_SYSROOT}/include
    ${OpenCV_INCLUDE_DIRS}
    ${_RM_SHM_INC}
)
target_link_libraries(recharge_mark PRIVATE
  dl
  pthread
  z
)
# 与 point-cloud-processingtest 相同：OpenCV 整份静态打进 so。
# 只链 aruco.a、core/imgproc 走动态 4.5.4 时，detectMarkers 的
# adaptiveThreshold 与 aruco 不是同一份实现，板端 cand=0。
# --exclude-libs 不把这些符号暴露给 percep_demo；跨 so 只传 NV12/POD。
set(_CV_LIBDIR "${HORIZON_SYSROOT}/lib/aarch64-linux-gnu")
set(RECHARGE_OPENCV_STATIC
  ${_CV_LIBDIR}/libopencv_aruco.a
  ${_CV_LIBDIR}/libopencv_calib3d.a
  ${_CV_LIBDIR}/libopencv_features2d.a
  ${_CV_LIBDIR}/libopencv_flann.a
  ${_CV_LIBDIR}/libopencv_imgproc.a
  ${_CV_LIBDIR}/libopencv_core.a
)
foreach(_f IN LISTS RECHARGE_OPENCV_STATIC)
  if(NOT EXISTS "${_f}")
    message(FATAL_ERROR "[recharge_mark] missing static OpenCV: ${_f}")
  endif()
endforeach()
set(RECHARGE_TBB_LIB "${_CV_LIBDIR}/libtbb.so.2")
if(NOT EXISTS "${RECHARGE_TBB_LIB}")
  message(FATAL_ERROR "[recharge_mark] missing ${RECHARGE_TBB_LIB}")
endif()
target_link_libraries(recharge_mark PRIVATE
  ${RECHARGE_OPENCV_STATIC}
  ${RECHARGE_TBB_LIB}
)
target_link_options(recharge_mark PRIVATE
  "-Wl,--no-as-needed"
  "-Wl,--exclude-libs,libopencv_aruco.a"
  "-Wl,--exclude-libs,libopencv_calib3d.a"
  "-Wl,--exclude-libs,libopencv_features2d.a"
  "-Wl,--exclude-libs,libopencv_flann.a"
  "-Wl,--exclude-libs,libopencv_imgproc.a"
  "-Wl,--exclude-libs,libopencv_core.a"
)
message(STATUS "[recharge_mark] static OpenCV+aruco (same as point-cloud-processingtest)")
set_target_properties(recharge_mark PROPERTIES
  BUILD_RPATH "\$ORIGIN"
  INSTALL_RPATH "\$ORIGIN"
  OUTPUT_NAME "recharge_mark"
)
target_link_directories(recharge_mark PRIVATE
  ${HORIZON_SYSROOT}/lib/aarch64-linux-gnu
)

# lib/ ：供 ament find_package / 链接
# lib/ai_node/ ：板端与 percep_demo 同目录部署
install(
  TARGETS recharge_mark
  EXPORT export_recharge_mark
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)
install(FILES $<TARGET_FILE:recharge_mark> DESTINATION lib/ai_node)
install(DIRECTORY "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/include/" DESTINATION include
        FILES_MATCHING PATTERN "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/*.h")
install(DIRECTORY "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/config/"
        DESTINATION config/ai_node
        FILES_MATCHING PATTERN "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/*.json")

ament_export_targets(export_recharge_mark HAS_LIBRARY_TARGET)
ament_export_include_directories("${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/recharge/original_snapshot/include")
ament_export_dependencies(rclcpp sensor_msgs common_interface)
ament_package()
