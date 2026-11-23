cmake_minimum_required(VERSION 3.16)
project(percep_main)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
add_compile_options(-frtti)
add_compile_options(-Wno-psabi)
if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Keep the production default; validation builds may skip the independent
# recharge SDK with -DPERCEP_ENABLE_RECHARGE=OFF.
option(PERCEP_ENABLE_RECHARGE "Build recharge marker integration" ON)

# Cross builds must not resolve x86 host libraries through fallback searches.
if(CMAKE_CROSSCOMPILING)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
else()
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
  set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
endif()
# 获取 Git 信息
execute_process(
    COMMAND git rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

# 定义编译宏（如果 Git 获取失败，使用默认值）
if(NOT GIT_COMMIT)
    set(GIT_COMMIT "unknown")
endif()
if(NOT GIT_BRANCH)
    set(GIT_BRANCH "unknown")
endif()

add_definitions(
    -DGIT_COMMIT="${GIT_COMMIT}${GIT_DIRTY}"
    -DGIT_BRANCH="${GIT_BRANCH}"
)

# pcap 是 pcl_io 的依赖。从 CMakeLists.txt 自身位置推算 sysroot 路径，
# 不依赖任何环境变量（TROS_ROOT、CMAKE_SYSROOT 等在不同机器/Docker 中各不同）。
# CMakeLists.txt 在 ai_ws/src/ai_node/point-cloud-processing/，
# 上 4 级 -> ai_root/ -> sysroot_docker/usr_x5
get_filename_component(_PCAP_SYSROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../sysroot_docker/usr_x5"
    REALPATH)
set(HORIZON_SYSROOT "${_PCAP_SYSROOT}" CACHE PATH "X5 runtime prefix containing include/ and lib/")
# The X5 prefix also contains transitive ROS dependencies such as spdlog/fmt.
# Search that target prefix before system paths in the cross-build container.
list(PREPEND CMAKE_PREFIX_PATH "${HORIZON_SYSROOT}")
if(CMAKE_CROSSCOMPILING)
  # Explicit prefixes supplied by the X5 build script include target ROS and
  # the matching vendor package install. Keep those usable with root-only find.
  list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_PREFIX_PATH})
  list(REMOVE_DUPLICATES CMAKE_FIND_ROOT_PATH)
endif()
if(EXISTS "${HORIZON_SYSROOT}/lib/aarch64-linux-gnu/libpcap.so")
  set(PCAP_LIBRARY
      "${HORIZON_SYSROOT}/lib/aarch64-linux-gnu/libpcap.so"
      CACHE FILEPATH "pcap library" FORCE)
  set(PCAP_INCLUDE_DIR
      "${HORIZON_SYSROOT}/include"
      CACHE FILEPATH "pcap include dir" FORCE)
  message(STATUS "[pcap] pinned: ${PCAP_LIBRARY}")
else()
  message(WARNING "[pcap] not found at ${HORIZON_SYSROOT}")
endif()

add_definitions(-DHAVE_PCAP)
add_definitions(-DPCL_NO_PRECOMPILE)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|AARCH64")
  set(THIRDPARTY_PLATFORM aarch64)
else()
  set(THIRDPARTY_PLATFORM amd64)
endif()

set(THIRDPARTY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty" CACHE PATH "daye third-party SDK root")
set(COMMON_INTERFACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../common_interface" CACHE PATH "daye common_interface source root")

# message(WARNING "CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
set(DAYE_DEMO_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")
set(DAYE_DEMO_SRC_DIR     "${CMAKE_SOURCE_DIR}/src")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(common_interface REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters features kdtree octree registration sample_consensus search segmentation surface)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ZLIB REQUIRED)



# Demo so
set(PERCEP_DEMO_SRCS
  src/percep_main.cpp
  src/pcl_common.cpp
  src/patchworkpp.cpp
  src/offline.cpp
  src/percep_init.cpp
  src/percep_compute.cpp
  src/rampRecovery.cpp
)

list(APPEND PERCEP_DEMO_SRCS src/dnn_runtime.cpp
      src/label_fusion.cpp src/stereo_runtime.cpp src/raw_lux_runtime.cpp)

# Replace only the detector/segmenter and cloud matching implementation.
# The public application entry points and existing model/config files remain.
set(PERCEP_DSC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/det_seg_cloud_sdk_x5"
    CACHE PATH "Optimized X5 binary SDK root")
add_subdirectory("${PERCEP_DSC_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/det_seg_cloud_binary")
set(PERCEP_STEREO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/stereo_depth_sdk_x5"
    CACHE PATH "X5 stereo binary SDK root")
add_subdirectory("${PERCEP_STEREO_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/stereo_depth_binary")
set(PERCEP_RAW_LUX_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/raw_lux_sdk_x5"
    CACHE PATH "X5 native RAW light classification binary SDK root")
add_subdirectory("${PERCEP_RAW_LUX_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/raw_lux_binary")
set(PERCEP_DARK_SCENE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/dark_scene_sdk_x5" CACHE PATH "X5 dark-scene binary SDK")
add_subdirectory("${PERCEP_DARK_SCENE_ROOT}" "${CMAKE_CURRENT_BINARY_DIR}/dark_scene_binary")

set(PERCEP_V2_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/perception_v2_sdk_x5"
    CACHE PATH "V2 X5 SDK containing matching headers and shared libraries")
add_library(PercepV2::SDK SHARED IMPORTED)
set_target_properties(PercepV2::SDK PROPERTIES
    IMPORTED_LOCATION "${PERCEP_V2_SDK_ROOT}/lib/libperception_sdk.so.1"
    INTERFACE_INCLUDE_DIRECTORIES "${PERCEP_V2_SDK_ROOT}/include")
add_library(PercepV2::Recharge SHARED IMPORTED)
set_target_properties(PercepV2::Recharge PROPERTIES
    IMPORTED_LOCATION "${PERCEP_V2_SDK_ROOT}/lib/libperception_recharge_v0.so.1")
add_library(${PROJECT_NAME} SHARED ${PERCEP_DEMO_SRCS}
    src/v2_sdk_runtime.cpp src/v2_host_bridge.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY INSTALL_RPATH "$ORIGIN")
install(FILES "${PERCEP_V2_SDK_ROOT}/lib/libperception_sdk.so.1"
              "${PERCEP_V2_SDK_ROOT}/lib/libperception_recharge_v0.so.1"
        DESTINATION lib/ai_node)

# Keep SDK boundary validation and finite checks under strict floating-point rules.
set_source_files_properties(src/dnn_runtime.cpp src/stereo_runtime.cpp
  src/raw_lux_runtime.cpp src/percep_main.cpp
  src/percep_compute.cpp src/offline.cpp src/v2_sdk_runtime.cpp src/v2_host_bridge.cpp PROPERTIES COMPILE_OPTIONS "-fno-fast-math")
target_compile_options(${PROJECT_NAME} PRIVATE
  -O3
  -fvisibility=hidden
  -march=armv8.2-a
  -ffast-math
  -funroll-loops  
)
target_link_directories(${PROJECT_NAME} PRIVATE
  ${THIRDPARTY_ROOT}/lib/${THIRDPARTY_PLATFORM}
)
target_link_libraries(${PROJECT_NAME}
  base_framework
  dl
  pthread
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS}
  ZLIB::ZLIB
)

target_link_libraries(${PROJECT_NAME}
    DetSegCloud::Adapter
    StereoDepth::SDK
    RawLux::SDK
    DarkScene::SDK
    "-Wl,--push-state,--no-as-needed"
    PercepV2::Recharge
    "-Wl,--pop-state"
    PercepV2::SDK
  )
  target_link_directories(${PROJECT_NAME} PRIVATE
    ${HORIZON_SYSROOT}/lib
  )

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${THIRDPARTY_ROOT}/include
    ${COMMON_INTERFACE_ROOT}/include
    ${COMMON_INTERFACE_ROOT}/include/ros_interface
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR} 
    ${OpenCV_INCLUDE_DIRS}
)

  target_include_directories(${PROJECT_NAME} PRIVATE
    ${HORIZON_SYSROOT}/include
  )

target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    FUNCTION_NAME="${PROJECT_NAME}"
    PCL_NO_PRECOMPILE
  PUBLIC
    PERCEP_ENABLE_RECHARGE=$<BOOL:${PERCEP_ENABLE_RECHARGE}>
)
if(PERCEP_ENABLE_RECHARGE)
  ament_export_definitions(PERCEP_ENABLE_RECHARGE=1)
else()
  ament_export_definitions(PERCEP_ENABLE_RECHARGE=0)
endif()


# ---------- 回充二维码：submodule 钉 charging.git（头 + so，入口 RechargeMarkHost）----------
set(_RM_SDK "${CMAKE_CURRENT_SOURCE_DIR}/recharge_mark" CACHE PATH "Recharge marker SDK root")
set(_RM_SO "${_RM_SDK}/lib/aarch64/librecharge_mark.so")
set(_RM_INC "${_RM_SDK}/include")
if(PERCEP_ENABLE_RECHARGE)
  if(NOT EXISTS "${_RM_SO}")
  message(FATAL_ERROR
    "[recharge_mark] missing ${_RM_SO}\n"
    "  git submodule update --init --recursive\n"
    "  或在 recharge_mark_new: bash /home/st/code/recharge_mark_new/scripts/build_and_export.sh")
  endif()
  if(NOT EXISTS "${_RM_INC}/percep_demo/api_recharge_mark.h")
  message(FATAL_ERROR "[recharge_mark] missing ${_RM_INC}/percep_demo/api_recharge_mark.h")
  endif()
  add_library(recharge_mark SHARED IMPORTED)
  set_target_properties(recharge_mark PROPERTIES
  IMPORTED_LOCATION "${_RM_SO}"
  INTERFACE_INCLUDE_DIRECTORIES "${_RM_INC}"
  )
  target_link_libraries(${PROJECT_NAME} recharge_mark)
  message(STATUS "[recharge_mark] IMPORTED ${_RM_SO}")
  install(FILES "${_RM_SO}" DESTINATION lib/ai_node)
else()
  message(STATUS "[recharge_mark] disabled for this build")
endif()


# 运行时与 librecharge_mark.so 同目录（lib/ai_node）
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH "$ORIGIN"
  INSTALL_RPATH "$ORIGIN"
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  common_interface
  sensor_msgs
  pcl_conversions
  visualization_msgs
  tf2_ros
)

# 安装配置文件
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/
        DESTINATION config/ai_node/
        FILES_MATCHING PATTERN "*.yaml" PATTERN "*.json" PATTERN "*.cfg")
# 回充子仓配置：recharge_mark/config/recharge_mark.json 同步到 install/config/ai_node/
# 与上面 config/ 一起落到 /app/daye/config/ai_node/recharge_mark.json
if(PERCEP_ENABLE_RECHARGE AND EXISTS "${_RM_SDK}/config/recharge_mark.json")
  install(FILES "${_RM_SDK}/config/recharge_mark.json"
          DESTINATION config/ai_node/)
elseif(PERCEP_ENABLE_RECHARGE)
  message(WARNING "[recharge_mark] config missing: ${_RM_SDK}/config/recharge_mark.json")
endif()
# 安装库（librecharge_mark.so 由上面 IMPORTED 目标 install(FILES) 拷到 lib/ai_node）
install(
  TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION lib/ai_node
)

#安装模型
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/model/
        DESTINATION model/
        )

#安装版本文件
file(GLOB VERSION_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/V*.txt"
)

install(
  FILES ${VERSION_FILES}
  DESTINATION config/ai_node
)

ament_package()
