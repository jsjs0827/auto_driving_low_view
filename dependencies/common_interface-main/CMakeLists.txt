cmake_minimum_required(VERSION 3.16)
project(common_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 依赖apt update && apt install -y protobuf-compiler
# find_package(Protobuf REQUIRED)
# add_subdirectory(proto)

set(MSG_FILES
  msg/Imu.msg
  msg/Motor.msg
  msg/MotorNew.msg
  msg/MotorSet.msg
  msg/MotorSetNew.msg
  msg/GridMapInfo.msg
  msg/Bat.msg
  msg/McuSignal.msg
  msg/Odometer.msg
  msg/ButtonKey.msg
  msg/LightCtrl.msg
  msg/McuOta.msg
  msg/McuOtaStatus.msg
  msg/DockPose.msg
  msg/HealthCode.msg
  msg/MowingStatus.msg
  msg/M720Status.msg
  msg/M7xxFirmwareUpgradeProgress.msg
  msg/LidarUpgradeProgress.msg
  msg/MowerStatus.msg
  msg/McuCtrl.msg
  msg/WorkMapPoint.msg
  msg/WorkMapElement.msg
  msg/WorkMapZone.msg
  msg/WorkMap.msg
  msg/CameraParam.msg
  msg/RemoteCtrl.msg
  msg/PathWay.msg
)

set(SRV_FILES
  srv/Sum.srv
  srv/ToAppCmd.srv
  srv/ToNaviCmd.srv
  srv/ToSlamCmd.srv
  srv/ToPercepCmd.srv
  srv/ToCameraCmd.srv
  srv/GetNaviStatus.srv
  srv/GetPercepStatus.srv
  srv/GetSlamStatus.srv
  srv/IotCustomCmd.srv
  srv/GetWorkMap.srv
  srv/RTKVlsam.srv
  srv/M7xxFirmwareUpgrade.srv
  srv/LidarUpgrade.srv
  srv/PoseSeed.srv
  srv/EnterProductionTestMode.srv
  srv/ExitProductionTestMode.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs
)

set(GENERATED_CPP_HEADER_DST
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ros_interface/${PROJECT_NAME}
)
set(GENERATED_CPP_HEADER_SRC_DIRS
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/${PROJECT_NAME}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_cpp/${PROJECT_NAME}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_fastrtps_cpp/${PROJECT_NAME}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_typesupport_introspection_cpp/${PROJECT_NAME}
)
string(JOIN "," GENERATED_CPP_HEADER_SRC_DIRS_ARG ${GENERATED_CPP_HEADER_SRC_DIRS})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|AARCH64")
  add_custom_target(${PROJECT_NAME}_copy_generated_cpp_headers ALL
    COMMAND ${CMAKE_COMMAND}
      -DGENERATED_HEADER_DST=${GENERATED_CPP_HEADER_DST}
      "-DGENERATED_HEADER_SRC_DIRS=${GENERATED_CPP_HEADER_SRC_DIRS_ARG}"
      -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/copy_generated_cpp_headers.cmake
    DEPENDS
      ${PROJECT_NAME}__rosidl_generator_cpp
      ${PROJECT_NAME}__rosidl_typesupport_cpp
      ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp
      ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp
    COMMENT "Copy generated C++ interface headers to include/ros_interface"
  )
else()
  
endif()

install(
  DIRECTORY include/
  DESTINATION include/common_interface
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime std_msgs)
ament_package()
