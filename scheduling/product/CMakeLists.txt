add_library(product_dag STATIC "${PERCEPTION_PROJECT_ROOT}/scheduling/product/src/product_dag.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/product/src/product_events.cpp" "${PERCEPTION_PROJECT_ROOT}/features/offline/backend/src/offline_input.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/product/src/status_api.cpp")
target_include_directories(product_dag PUBLIC "${PERCEPTION_PROJECT_ROOT}/scheduling/product/include" "${PERCEPTION_PROJECT_ROOT}/scheduling/runtime/include")
target_link_libraries(product_dag PUBLIC dag_framework runtime_status platform_metrics)
if(MSVC)
  target_compile_options(product_dag PRIVATE /utf-8 /EHsc)
endif()
if(BUILD_TESTING)
  add_executable(product_dag_tests "${PERCEPTION_PROJECT_ROOT}/validation/sdk/product/tests/product_dag_tests.cpp")
  target_link_libraries(product_dag_tests PRIVATE product_dag)
  if(MSVC)
    target_compile_options(product_dag_tests PRIVATE /utf-8 /EHsc /fp:strict)
  else()
    target_compile_options(product_dag_tests PRIVATE -fno-fast-math -ffp-contract=off)
  endif()
  add_test(NAME product_original_framework_order_failure_modes_projection
    COMMAND product_dag_tests "${PERCEPTION_PROJECT_ROOT}/scheduling/product/config/product_dag.json")
  foreach(_test IN ITEMS product_command offline_input)
    add_executable(${_test}_tests "${PERCEPTION_PROJECT_ROOT}/validation/sdk/product/tests/${_test}_tests.cpp")
    target_link_libraries(${_test}_tests PRIVATE product_dag)
    if(MSVC)
      target_compile_options(${_test}_tests PRIVATE /utf-8 /EHsc)
      target_link_options(${_test}_tests PRIVATE "/MANIFEST:EMBED" "/MANIFESTINPUT:${PROJECT_SOURCE_DIR}/cmake/windows_utf8.manifest")
    endif()
    add_test(NAME ${_test}_contract COMMAND ${_test}_tests)
  endforeach()
  add_executable(product_event_tests "${PERCEPTION_PROJECT_ROOT}/validation/sdk/tests/product_event_tests.cpp")
  target_link_libraries(product_event_tests PRIVATE product_dag)
  if(MSVC)
    target_compile_options(product_event_tests PRIVATE /utf-8 /EHsc)
  endif()
  add_test(NAME product_actual_event_graphs_and_callback_isolation COMMAND product_event_tests "${PERCEPTION_PROJECT_ROOT}/scheduling/product/config/product_events.json")
  set_tests_properties(product_actual_event_graphs_and_callback_isolation PROPERTIES TIMEOUT 30)
  # This host test covers module identity and receipt I/O only. It does not
  # substitute for compiling against the real ROS/daye SHM ABI on Ubuntu.
  add_executable(ros_io_identity_tests "${PERCEPTION_PROJECT_ROOT}/validation/sdk/product/tests/ros_io_identity_tests.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/product/src/ros_io_identity.cpp")
  target_include_directories(ros_io_identity_tests PRIVATE "${PERCEPTION_PROJECT_ROOT}/scheduling/product/include" "${PERCEPTION_PROJECT_ROOT}/features/inference/preprocess/include")
  target_link_libraries(ros_io_identity_tests PRIVATE runtime_status ${CMAKE_DL_LIBS})
  if(MSVC)
    target_compile_options(ros_io_identity_tests PRIVATE /utf-8 /EHsc)
    target_link_options(ros_io_identity_tests PRIVATE "/MANIFEST:EMBED" "/MANIFESTINPUT:${PROJECT_SOURCE_DIR}/cmake/windows_utf8.manifest")
  endif()
  add_test(NAME original_ros_plugin_identity_receipt COMMAND ros_io_identity_tests)
endif()

# The installed dependency probe must use the same declarations as CMake.
install(FILES "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/dependency_headers.json" DESTINATION product)
install(FILES "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/include/percep_demo/api_recharge_mark.h" DESTINATION product/include/percep_demo)
install(DIRECTORY "${PERCEPTION_PROJECT_ROOT}/scheduling/product/config/" DESTINATION product/config)

option(PERCEPTION_BUILD_PRODUCT "Build real ROS2/daye/PCL/PatchWork++/AiFusion/recharge plugin" OFF)
option(PRODUCT_WITH_RECHARGE "Compile and link the original recharge SDK integration" ON)
set(PRODUCT_DEPS_ROOT "" CACHE PATH "Original ai_ws/src containing thirdparty and common_interface")
set(RECHARGE_SDK_ROOT "" CACHE PATH "Matched original recharge SDK root with include/percep_demo and lib/aarch64")
set(PRODUCT_CROSS_PREFIX_PATH "" CACHE STRING "Explicit AArch64 ROS/workspace package prefixes outside the sysroot (semicolon separated)")
if(NOT PERCEPTION_BUILD_PRODUCT)
  return()
endif()

# Refuse encoded/binary copies of original headers before searching ROS/PCL.
# The local original tree sometimes contains protected binary .h files. They
# cannot compile and must never be replaced with synthetic ABI declarations.
if(NOT PRODUCT_DEPS_ROOT)
  set(_product_candidate "${PROJECT_SOURCE_DIR}/../gecao/ai_ws/src")
  if(EXISTS "${_product_candidate}/thirdparty/include/base_framework/function_factory.h")
    get_filename_component(PRODUCT_DEPS_ROOT "${_product_candidate}" ABSOLUTE)
  else()
    message(FATAL_ERROR "Full product needs -DPRODUCT_DEPS_ROOT=<original ai_ws/src with readable source headers>")
  endif()
endif()
function(require_product_source_header path)
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR "Full product dependency header missing: ${path}")
  endif()
  file(READ "${path}" _head LIMIT 64 HEX)
  if(NOT _head MATCHES "^(efbbbf)?(20|09|0a|0d)*(23|2f)")
    message(FATAL_ERROR "Full product dependency is a binary/encoded header: ${path}. Use the original Ubuntu project's readable, compilable copy; fake SDK headers are not supported.")
  endif()
endfunction()
file(READ "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/dependency_headers.json" _dependency_headers)
string(JSON _dependency_header_count LENGTH "${_dependency_headers}" headers)
math(EXPR _dependency_last "${_dependency_header_count} - 1")
foreach(_index RANGE 0 ${_dependency_last})
  string(JSON _relative GET "${_dependency_headers}" headers ${_index} path)
  require_product_source_header("${PRODUCT_DEPS_ROOT}/${_relative}")
endforeach()
if(NOT R20_WITH_BPU OR NOT TARGET original_runner)
  message(FATAL_ERROR "Full product requires R20_WITH_BPU=ON and the original Linux AArch64 model runner")
endif()
if(PRODUCT_WITH_RECHARGE)
if(NOT RECHARGE_SDK_ROOT)
  message(FATAL_ERROR "Full product needs RECHARGE_SDK_ROOT with the ABI-matched original recharge header and AArch64 so")
endif()
require_product_source_header("${RECHARGE_SDK_ROOT}/include/percep_demo/api_recharge_mark.h")
file(READ "${RECHARGE_SDK_ROOT}/include/percep_demo/api_recharge_mark.h" _sdk_header)
file(READ "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/include/percep_demo/api_recharge_mark.h" _product_header)
string(REPLACE "\r\n" "\n" _sdk_header "${_sdk_header}")
string(REPLACE "\r\n" "\n" _product_header "${_product_header}")
string(ASCII 239 187 191 _product_utf8_bom)
string(REGEX REPLACE "^${_product_utf8_bom}" "" _sdk_header "${_sdk_header}")
string(REGEX REPLACE "^${_product_utf8_bom}" "" _product_header "${_product_header}")
# A reviewed source snapshot changes this comment only. No structure member,
# function pointer, declaration, or initialization expression may differ.
string(REPLACE "    // ABI 空实现（ArUco 路径已移除）"
  "    // ArUco：字典 / 目标 id / 码边长(m) / 重投影门限(px)"
  _sdk_header "${_sdk_header}")
if(NOT _sdk_header STREQUAL _product_header)
  message(FATAL_ERROR "Recharge SDK header does not match the preserved product ABI. Supply the matching original header/so pair; a newer RechargeMarkHost SDK cannot be silently substituted.")
endif()
find_library(PRODUCT_RECHARGE_LIBRARY NAMES recharge_mark
  PATHS "${RECHARGE_SDK_ROOT}/lib/aarch64" "${RECHARGE_SDK_ROOT}/lib" "${RECHARGE_SDK_ROOT}" "${RECHARGE_SDK_ROOT}/lib/ai_node"
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED)
endif()
find_library(PRODUCT_DAYE_LIBRARY NAMES base_framework
  PATHS "${PRODUCT_DEPS_ROOT}/thirdparty/lib/aarch64" NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH REQUIRED)
set(_product_vendor_libraries "${PRODUCT_DAYE_LIBRARY}")
if(PRODUCT_WITH_RECHARGE)
  list(APPEND _product_vendor_libraries "${PRODUCT_RECHARGE_LIBRARY}")
else()
  message(STATUS "Product recharge integration disabled: no recharge header or library required")
endif()
foreach(_library IN LISTS _product_vendor_libraries)
  file(READ "${_library}" _elf LIMIT 20 HEX)
  if(NOT _elf MATCHES "^7f454c460201" OR NOT _elf MATCHES "${PERCEPTION_PROJECT_ROOT}/scheduling/product/b700$")
    message(FATAL_ERROR "Full product requires actual little-endian AArch64 ELF library: ${_library}")
  endif()
endforeach()
# Keep exact/default root-only search policy. Explicit AArch64 package prefixes
# are additional roots only in this product directory, never host auto-search.
foreach(_prefix IN LISTS PRODUCT_CROSS_PREFIX_PATH)
  if(NOT IS_ABSOLUTE "${_prefix}" OR NOT IS_DIRECTORY "${_prefix}")
    message(FATAL_ERROR "PRODUCT_CROSS_PREFIX_PATH must contain existing absolute AArch64 package prefixes: ${_prefix}")
  endif()
  list(APPEND CMAKE_PREFIX_PATH "${_prefix}")
  list(APPEND CMAKE_FIND_ROOT_PATH "${_prefix}")
endforeach()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(common_interface REQUIRED)
find_package(pcl_conversions REQUIRED)

# The Windows original workspace snapshot contains a common_interface checkout
# without CameraParam. Use the generated package from the working original ROS
# workspace, rather than inventing a message layout or removing the subscriber.
set(_product_camera_param_hints ${common_interface_INCLUDE_DIRS})
foreach(_prefix IN LISTS PRODUCT_CROSS_PREFIX_PATH)
  list(APPEND _product_camera_param_hints "${_prefix}/include" "${_prefix}/include/common_interface")
endforeach()
find_path(PRODUCT_CAMERA_PARAM_INCLUDE_DIR common_interface/msg/camera_param.hpp
  HINTS ${_product_camera_param_hints}
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
if(NOT PRODUCT_CAMERA_PARAM_INCLUDE_DIR)
  message(FATAL_ERROR "The matching original common_interface package must provide common_interface/msg/camera_param.hpp. Pass its AArch64 install prefix in PRODUCT_CROSS_PREFIX_PATH; the local source snapshot without CameraParam cannot substitute for the original ROS ABI.")
endif()
require_product_source_header("${PRODUCT_CAMERA_PARAM_INCLUDE_DIR}/common_interface/msg/camera_param.hpp")
enable_language(C)
find_package(PCL REQUIRED COMPONENTS common filters search kdtree features sample_consensus segmentation)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)

add_library(product_percep_demo SHARED
  "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/src/percep_demo.cpp" "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/src/pcl_common.cpp" "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/src/patchworkpp.cpp" "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/src/offline.cpp"
  "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/src/dnn_runtime.cpp" "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/src/label_fusion.cpp" "${PERCEPTION_PROJECT_ROOT}/features/inference/product/src/exact_dnn_runner.cpp" "${PERCEPTION_PROJECT_ROOT}/archive/r36_sdk/product/src/product_image_codec.cpp"
  "${PERCEPTION_PROJECT_ROOT}/scheduling/product/src/ros_io_identity.cpp")
set_target_properties(product_percep_demo PROPERTIES OUTPUT_NAME percep_demo
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/product"
  BUILD_RPATH "\$ORIGIN" INSTALL_RPATH "\$ORIGIN")
target_include_directories(product_percep_demo PRIVATE
  "${PERCEPTION_PROJECT_ROOT}/scheduling/product/include" "${PERCEPTION_PROJECT_ROOT}/scheduling/runtime/include" "${PERCEPTION_PROJECT_ROOT}/features/inference/preprocess/include"
  "${PERCEPTION_PROJECT_ROOT}/third_party/stb"
  "${PRODUCT_DEPS_ROOT}/thirdparty/include" "${PRODUCT_DEPS_ROOT}/common_interface/include"
  "${PRODUCT_DEPS_ROOT}/common_interface/include/ros_interface"
  ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(product_percep_demo PRIVATE
  PERCEP_DNN_ENABLE=1 PCL_NO_PRECOMPILE FUNCTION_NAME="percep_demo"
  PERCEPTION_PRODUCT_WITH_RECHARGE=$<BOOL:${PRODUCT_WITH_RECHARGE}>)
target_compile_options(product_percep_demo PRIVATE -O3 -frtti -fno-fast-math -ffp-contract=off)
target_link_libraries(product_percep_demo PRIVATE product_dag dag_exact_features original_runner
  app_support ${_product_vendor_libraries}
  ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp OpenMP::OpenMP_CXX ${CMAKE_DL_LIBS})
target_link_options(product_percep_demo PRIVATE "-Wl,--no-undefined")
ament_target_dependencies(product_percep_demo rclcpp std_msgs sensor_msgs common_interface
  pcl_conversions visualization_msgs tf2_ros)
foreach(_static IN ITEMS product_dag dag_framework runtime_status dag_exact_features original_runner
    exact_input legacy_exact legacy_reference seg_post hardware_routes app_support execution_schedule output_features)
  if(TARGET ${_static})
    set_target_properties(${_static} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  endif()
endforeach()
install(TARGETS product_percep_demo LIBRARY DESTINATION lib/ai_node)
if(PRODUCT_WITH_RECHARGE)
  install(FILES "${PRODUCT_RECHARGE_LIBRARY}" DESTINATION lib/ai_node)
endif()
install(DIRECTORY "${PERCEPTION_PROJECT_ROOT}/scheduling/product/config/" DESTINATION config/ai_node FILES_MATCHING PATTERN "${PERCEPTION_PROJECT_ROOT}/scheduling/product/*.json" PATTERN "${PERCEPTION_PROJECT_ROOT}/scheduling/product/*.yaml")
install(DIRECTORY "${PERCEPTION_PROJECT_ROOT}/scheduling/config/profiles/" DESTINATION share/perception_x5/profiles)
install(DIRECTORY "${PERCEPTION_PROJECT_ROOT}/scheduling/product/rviz/" DESTINATION share/perception_x5/rviz)
