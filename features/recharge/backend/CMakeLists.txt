# V0 algorithm-bearing prebuilt SDK integration. The DAG still owns input,
# scheduling and publication. The original DSO retains its real ROS/common_interface/
# TBB ELF runtime dependencies even though this bridge does not start a ROS Host.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.20)
  project(recharge_v0_sdk_integration LANGUAGES CXX)
  include(CTest)
endif()
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "The selected original V0 SDK is Linux aarch64; no substitute/mock backend is provided")
endif()
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _v0_processor)
if(NOT _v0_processor MATCHES "^(aarch64|arm64)$")
  message(FATAL_ERROR "The selected original V0 SDK is Linux aarch64 only; configure a real aarch64 toolchain or turn R33_WITH_RECHARGE OFF")
endif()
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_path(R33_RECHARGE_RAPIDJSON_INCLUDE_DIR rapidjson/document.h
  HINTS "${R33_RECHARGE_RAPIDJSON_ROOT}" "${PERCEPTION_PROJECT_ROOT}/third_party/recharge/dependencies/include"
        "${HORIZON_ROOT}/include" "${HORIZON_SYSROOT}/include" "${R33_SYSROOT}/include"
  REQUIRED)
set(_v0_root "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/v0-sdk")
set(_v0_library "${_v0_root}/lib/aarch64/librecharge_mark.so")
set(_v0_header "${_v0_root}/include/percep_demo/api_recharge_mark.h")
file(SHA256 "${_v0_library}" _v0_library_sha)
file(SHA256 "${_v0_header}" _v0_header_sha)
if(NOT _v0_library_sha STREQUAL "8e458ce9a48aa8555c2452e09b3edc57baa27fbe44a6db77ddbfc8fc751e9f0b")
  message(FATAL_ERROR "Selected original V0 SDK bytes differ from the audited baseline")
endif()
if(NOT _v0_header_sha STREQUAL "bd3cbfa3df3883b59a9039579c137c8f885f53f53232d3b16fb8c0c8018cda3e")
  message(FATAL_ERROR "Selected V0 API header differs from its matching original ABI")
endif()
foreach(_zlib IN LISTS ZLIB_LIBRARIES)
  if(_zlib MATCHES "\\.a$")
    message(FATAL_ERROR "The V0 shim requires the target shared zlib, not a static archive")
  endif()
endforeach()
set(R36_RECHARGE_ORIGINAL_V0 ON CACHE INTERNAL "Original V0 algorithm SDK is active" FORCE)
# Compatibility function for consumers. Linux has no OpenCV DLL copy operation;
# the real original V0 and shim installation rules below carry this backend.
function(r33_copy_opencv_runtime target)
endfunction()

# Only the small private loader is compiled here. NONE of vendor/src, its old
# factory, private static OpenCV, or backend_lifecycle.cpp are linked anymore.
add_library(r35_recharge_backend SHARED
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/v0_runtime.cpp")
set_target_properties(r35_recharge_backend PROPERTIES
  POSITION_INDEPENDENT_CODE ON CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES
  OUTPUT_NAME perception_recharge_v0 VERSION 1.0.0 SOVERSION 1
  CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES
  BUILD_RPATH "$ORIGIN" INSTALL_RPATH "$ORIGIN")
target_include_directories(r35_recharge_backend PRIVATE
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend" ${ZLIB_INCLUDE_DIRS})
target_link_libraries(r35_recharge_backend PRIVATE ${ZLIB_LIBRARIES} ${CMAKE_DL_LIBS} Threads::Threads)
# The version script limits exports. Declarations get default visibility only
# for these private bridge functions; original SDK/OpenCV names are not exported.
set_source_files_properties("${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/v0_runtime.cpp"
  PROPERTIES COMPILE_DEFINITIONS "RECHARGE_V0_RUNTIME_BUILD=1")
target_link_options(r35_recharge_backend PRIVATE "-Wl,--no-undefined"
  "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/v0_runtime.map")
set_property(TARGET r35_recharge_backend APPEND PROPERTY LINK_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/v0_runtime.map")
add_custom_command(TARGET r35_recharge_backend POST_BUILD
  COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_v0_library}"
    "$<TARGET_FILE_DIR:r35_recharge_backend>/librecharge_mark.so" VERBATIM)
install(TARGETS r35_recharge_backend LIBRARY DESTINATION lib COMPONENT SharedSDK)
# Preserve original filename/bytes. This is runtime-dlopen-only, never a public
# SDK DT_NEEDED or a renamed substitute for the previous ArUco backend.
install(FILES "${_v0_library}" DESTINATION lib COMPONENT SharedSDK)

add_library(r33_recharge_core STATIC "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/bridge.cpp")
set_target_properties(r33_recharge_core PROPERTIES POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(r33_recharge_core PUBLIC
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend"
  "${_v0_root}/include" PRIVATE "${R33_RECHARGE_RAPIDJSON_INCLUDE_DIR}")
target_link_libraries(r33_recharge_core PUBLIC r35_recharge_backend Threads::Threads)
foreach(target r33_recharge_core r35_recharge_backend)
  target_compile_options(${target} PRIVATE -frtti -fno-fast-math -ffp-contract=off)
endforeach()
if(BUILD_TESTING)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
  add_executable(r35_aruco_fixtures "${PERCEPTION_PROJECT_ROOT}/validation/sdk/recharge/tests/aruco_fixtures.cpp")
  target_include_directories(r35_aruco_fixtures PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(r35_aruco_fixtures PRIVATE ${OpenCV_LIBS})
  target_compile_features(r35_aruco_fixtures PRIVATE cxx_std_17)
  add_test(NAME r35_aruco_fixtures COMMAND r35_aruco_fixtures "${CMAKE_CURRENT_BINARY_DIR}/fixtures")
  set_tests_properties(r35_aruco_fixtures PROPERTIES FIXTURES_SETUP aruco_frames)
  add_executable(r33_recharge_contract_tests
    "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/v0_contract_tests.cpp")
  target_include_directories(r33_recharge_contract_tests PRIVATE "${R33_RECHARGE_RAPIDJSON_INCLUDE_DIR}")
  target_link_libraries(r33_recharge_contract_tests PRIVATE r33_recharge_core)
  add_test(NAME recharge_V0_real_API_bridge_lifecycle_ownership_no_accuracy_claim
    COMMAND r33_recharge_contract_tests
      "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/config/recharge_mark.json"
      "${CMAKE_CURRENT_BINARY_DIR}/owned-contract-runs")
endif()
install(FILES "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/config/recharge_mark.json"
  DESTINATION share/perception_shared_sdk/config)
