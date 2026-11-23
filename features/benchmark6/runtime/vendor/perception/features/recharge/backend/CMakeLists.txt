# DAG integration of charing-apriltag, with the original algorithm and JSON
# parser. The DAG caller owns acquisition and scheduling; no ROS/SHM is needed.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.20)
  project(r33_recharge_validation LANGUAGES CXX)
  include(CTest)
endif()
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d aruco)
find_path(R33_RECHARGE_RAPIDJSON_INCLUDE_DIR rapidjson/document.h
  HINTS "${R33_RECHARGE_RAPIDJSON_ROOT}" "${PERCEPTION_PROJECT_ROOT}/third_party/recharge/dependencies/include"
        "${HORIZON_ROOT}/include"
        "${HORIZON_SYSROOT}/include" "${R33_SYSROOT}/include"
        "$ENV{HORIZON_SYSROOT}/include" "$ENV{USR_X5}/include"
  REQUIRED)

# The replacement donor requires one internally consistent static OpenCV on X5.
# Isolate it in a private DSO; mapping continues using its existing dynamic CV.
# No cv::Mat or OpenCV symbol is exported at this boundary.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(R35_RECHARGE_STATIC_ROOT "${HORIZON_ROOT}" CACHE PATH "Matching target OpenCV/TBB prefix")
  if(NOT R35_RECHARGE_STATIC_ROOT)
    message(FATAL_ERROR "Set R35_RECHARGE_STATIC_ROOT to the matching target usr_x5 prefix")
  endif()
  set(_r35_libdir "${R35_RECHARGE_STATIC_ROOT}/lib/aarch64-linux-gnu")
  set(R35_RECHARGE_LIBS "")
  foreach(module aruco calib3d features2d flann imgproc core)
    set(archive "${_r35_libdir}/libopencv_${module}.a")
    if(NOT EXISTS "${archive}")
      message(FATAL_ERROR "charing-apriltag requires matching static OpenCV: ${archive}")
    endif()
    list(APPEND R35_RECHARGE_LIBS "${archive}")
  endforeach()
  set(_r35_tbb "${_r35_libdir}/libtbb.so.2")
  if(NOT EXISTS "${_r35_tbb}")
    message(FATAL_ERROR "Missing donor-compatible TBB: ${_r35_tbb}")
  endif()
  find_library(R35_RECHARGE_ZLIB NAMES z HINTS "${_r35_libdir}" "${R35_RECHARGE_STATIC_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
  list(APPEND R35_RECHARGE_LIBS "${_r35_tbb}" "${R35_RECHARGE_ZLIB}")
  set(_r35_backend_kind SHARED)
else()
  # Real native-host validation uses its installed OpenCV; it does not claim
  # target static-OpenCV equivalence or fabricate a Linux shared library.
  set(R35_RECHARGE_LIBS ${OpenCV_LIBS})
  set(_r35_backend_kind STATIC)
endif()

# Generate in the directory where imported targets are visible. The packager
# consumes these concrete target-file paths instead of out-of-scope variables.
set(_r33_linked_pending ${R35_RECHARGE_LIBS})
set(_r33_linked_seen "")
set(_r33_linked_content "")

while(_r33_linked_pending)
  list(POP_FRONT _r33_linked_pending _r33_linked_target)
  if(_r33_linked_target IN_LIST _r33_linked_seen)
    continue()
  endif()
  list(APPEND _r33_linked_seen "${_r33_linked_target}")
  if(TARGET "${_r33_linked_target}")
    get_target_property(_r33_linked_type "${_r33_linked_target}" TYPE)
    if(NOT _r33_linked_type STREQUAL "INTERFACE_LIBRARY")
      string(APPEND _r33_linked_content "$<TARGET_FILE:${_r33_linked_target}>\n")
    endif()
    get_target_property(_r33_linked_dependencies "${_r33_linked_target}" INTERFACE_LINK_LIBRARIES)
    foreach(_r33_linked_dependency IN LISTS _r33_linked_dependencies)
      if(_r33_linked_dependency MATCHES "^\\$<LINK_ONLY:([^>]+)>$")
        set(_r33_linked_dependency "${CMAKE_MATCH_1}")
      endif()
      if(TARGET "${_r33_linked_dependency}")
        list(APPEND _r33_linked_pending "${_r33_linked_dependency}")
      endif()
    endforeach()
  elseif(IS_ABSOLUTE "${_r33_linked_target}" AND EXISTS "${_r33_linked_target}")
    string(APPEND _r33_linked_content "${_r33_linked_target}\n")
  endif()
endwhile()
if(CMAKE_CONFIGURATION_TYPES)
  file(GENERATE OUTPUT "${PROJECT_BINARY_DIR}/r33_recharge_linked_$<CONFIG>.txt"
    CONTENT "${_r33_linked_content}")
  if("Release" IN_LIST CMAKE_CONFIGURATION_TYPES)
    set(_r33_linked_default_config Release)
  else()
    list(GET CMAKE_CONFIGURATION_TYPES 0 _r33_linked_default_config)
  endif()
  file(GENERATE OUTPUT "${PROJECT_BINARY_DIR}/r33_recharge_linked.txt"
    CONTENT "${_r33_linked_content}" CONDITION "$<CONFIG:${_r33_linked_default_config}>")
else()
  file(GENERATE OUTPUT "${PROJECT_BINARY_DIR}/r33_recharge_linked.txt" CONTENT "${_r33_linked_content}")
endif()

# Resolve the selected imported OpenCV targets, their transitive modules, and
# their real PE dependencies. In particular calib3d imports features2d/flann,
# while this OpenCV core imports its SDK's zlib.dll. Do not rely on PATH or copy
# unrelated DLLs from another OpenCV installation into the consuming process.
set(R33_OPENCV_RUNTIME_FILES "")
if(WIN32)
  set(_r33_opencv_pending ${OpenCV_LIBS})
  set(_r33_opencv_seen "")
  set(_r33_opencv_runtime_roots "")
  set(_r33_opencv_runtime_dirs "")
  while(_r33_opencv_pending)
    list(POP_FRONT _r33_opencv_pending _r33_opencv_target)
    if(NOT TARGET "${_r33_opencv_target}" OR _r33_opencv_target IN_LIST _r33_opencv_seen)
      continue()
    endif()
    list(APPEND _r33_opencv_seen "${_r33_opencv_target}")
    get_target_property(_r33_opencv_links "${_r33_opencv_target}" INTERFACE_LINK_LIBRARIES)
    foreach(_r33_opencv_link IN LISTS _r33_opencv_links)
      # Standard exported OpenCV modules use plain target names; also accept
      # the common LINK_ONLY wrapper without evaluating arbitrary expressions.
      if(_r33_opencv_link MATCHES "^\\$<LINK_ONLY:([^>]+)>$")
        set(_r33_opencv_link "${CMAKE_MATCH_1}")
      endif()
      if(TARGET "${_r33_opencv_link}")
        list(APPEND _r33_opencv_pending "${_r33_opencv_link}")
      endif()
    endforeach()
    get_target_property(_r33_opencv_configs "${_r33_opencv_target}" IMPORTED_CONFIGURATIONS)
    set(_r33_opencv_locations IMPORTED_LOCATION)
    foreach(_r33_opencv_config IN LISTS _r33_opencv_configs)
      string(TOUPPER "${_r33_opencv_config}" _r33_opencv_config)
      list(APPEND _r33_opencv_locations "IMPORTED_LOCATION_${_r33_opencv_config}")
    endforeach()
    foreach(_r33_opencv_property IN LISTS _r33_opencv_locations)
      get_target_property(_r33_opencv_location "${_r33_opencv_target}" "${_r33_opencv_property}")
      if(_r33_opencv_location AND EXISTS "${_r33_opencv_location}" AND
         _r33_opencv_location MATCHES "\\.[dD][lL][lL]$")
        list(APPEND _r33_opencv_runtime_roots "${_r33_opencv_location}")
        get_filename_component(_r33_opencv_runtime_dir "${_r33_opencv_location}" DIRECTORY)
        list(APPEND _r33_opencv_runtime_dirs "${_r33_opencv_runtime_dir}")
      endif()
    endforeach()
  endwhile()
  list(REMOVE_DUPLICATES _r33_opencv_runtime_roots)
  list(REMOVE_DUPLICATES _r33_opencv_runtime_dirs)
  if(NOT _r33_opencv_runtime_roots)
    message(FATAL_ERROR "No real imported OpenCV DLL locations were provided by the selected SDK")
  endif()
  get_filename_component(_r33_compiler_bin "${CMAKE_CXX_COMPILER}" DIRECTORY)
  if(MSVC)
    find_program(_r33_opencv_runtime_tool NAMES dumpbin HINTS "${_r33_compiler_bin}" REQUIRED)
    set(_r33_opencv_runtime_tool_name dumpbin)
  else()
    find_program(_r33_opencv_runtime_tool NAMES objdump HINTS "${_r33_compiler_bin}" REQUIRED)
    set(_r33_opencv_runtime_tool_name objdump)
  endif()
  set(_r33_opencv_manifest "${CMAKE_CURRENT_BINARY_DIR}/r33_opencv_runtime_files.cmake")
  set(_r33_opencv_scan_template [=[
set(CMAKE_GET_RUNTIME_DEPENDENCIES_PLATFORM "windows+pe")
set(CMAKE_GET_RUNTIME_DEPENDENCIES_TOOL "@_r33_opencv_runtime_tool_name@")
set(CMAKE_GET_RUNTIME_DEPENDENCIES_COMMAND "@_r33_opencv_runtime_tool@")
set(roots [==[@_r33_opencv_runtime_roots@]==])
set(directories [==[@_r33_opencv_runtime_dirs@]==])
file(GET_RUNTIME_DEPENDENCIES
  LIBRARIES ${roots}
  DIRECTORIES ${directories}
  RESOLVED_DEPENDENCIES_VAR resolved
  UNRESOLVED_DEPENDENCIES_VAR unresolved
  PRE_EXCLUDE_REGEXES "^[Aa][Pp][Ii]-[Mm][Ss]-" "^[Ee][Xx][Tt]-[Mm][Ss]-"
  POST_EXCLUDE_REGEXES "^[A-Za-z]:[/\\\\][Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\\\]")
if(unresolved)
  message(FATAL_ERROR "Selected OpenCV SDK has unresolved runtime dependencies: ${unresolved}")
endif()
set(files "${roots};${resolved}")
list(REMOVE_DUPLICATES files)
file(WRITE "@_r33_opencv_manifest@" "set(R33_OPENCV_RUNTIME_FILES [==[${files}]==])\n")
]=])
  string(CONFIGURE "${_r33_opencv_scan_template}" _r33_opencv_scan @ONLY)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/r33_scan_opencv_runtime.cmake" "${_r33_opencv_scan}")
  # Run dependency scanning in script mode, where GET_RUNTIME_DEPENDENCIES is
  # supported; imported target names need not escape this directory's scope.
  execute_process(COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/r33_scan_opencv_runtime.cmake"
    RESULT_VARIABLE _r33_opencv_scan_result OUTPUT_VARIABLE _r33_opencv_scan_output
    ERROR_VARIABLE _r33_opencv_scan_error)
  if(NOT _r33_opencv_scan_result EQUAL 0)
    message(FATAL_ERROR "OpenCV runtime audit failed: ${_r33_opencv_scan_error}")
  endif()
  include("${_r33_opencv_manifest}")
endif()
set_property(GLOBAL PROPERTY R33_OPENCV_RUNTIME_FILES "${R33_OPENCV_RUNTIME_FILES}")
if(NOT CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(R33_OPENCV_RUNTIME_FILES "${R33_OPENCV_RUNTIME_FILES}" PARENT_SCOPE)
endif()

# Call from the directory which owns each consuming target. An always-run
# dependency also repairs an existing build's DLL layout without relinking it.
function(r33_copy_opencv_runtime target)
  if(NOT WIN32)
    return()
  endif()
  if(NOT TARGET "${target}")
    message(FATAL_ERROR "r33_copy_opencv_runtime requires an existing target: ${target}")
  endif()
  get_property(_r33_runtime_files GLOBAL PROPERTY R33_OPENCV_RUNTIME_FILES)
  if(NOT _r33_runtime_files)
    message(FATAL_ERROR "OpenCV runtime files have not been audited")
  endif()
  if(TARGET "${target}_opencv_runtime")
    return()
  endif()
  add_custom_target("${target}_opencv_runtime"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:${target}>"
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${_r33_runtime_files} "$<TARGET_FILE_DIR:${target}>"
    VERBATIM)
  add_dependencies("${target}" "${target}_opencv_runtime")
endfunction()

add_library(r35_recharge_backend ${_r35_backend_kind}
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/backend_lifecycle.cpp"
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/src/mark_recharge_detect.cpp"
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/src/mark_aruco_detect.cpp"
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/src/recharge_mark_config.cpp"
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/src/recharge_mark_logger.cpp"
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/src/api_recharge_mark.cpp")
set_target_properties(r35_recharge_backend PROPERTIES POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES OUTPUT_NAME perception_recharge_aruco
  VERSION 1.0.35 SOVERSION 1)
target_include_directories(r35_recharge_backend PRIVATE "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend"
  "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/include" "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/src" "${R33_RECHARGE_RAPIDJSON_INCLUDE_DIR}" ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(r35_recharge_backend PRIVATE RECHARGE_MARK_WITH_SHM=0
  RECHARGE_MARK_WITH_AIPER_AI=0 _USE_MATH_DEFINES NOMINMAX)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_libraries(r35_recharge_backend PRIVATE "-Wl,--start-group" ${R35_RECHARGE_LIBS}
    "-Wl,--end-group" Threads::Threads ${CMAKE_DL_LIBS})
  target_link_options(r35_recharge_backend PRIVATE "-Wl,--no-undefined" "-Wl,-Bsymbolic"
    "-Wl,--exclude-libs,ALL" "-Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/backend.map")
  set_property(TARGET r35_recharge_backend APPEND PROPERTY LINK_DEPENDS "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/backend.map")
  set_target_properties(r35_recharge_backend PROPERTIES INSTALL_RPATH "$ORIGIN")
  install(TARGETS r35_recharge_backend LIBRARY DESTINATION lib COMPONENT SharedSDK)
else()
  target_link_libraries(r35_recharge_backend PUBLIC ${R35_RECHARGE_LIBS} Threads::Threads ${CMAKE_DL_LIBS})
endif()
add_library(r33_recharge_core STATIC "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/bridge.cpp")
set_target_properties(r33_recharge_core PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_compile_features(r33_recharge_core PUBLIC cxx_std_17)
if(MSVC)
  target_compile_options(r33_recharge_core PUBLIC /utf-8)
endif()
target_include_directories(r33_recharge_core PUBLIC "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend" "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/include")
target_link_libraries(r33_recharge_core PUBLIC r35_recharge_backend Threads::Threads)
foreach(t r33_recharge_core r35_recharge_backend)
  if(MSVC)
    target_compile_options(${t} PRIVATE /utf-8 /EHsc /GR)
  else()
    target_compile_options(${t} PRIVATE -frtti -fno-fast-math -ffp-contract=off)
  endif()
endforeach()

if(BUILD_TESTING)
  add_executable(r33_recharge_contract_tests "${PERCEPTION_PROJECT_ROOT}/validation/sdk/recharge/tests/contract_tests.cpp")
  target_link_libraries(r33_recharge_contract_tests PRIVATE r33_recharge_core)
  add_executable(r33_recharge_sequence_parity "${PERCEPTION_PROJECT_ROOT}/validation/sdk/recharge/tests/sequence_parity.cpp")
  target_link_libraries(r33_recharge_sequence_parity PRIVATE r33_recharge_core)
  r33_copy_opencv_runtime(r33_recharge_contract_tests)
  r33_copy_opencv_runtime(r33_recharge_sequence_parity)
  add_executable(r35_aruco_fixtures "${PERCEPTION_PROJECT_ROOT}/validation/sdk/recharge/tests/aruco_fixtures.cpp")
  target_include_directories(r35_aruco_fixtures PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(r35_aruco_fixtures PRIVATE ${OpenCV_LIBS})
  target_compile_features(r35_aruco_fixtures PRIVATE cxx_std_17)
  r33_copy_opencv_runtime(r35_aruco_fixtures)
  add_test(NAME r35_aruco_fixtures COMMAND r35_aruco_fixtures "${CMAKE_CURRENT_BINARY_DIR}/fixtures")
  set_tests_properties(r35_aruco_fixtures PROPERTIES FIXTURES_SETUP aruco_frames)
  set(_frames "")
  foreach(n dual offset left right blank wrong_id dual_repeat small_dual)
    list(APPEND _frames "${CMAKE_CURRENT_BINARY_DIR}/fixtures/${n}.ppm")
  endforeach()
  add_test(NAME r35_aruco_sequence_parity COMMAND r33_recharge_sequence_parity
    "${PERCEPTION_PROJECT_ROOT}/validation/sdk/recharge/tests/aruco_overlay.json" ${_frames})
  set_tests_properties(r35_aruco_sequence_parity PROPERTIES FIXTURES_REQUIRED aruco_frames)
  add_test(NAME r33_recharge_contract COMMAND r33_recharge_contract_tests
    "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/config/recharge_mark.json")
endif()

install(FILES "${PERCEPTION_PROJECT_ROOT}/features/recharge/backend/vendor/config/recharge_mark.json" DESTINATION share/perception_shared_sdk/config)
