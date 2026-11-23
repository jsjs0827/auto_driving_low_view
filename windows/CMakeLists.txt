get_filename_component(_windows_project_root "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
if(NOT TARGET perception_native_config OR NOT TARGET perception::sdk OR NOT TARGET app_support)
  message(FATAL_ERROR "Windows offline debugging requires source SDK and native configuration targets")
endif()
add_executable(perception_windows_debug src/main.cpp)
file(GLOB _windows_debug_json CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/config/*.json")
target_sources(perception_windows_debug PRIVATE ${_windows_debug_json}
  "${_windows_project_root}/windows_debug.json"
  "${_windows_project_root}/CMakePresets.json"
  "${_windows_project_root}/docs/WINDOWS_VS2022调试.md")
source_group(TREE "${_windows_project_root}" FILES ${_windows_debug_json}
  "${_windows_project_root}/windows_debug.json"
  "${_windows_project_root}/CMakePresets.json"
  "${_windows_project_root}/docs/WINDOWS_VS2022调试.md")
target_compile_features(perception_windows_debug PRIVATE cxx_std_17)
target_include_directories(perception_windows_debug PRIVATE
  "${_windows_project_root}/features/offline/backend/include/native"
  "${_windows_project_root}/features/inference/framework/include"
  "${_windows_project_root}/core/runtime/include")
target_link_libraries(perception_windows_debug PRIVATE perception_native_config perception::sdk app_support)
set_target_properties(perception_windows_debug PROPERTIES
  VS_DEBUGGER_WORKING_DIRECTORY "${_windows_project_root}"
  VS_DEBUGGER_COMMAND_ARGUMENTS "--config \"${_windows_project_root}/windows_debug.json\"")
set(PERCEPTION_WINDOWS_DEBUG_TARGET perception_windows_debug PARENT_SCOPE)
if(MSVC)
  target_compile_options(perception_windows_debug PRIVATE /utf-8 /EHsc /fp:strict)
  target_link_options(perception_windows_debug PRIVATE /MANIFEST:EMBED
    "/MANIFESTINPUT:${_windows_project_root}/sdk/cmake/windows_utf8.manifest")
endif()
if(WIN32)
  find_file(_debug_ort_runtime onnxruntime.dll PATHS "${ORT_ROOT}/lib" "${ORT_ROOT}/bin" NO_DEFAULT_PATH REQUIRED)
  add_custom_command(TARGET perception_windows_debug POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:perception_sdk>" "$<TARGET_FILE_DIR:perception_windows_debug>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_debug_ort_runtime}" "$<TARGET_FILE_DIR:perception_windows_debug>")
endif()
if(MSVC)
  add_custom_command(TARGET perception_windows_debug POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_PDB_FILE:perception_sdk>" "$<TARGET_FILE_DIR:perception_windows_debug>")
endif()
install(TARGETS perception_windows_debug RUNTIME DESTINATION bin)
