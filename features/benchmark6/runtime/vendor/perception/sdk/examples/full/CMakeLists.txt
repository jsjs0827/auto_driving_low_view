cmake_minimum_required(VERSION 3.20)
project(perception_full_consumer LANGUAGES CXX)
set(PSDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." CACHE PATH "Installed perception SDK root")
set(PSDK_LINK_DEPENDENCY_DIRS "" CACHE STRING
  "Matching target sysroot vendor-library directories for link time only (semicolon separated)")
get_filename_component(PSDK_ROOT "${PSDK_ROOT}" ABSOLUTE)
find_path(PSDK_INCLUDE_DIR perception_full_sdk.h PATHS "${PSDK_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(PSDK_LIBRARY NAMES perception_sdk PATHS "${PSDK_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/full_demo.cpp")
  set(FULL_DEMO_SOURCE "${CMAKE_CURRENT_LIST_DIR}/full_demo.cpp")
else()
  set(FULL_DEMO_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../full_demo.cpp")
endif()
add_executable(perception_full_demo "${FULL_DEMO_SOURCE}")
target_compile_features(perception_full_demo PRIVATE cxx_std_17)
target_include_directories(perception_full_demo PRIVATE "${PSDK_INCLUDE_DIR}")
target_link_libraries(perception_full_demo PRIVATE "${PSDK_LIBRARY}")
set_target_properties(perception_full_demo PROPERTIES BUILD_RPATH "${PSDK_ROOT}/lib" INSTALL_RPATH "$ORIGIN/../lib")
if(WIN32)
  file(GLOB PSDK_RUNTIME_DLLS CONFIGURE_DEPENDS "${PSDK_ROOT}/bin/*.dll")
  foreach(PSDK_RUNTIME_DLL IN LISTS PSDK_RUNTIME_DLLS)
    add_custom_command(TARGET perception_full_demo POST_BUILD COMMAND ${CMAKE_COMMAND} -E
      copy_if_different "${PSDK_RUNTIME_DLL}" "$<TARGET_FILE_DIR:perception_full_demo>")
  endforeach()
endif()
if(MSVC)
  target_compile_options(perception_full_demo PRIVATE /utf-8 /EHsc)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/consumer_utf8.manifest" [=[<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<assembly xmlns="urn:schemas-microsoft-com:asm.v1" manifestVersion="1.0">
  <application xmlns="urn:schemas-microsoft-com:asm.v3"><windowsSettings>
    <activeCodePage xmlns="http://schemas.microsoft.com/SMI/2019/WindowsSettings">UTF-8</activeCodePage>
  </windowsSettings></application>
</assembly>
]=])
  target_link_options(perception_full_demo PRIVATE "/MANIFEST:EMBED" "/MANIFESTINPUT:${CMAKE_CURRENT_BINARY_DIR}/consumer_utf8.manifest")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(perception_full_demo PRIVATE "-Wl,-rpath-link,${PSDK_ROOT}/lib"
    "-Wl,-rpath-link,/usr/hobot/lib" "-Wl,-rpath-link,/usr_x5/hobot/lib")
  foreach(directory IN LISTS PSDK_LINK_DEPENDENCY_DIRS)
    target_link_options(perception_full_demo PRIVATE "-Wl,-rpath-link,${directory}")
  endforeach()
endif()
install(TARGETS perception_full_demo RUNTIME DESTINATION "${PSDK_ROOT}/bin")
