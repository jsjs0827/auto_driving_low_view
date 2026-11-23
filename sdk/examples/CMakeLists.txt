cmake_minimum_required(VERSION 3.16)
project(perception_sdk_consumer LANGUAGES CXX)
set(PSDK_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." CACHE PATH "Installed perception SDK root")
set(PSDK_LINK_DEPENDENCY_DIRS "" CACHE STRING
  "Matching target sysroot vendor-library directories for link time only (semicolon separated)")
get_filename_component(PSDK_ROOT "${PSDK_ROOT}" ABSOLUTE)
find_path(PSDK_INCLUDE_DIR perception_sdk.h PATHS "${PSDK_ROOT}/include" NO_DEFAULT_PATH)
find_library(PSDK_LIBRARY NAMES perception_sdk PATHS "${PSDK_ROOT}/lib" NO_DEFAULT_PATH)
if(NOT PSDK_INCLUDE_DIR OR NOT PSDK_LIBRARY)
  message(FATAL_ERROR "PSDK_ROOT must contain include/perception_sdk.h and lib/libperception_sdk")
endif()
add_executable(my_demo "${PERCEPTION_PROJECT_ROOT}/sdk/examples/demo.cpp")
target_compile_features(my_demo PRIVATE cxx_std_17)
target_include_directories(my_demo PRIVATE "${PSDK_INCLUDE_DIR}")
target_link_libraries(my_demo PRIVATE "${PSDK_LIBRARY}")
set_target_properties(my_demo PROPERTIES BUILD_RPATH "${PSDK_ROOT}/lib" INSTALL_RPATH "$ORIGIN/../lib")
if(WIN32)
  # Place the matching DLLs beside the consumer. PATH alone can allow a system
  # onnxruntime.dll to win Windows DLL search and load an incompatible runtime.
  file(GLOB PSDK_RUNTIME_DLLS CONFIGURE_DEPENDS "${PSDK_ROOT}/bin/*.dll")
  foreach(PSDK_RUNTIME_DLL IN LISTS PSDK_RUNTIME_DLLS)
    if(NOT IS_DIRECTORY "${PSDK_RUNTIME_DLL}")
      add_custom_command(TARGET my_demo POST_BUILD COMMAND ${CMAKE_COMMAND} -E
        copy_if_different "${PSDK_RUNTIME_DLL}" "$<TARGET_FILE_DIR:my_demo>")
    endif()
  endforeach()
endif()
if(MSVC)
  target_compile_options(my_demo PRIVATE /utf-8 /EHsc)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/consumer_utf8.manifest" [=[<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<assembly xmlns="urn:schemas-microsoft-com:asm.v1" manifestVersion="1.0">
  <application xmlns="urn:schemas-microsoft-com:asm.v3">
    <windowsSettings>
      <activeCodePage xmlns="http://schemas.microsoft.com/SMI/2019/WindowsSettings">UTF-8</activeCodePage>
    </windowsSettings>
  </application>
</assembly>
]=])
  target_link_options(my_demo PRIVATE "/MANIFEST:EMBED"
    "/MANIFESTINPUT:${CMAKE_CURRENT_BINARY_DIR}/consumer_utf8.manifest")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # The X5 native linker can resolve the shared SDK's vendor dependencies here.
  target_link_options(my_demo PRIVATE
    "-Wl,-rpath-link,${PSDK_ROOT}/lib" "-Wl,-rpath-link,/usr/hobot/lib" "-Wl,-rpath-link,/usr_x5/hobot/lib")
  foreach(directory IN LISTS PSDK_LINK_DEPENDENCY_DIRS)
    target_link_options(my_demo PRIVATE "-Wl,-rpath-link,${directory}")
  endforeach()
endif()
install(TARGETS my_demo RUNTIME DESTINATION "${PSDK_ROOT}/bin")
