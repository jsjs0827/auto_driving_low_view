set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
if(NOT DEFINED X5_SYSROOT OR NOT IS_DIRECTORY "${X5_SYSROOT}")
  message(FATAL_ERROR "Pass -DX5_SYSROOT=/absolute/path/to/matching/X5/sysroot")
endif()
set(X5_CXX_COMPILER aarch64-linux-gnu-g++ CACHE STRING "X5 SDK C++ compiler absolute path or command")
execute_process(
  COMMAND "${X5_CXX_COMPILER}" -dumpmachine
  RESULT_VARIABLE _x5_compiler_result
  OUTPUT_VARIABLE _x5_compiler_machine
  ERROR_VARIABLE _x5_compiler_error
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE)
if(NOT _x5_compiler_result STREQUAL "0")
  message(FATAL_ERROR
    "X5 compiler '${X5_CXX_COMPILER}' failed -dumpmachine: ${_x5_compiler_error}")
endif()
if(NOT _x5_compiler_machine MATCHES "^aarch64.*-linux-")
  message(FATAL_ERROR
    "X5 compiler must target aarch64 Linux; '${X5_CXX_COMPILER} -dumpmachine' returned '${_x5_compiler_machine}'")
endif()
set(CMAKE_CXX_COMPILER "${X5_CXX_COMPILER}")
if(NOT DEFINED X5_C_COMPILER)
  string(REGEX REPLACE "g\\+\\+$" "gcc" X5_C_COMPILER "${X5_CXX_COMPILER}")
endif()
set(CMAKE_C_COMPILER "${X5_C_COMPILER}")
set(CMAKE_SYSROOT "${X5_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH "${X5_SYSROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_TRY_COMPILE_PLATFORM_VARIABLES X5_SYSROOT X5_CXX_COMPILER X5_C_COMPILER)
unset(_x5_compiler_result)
unset(_x5_compiler_machine)
unset(_x5_compiler_error)
