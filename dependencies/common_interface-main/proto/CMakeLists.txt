cmake_minimum_required(VERSION 3.16)
project(mower_proto LANGUAGES CXX)

find_package(Protobuf REQUIRED)

set(MOWER_PROTO_FILE "${CMAKE_CURRENT_SOURCE_DIR}/mower.proto")

# Generate mower.pb.cc and mower.pb.h in the CMake build directory.
protobuf_generate_cpp(
  MOWER_PROTO_SOURCES
  MOWER_PROTO_HEADERS
  "${MOWER_PROTO_FILE}"
)

add_library(${PROJECT_NAME} STATIC
  ${MOWER_PROTO_SOURCES}
  ${MOWER_PROTO_HEADERS}
)

add_library(mower::proto ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include/mower_proto>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    protobuf::libprotobuf
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME mower_proto
  POSITION_INDEPENDENT_CODE ON
)

get_filename_component(
  MOWER_THIRDPARTY_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty"
  ABSOLUTE
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(PROTO_PLATFORM_DIR amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(X5|aarch64|arm64|AARCH64)$")
  set(PROTO_PLATFORM_DIR aarch64)
else()
  set(PROTO_PLATFORM_DIR ${CMAKE_SYSTEM_PROCESSOR})
endif()

set(MOWER_THIRDPARTY_INCLUDE_DIR "${MOWER_THIRDPARTY_DIR}/include")
set(MOWER_THIRDPARTY_LIB_DIR "${MOWER_THIRDPARTY_DIR}/lib/${PROTO_PLATFORM_DIR}")



# Keep the generated header and shared library available to other workspace
# projects through the existing thirdparty directories.
add_custom_target(mower_proto_copy_artifacts ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory
    "${MOWER_THIRDPARTY_INCLUDE_DIR}"
    "${MOWER_THIRDPARTY_LIB_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${MOWER_PROTO_HEADERS}"
    "${MOWER_THIRDPARTY_INCLUDE_DIR}/proto/mower.pb.h"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "$<TARGET_FILE:${PROJECT_NAME}>"
    "${MOWER_THIRDPARTY_LIB_DIR}/$<TARGET_FILE_NAME:${PROJECT_NAME}>"
  DEPENDS ${PROJECT_NAME}
  COMMENT "Copy mower protobuf artifacts to src/thirdparty/proto"
  VERBATIM
)

# install(
#   TARGETS ${PROJECT_NAME}
#   EXPORT mower_proto_targets
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(
#   FILES ${MOWER_PROTO_HEADERS}
#   DESTINATION include/mower_proto
# )

# install(
#   EXPORT mower_proto_targets
#   FILE mower_protoTargets.cmake
#   NAMESPACE mower::
#   DESTINATION lib/cmake/mower_proto
# )
