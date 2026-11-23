# Shared plane operations for the online provider, comparison and standalone CLI.
# This target intentionally has no Horizon dependency, so geometry is host-testable.
if(NOT TARGET stereo_nv12)
  get_filename_component(_nv12_project_root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
  add_library(stereo_nv12 STATIC "${_nv12_project_root}/features/stereo/runner/src/nv12.cpp")
  target_compile_features(stereo_nv12 PUBLIC cxx_std_17)
  target_include_directories(stereo_nv12 PUBLIC "${_nv12_project_root}/features/stereo/runner/include")
  target_include_directories(stereo_nv12 SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(stereo_nv12 PUBLIC ${OpenCV_LIBS})
endif()
