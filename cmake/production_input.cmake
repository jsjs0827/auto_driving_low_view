include_guard(GLOBAL)

if(PERCEPTION_ENABLE_LOCAL_CAMERA_SHM)
  message(FATAL_ERROR "This source tree supports the production host, direct camera and offline inputs.")
endif()
