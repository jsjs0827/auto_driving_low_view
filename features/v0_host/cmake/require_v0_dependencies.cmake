function(perception_require_v0_dependencies)
  foreach(_root IN ITEMS PERCEPTION_V0_FRAMEWORK_ROOT PERCEPTION_V0_INTERFACE_INCLUDE)
    if(NOT ${_root})
      message(FATAL_ERROR "V0 plugin dependency gate: set ${_root} to the real ABI-matched upstream dependency; no fallback ABI is generated")
    endif()
  endforeach()
  set(_message_root "${PERCEPTION_V0_INTERFACE_INCLUDE}")
  if(NOT EXISTS "${_message_root}/common_interface/msg/grid_map_info.hpp" AND
      EXISTS "${PERCEPTION_V0_INTERFACE_INCLUDE}/ros_interface/common_interface/msg/grid_map_info.hpp")
    set(_message_root "${PERCEPTION_V0_INTERFACE_INCLUDE}/ros_interface")
  endif()
  set(PERCEPTION_V0_MESSAGE_INCLUDE "${_message_root}" PARENT_SCOPE)
  foreach(_header IN ITEMS
      "${PERCEPTION_V0_FRAMEWORK_ROOT}/include/base_framework/function_factory.h"
      "${PERCEPTION_V0_FRAMEWORK_ROOT}/include/base_framework/node_factory.h"
      "${PERCEPTION_V0_FRAMEWORK_ROOT}/include/base_framework/daye_shm.h"
      "${PERCEPTION_V0_FRAMEWORK_ROOT}/include/base_framework/base_utils.h"
      "${PERCEPTION_V0_INTERFACE_INCLUDE}/shm_struct/camera_struct.h"
      "${PERCEPTION_V0_INTERFACE_INCLUDE}/topic_macros.h"
      "${_message_root}/common_interface/msg/camera_param.hpp"
      "${_message_root}/common_interface/msg/grid_map_info.hpp"
      "${_message_root}/common_interface/msg/dock_pose.hpp"
      "${_message_root}/common_interface/srv/to_percep_cmd.hpp"
      "${_message_root}/common_interface/srv/get_percep_status.hpp"
      "${_message_root}/common_interface/srv/to_app_cmd.hpp")
    if(NOT EXISTS "${_header}")
      message(FATAL_ERROR "V0 plugin dependency gate: matching upstream header missing: ${_header}. The found older common_interface package cannot substitute for the 1104 ABI.")
    endif()
    file(READ "${_header}" _first LIMIT 64 HEX)
    if(NOT _first MATCHES "^(efbbbf)?(20|09|0a|0d)*(23|2f)")
      message(FATAL_ERROR "V0 plugin dependency gate: binary/encoded header ${_header}; use genuine readable upstream C++ source")
    endif()
  endforeach()
  file(READ "${PERCEPTION_V0_INTERFACE_INCLUDE}/shm_struct/camera_struct.h" _camera)
  foreach(_token IN ITEMS CameraGdcData left_data disp_data kCameraGdcFrameShm kGdcFrameShmSize)
    if(NOT _camera MATCHES "${_token}")
      message(FATAL_ERROR "V0 plugin dependency gate: camera_struct.h lacks ${_token}; the old PicRgdData ABI is incompatible with the 1104 producer")
    endif()
  endforeach()
  if(NOT EXISTS "${PERCEPTION_V0_BASE_FRAMEWORK_LIBRARY}")
    message(FATAL_ERROR "V0 plugin dependency gate: set PERCEPTION_V0_BASE_FRAMEWORK_LIBRARY to the matching platform's genuine libbase_framework.so")
  endif()
endfunction()
