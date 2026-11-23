# Both the native root build and the ROS build ship the same user configuration
# tree. SDK-owned profiles, extrinsics and mapping templates are installed by
# the SDK SharedSDK component. Models are copied next to pipeline.json so the
# relative feature paths work after installation or moving the install tree.
# Vendor camera SDK files are still provided by the device environment.
function(perception_install_runtime_config source_root destination)
  install(FILES
    "${source_root}/README_PLUGINS_zh.md"
    "${source_root}/scene_cls_camera.json"
    "${source_root}/DAG_FORMAT_zh.md"
    "${source_root}/pipeline.json"
    "${source_root}/benchmark.json"
    "${source_root}/crop_compare.json"
    "${source_root}/camera_benchmark.json"
    "${source_root}/camera_detseg.json"
    "${source_root}/camera_original_q8.json"
    "${source_root}/camera_stereo.json"
    "${source_root}/camera_benchmark_visualization.json"
    "${source_root}/camera.json"
    "${source_root}/slip_camera.json"
    "${source_root}/slip_recording.json"
    "${source_root}/slip_ros.json"
    "${source_root}/rviz_cloud.json"
    "${source_root}/rviz_visualization.json"
    "${source_root}/rviz.json"
    "${source_root}/rviz_pipeline.json"
    "${source_root}/visualization.json"
    "${source_root}/parameter_map.json"
    DESTINATION "${destination}")
  file(GLOB _perception_feature_configs CONFIGURE_DEPENDS
    "${source_root}/features/*/config.json")
  foreach(_perception_feature_config IN LISTS _perception_feature_configs)
    file(RELATIVE_PATH _perception_feature_relative
      "${source_root}/features" "${_perception_feature_config}")
    get_filename_component(_perception_feature_name "${_perception_feature_relative}" DIRECTORY)
    install(FILES "${_perception_feature_config}"
      DESTINATION "${destination}/features/${_perception_feature_name}")
  endforeach()
  install(FILES "${source_root}/features/inference/cpu_profile.json"
    DESTINATION "${destination}/features/inference")
  install(DIRECTORY "${source_root}/models/"
    DESTINATION "${destination}/models")
  install(DIRECTORY "${source_root}/calibration/"
    DESTINATION "${destination}/calibration")
  install(DIRECTORY "${source_root}/config/"
    DESTINATION "${destination}/config")
  # The matrix shell runner calls the Python evidence checker. These three
  # runtime helpers are intentionally shipped despite the general *.py exclude.
  install(FILES
    "${source_root}/tools/configure_inference_matrix.py"
    "${source_root}/tools/check_inference_matrix_run.py"
    "${source_root}/tools/summarize_inference_matrix.py"
    DESTINATION "${destination}/tools")
  install(FILES "${source_root}/docs/V6_三种输入六组推理测试.md"
    DESTINATION "${destination}/docs")
endfunction()
