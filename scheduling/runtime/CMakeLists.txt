add_library(dag_framework STATIC
  "${PERCEPTION_PROJECT_ROOT}/features/inference/framework/src/preprocess.cpp" "${PERCEPTION_PROJECT_ROOT}/features/inference/framework/src/nv12.cpp" "${PERCEPTION_PROJECT_ROOT}/features/inference/framework/src/preprocess_workspace.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/config/src/framework_config.cpp"
  "${PERCEPTION_PROJECT_ROOT}/features/inference/framework/src/backend.cpp" "${PERCEPTION_PROJECT_ROOT}/features/inference/framework/src/model_plan.cpp" "${PERCEPTION_PROJECT_ROOT}/features/inference/framework/src/decoders.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/runtime/src/pipeline.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/runtime/src/legacy.cpp"
  "${PERCEPTION_PROJECT_ROOT}/scheduling/runtime/src/engine.cpp" "${PERCEPTION_PROJECT_ROOT}/core/runtime/src/error.cpp" "${PERCEPTION_PROJECT_ROOT}/core/runtime/src/logging.cpp" "${PERCEPTION_PROJECT_ROOT}/core/runtime/src/fault_signal.cpp"
  "${PERCEPTION_PROJECT_ROOT}/scheduling/runtime/src/staged_inference.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/dag/src/task_graph.cpp" "${PERCEPTION_PROJECT_ROOT}/features/fusion/framework/src/semantic_fusion.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/config/src/runtime_config_v2.cpp"
  "${PERCEPTION_PROJECT_ROOT}/scheduling/config/src/runtime_config_json.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/pipeline/builtin/runtime_features.cpp")
target_include_directories(dag_framework PUBLIC "${PERCEPTION_PROJECT_ROOT}/scheduling/runtime/include")
target_link_libraries(dag_framework PUBLIC Threads::Threads)
target_link_libraries(dag_framework PUBLIC perception_algorithm_catalog)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  # Match the original framework's cross-toolchain rule. Static try_compile can
  # make FindThreads omit flags even when the target needs pthread support.
  target_compile_options(dag_framework PUBLIC -pthread)
  target_link_options(dag_framework PUBLIC -pthread)
endif()
if(MSVC)
  target_compile_options(dag_framework PRIVATE /utf-8 /EHsc /fp:precise)
else()
  target_compile_options(dag_framework PRIVATE -fno-fast-math)
endif()
