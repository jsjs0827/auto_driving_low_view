add_executable(r33_full_graph_contract "${PERCEPTION_PROJECT_ROOT}/validation/sdk/full/tests/graph_contract.cpp" "${PERCEPTION_PROJECT_ROOT}/scheduling/dag/src/full_graph.cpp")
target_include_directories(r33_full_graph_contract PRIVATE "${PERCEPTION_PROJECT_ROOT}/scheduling/config/include" "${PERCEPTION_PROJECT_ROOT}/sdk/include")
target_link_libraries(r33_full_graph_contract PRIVATE dag_framework runtime_status)
if(MSVC)
  target_compile_options(r33_full_graph_contract PRIVATE /utf-8 /EHsc)
else()
  target_compile_options(r33_full_graph_contract PRIVATE -fno-fast-math -ffp-contract=off)
endif()
add_test(NAME r33_full_native_TaskGraph_dependency_error_contract COMMAND r33_full_graph_contract)
set_tests_properties(r33_full_native_TaskGraph_dependency_error_contract PROPERTIES TIMEOUT 30)

add_executable(v3_configurable_graph_contract
  "${PERCEPTION_PROJECT_ROOT}/validation/sdk/full/tests/configurable_graph_contract.cpp"
  "${PERCEPTION_PROJECT_ROOT}/scheduling/dag/src/full_graph.cpp")
target_include_directories(v3_configurable_graph_contract PRIVATE
  "${PERCEPTION_PROJECT_ROOT}/scheduling/config/include" "${PERCEPTION_PROJECT_ROOT}/sdk/include")
target_link_libraries(v3_configurable_graph_contract PRIVATE dag_framework runtime_status)
add_test(NAME v3_user_graph_controls_real_execution COMMAND v3_configurable_graph_contract)
set_tests_properties(v3_user_graph_controls_real_execution PROPERTIES TIMEOUT 30)
