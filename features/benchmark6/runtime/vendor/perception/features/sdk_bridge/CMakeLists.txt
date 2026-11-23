add_library(sdk_bridge STATIC src/sdk_caller.cpp)
target_compile_features(sdk_bridge PUBLIC cxx_std_17)
target_include_directories(sdk_bridge PUBLIC include)
target_link_libraries(sdk_bridge PUBLIC perception::sdk)
