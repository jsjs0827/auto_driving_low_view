add_library(offline_input STATIC src/offline_image.cpp)
target_compile_features(offline_input PUBLIC cxx_std_17)
target_include_directories(offline_input PUBLIC include PRIVATE vendor/stb)
