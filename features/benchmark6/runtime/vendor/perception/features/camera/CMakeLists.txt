add_library(camera_input STATIC src/mo_camera.cpp)
target_compile_features(camera_input PUBLIC cxx_std_17)
target_include_directories(camera_input PUBLIC include vendor/mo)
target_link_libraries(camera_input PUBLIC r36_core_contracts)
