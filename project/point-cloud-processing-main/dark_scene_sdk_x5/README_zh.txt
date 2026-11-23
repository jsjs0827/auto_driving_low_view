V1 同步黑暗场景 SDK，ARM64

include/percep_main/dark_scene_runtime.hpp 为公开 C++ 接口，src/libdark_scene.so 为运行库。
使用 DarkScene::SDK 链接。evaluateExposure()、evaluateRaw()、readResult() 立即返回结果结构，
各个方法在同一实例内串行访问策略。mode1 支持双窗、周期和旧逐帧。预热与失效都可读取快照。
该功能 SDK 不依赖 ROS、DDS 或相机共享内存。子目录只含头、SO、配置及导入规则。
示例和字段说明见根目录 双窗口_配置与接入.txt。
