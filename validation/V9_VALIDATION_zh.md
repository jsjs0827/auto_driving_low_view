# v9 验证记录

本记录区分编译/接口/真实 CPU 算法验证与 X5 物理运行。当前没有远程连接用户 X5；不宣称已取得板端 A/B 性能、真实相机采集或算法质量胜负。

- 输入/DAG 测试 12 项：左右缺失、raw 字节长度、捕获身份重复/回退、Q8 类型、内参、路径约束、DAG 选择/依赖等。
- 报告测试 9 项：warmup 排除、配对统计、加权 CPU、完整循环 FPS、PLY 读入、缺失侧、采集资源独立统计等。
- 新录制器 15 个 ABI 测试场景：owned buffer、同帧左右/Q8、参考内参 0/1、缺符号/坏内参/畸变/未知尺寸、重试/超时、release/close 错误、SIGINT 恢复原模式、逐帧和整段资源计时。使用明确的相机 ABI fixture，不冒充硬件相机。
- Linux x86 original-only 和真正 Horizon 版本均成功编译；C++ 契约测试通过。完整 ROS demo/raw_camera/stereo/compare/capture 构建成功，CTest 3/3。
- 实际 ROS2 DDS 测试收到相同 SDK 时间戳的左 NV12、右 NV12 和 Q8；字节数分别 522240/522240/696320，右目 frame_id 独立。输入仍为 ABI fixture。
- 使用工作区解包的真实 PCL/Eigen/Boost/FLANN、真实 ONNX Runtime，原 r36 源码以 MAP ON + ORT 构建成功，未替换原融合实现。
- 真实 CPU 检测/分割 + 原 MappingPipeline/Patchwork/AiFusion 回放：1 帧预热、4 帧测量、20 份 SDK 实际云 PLY。与 SDK cloud.bin 的坐标 float32、RGB、label 抽样逐位一致。输入视差来自已有 recorded_npy 验证数据，**不是录制的原相机 baseline，不作 A/B 精度证据**。
- 完整 JSON DAG 前端 → 原 NV12/Q8 回放 → 运行配置生成 → 真实 CPU SDK → 五类 PLY → summary/HTML 的流程通过；另用真实图像验证运行配置的原 RGBD 着色开关。
- CLI 负路径 10/10：参数、无双目构建、缺 mapping SDK、真实旧 x86 DNN 无法加载新 BIN 等均明确失败，无 fallback、无假成功输出。
- Chrome 实际 WebGL 与 Canvas2D 两条路径验证同步旋转/平移/缩放、同范围、缺失侧、RGB 为 0 提示和真实 SDK 非零点云显示。合成夹具用于交互测试，不作为算法效果。
- Python/JSON 全部解析成功，交付 shell 脚本 bash -n 通过。旧 v8p1 的 184 文件 SHA256/LF 校验仍通过。
- AArch64 录制器及比较核心均实际编译/链接成功。比较核心使用真实 r36 BPU + PCL mapping SDK 构建验证，SDK 未改源；最终工具依赖外部 libperception_sdk.so.1，无构建机 RPATH/RUNPATH。ARM 可执行文件未在本机执行，板端速度及正确性仍待 X5 运行。

原方法回放中的 Q8 已预先生成；新 stereo 的推理计算发生在 SDK disparity Task 内。报告对此单独提示，不能把两路 FPS 直接解释成两套完整实时相机系统的吞吐率。原相机采集进程资源单独记录，不能与回放 CPU 简单相加。

原 SDK camera_cloud 留存在上色前；真实 RGBD 颜色在 robot_cloud，semantic 为类别色。SDK 默认 debug.visualize=false 会使 robot RGB 全 0。v9 的有效运行配置对两路同时启用原 RGBD 着色，保留原 PLY 数据，不在 HTML 中伪造颜色。

SDK 参考内参不等于已验证的全套双目几何真值；绝对深度精度、左右矫正投影关系和模型适用场景仍需实物标尺/真实 X5 数据确认。本地旧 Horizon SDK 无法执行现有新版本 BIN 的失败有保留，未用 SGBM 冒充新模型。

具体 JSON 证据见本目录 evidence；编译和原始测试工作记录在工作区 local_env/stereo_compare_v9、local_env/stereo_compare_v9_capture、local_env/stereo_compare_v9_arm 与 local_env/compare_report_validation。
