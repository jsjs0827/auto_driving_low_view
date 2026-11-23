# 当前回充后端：ArUco 同步 C++ 集成

当前编译的 donor 是 vendor/src/mark_aruco_detect.cpp、api_recharge_mark.cpp 及其配置解析器。使用 charing-apriltag 的 ArUco 识别、PnP 和姿态发布滤波，不是旧讲稿中的 L 标/YOLO 辅助裁框路径。original/、PROVENANCE.json、ADAPTATIONS.json 是原接口与集成来源记录，不能替代当前 CMake 和调用路径。

日常入口是 features/recharge/config.json：config=null 继承 SDK full.json.recharge.config；显式路径相对主 pipeline.json，指向一份真实的 recharge_mark.json。allow_source_fallback=null 继承 SDK（默认 false）。算法参数仍由 vendor/src/recharge_mark_config.cpp 读取，可使用根对象或嵌套 recharge_mark，包括 ROI、ArUco 字典/双码物理几何、安装姿态、run_every_n、min_interval_ms、use_hires 等。

graph 的 recharge 节点控制是否执行。scheduling/pipeline/src/full_pipeline.cpp::runRecharge 负责按帧数/最小间隔调度，通过 Engine::processLeftNv12 同步完成本帧，不启动 donor 独立 worker。intrinsics_source=dataset 保留 donor 仓内内参；sdk_scaled 从同帧参数按识别图尺寸缩放。高清输入必须与左图同帧、同时间戳，缺失时只有显式允许 fallback 才可退回左图。

allow_source_fallback 是 SDK 处理层规则。当前 camera/ROS 应用启用回充时仍采集/同步高清 GDC，此字段不会自动放宽上游输入要求；offline/recording 路径不含该高清源，启动器明确拒绝回充组合。

原上游仍包含进程全局配置/先验/滤波状态，因此 bridge 限制每个进程只初始化一个 Engine 并串行调用，命令切换时重置会话。合法但未识别到标记的帧属于正常结果，应检查 mark_ok/pose_ok；异常和输入错误才返回失败。overlay 拷贝进 Output::overlay_bgr，原对象中的指针不会被当作长期有效数据保留。

save_raw.dir 是厂商诊断的额外保存位置，当前直接由 donor 使用，请显式配置绝对输出路径。它与根 visualization.json 的结果/PLY 保存不同，不自动相对主 pipeline 转换。publish_enabled 等原发布设置也不等价于本项目 ROS 节点的发布开关，当前 bridge 不启动 donor ROS 发布线程。

构建由 R33_WITH_RECHARGE=ON 启用。Linux/X5 按当前 CMakeLists.txt 使用匹配的静态 OpenCV/TBB 创建私有 libperception_recharge_aruco.so，通过隐藏符号边界与 mapping 的动态 OpenCV 隔离，不向外传递 cv::Mat。需要实际的 R35_RECHARGE_STATIC_ROOT、OpenCV 开发配置及 RapidJSON；不能把不同平台/版本库混在一起。非 Linux 原生验证使用配置的主机 OpenCV，不代表目标库等价。

已有测试：r33_recharge_contract_tests 检查桥接/输入/生命周期；r35_aruco_fixtures 生成双码、单码、空白、错误 ID 等图片；r35_aruco_sequence_parity 对原 API 与 bridge 的序列结果做比较。测试源码在 validation/sdk/recharge/tests/，启用 BUILD_TESTING 后由 CTest 管理，无需项目 Python 程序。合成图契约通过不代表真实码图精度、X5 性能或 ARM 板端验收通过。
