# 回充后端：选定V0原算法SDK移植

## 已知未修复红项：原SDK可选绘图

当前选定8e原SO在`publish_overlay=true`的1280×1088合成双码输入上，首帧于原库`BuildOverlayImage → cv::rectangle`内部崩溃。相同NV12在不链接V2、也不预载动态OpenCV的原API独立进程，以及显式预载动态OpenCV的原API进程中，均在同一指令复现。反汇编显示原SO内调用方传入Rect值寄存器，而被调函数把该参数当指针读取；这支持原库内部参数ABI不一致的判断，确切上游构建原因尚缺源码证据。此问题没有修改或修复，不能声称可选vendor overlay路径已通过。

本次2641帧姿态一致性验收使用正式默认`publish_overlay=false`；公开C ABI正向契约使用同样算法配置，另通过V2 `output.save_overlay`生成SDK自有source RGB，验证真实pose、details和source RGB跨帧及Engine销毁后的归属。该正向契约不覆盖原SO可选overlay生命周期。`save_result.enabled=true`仍映射到原接口，但本轮并未验证该独立厂商保存路径，不能由姿态通过推断它的绘图/保存安全性。原overlay=true失败程序、配置、调用栈、双侧对照均单独留痕，需上游提供修正原库后再验。

当前V2通过私有 `libperception_recharge_v0.so.1` 调用原名 `librecharge_mark.so`。算法由用户选定的charing-main.zip内原SO承载，SHA256为 `8e458ce9a48aa8555c2452e09b3edc57baa27fbe44a6db77ddbfc8fc751e9f0b`，原提交a80cbbe。缺少这版完整算法源码，不能声称重写了AprilTag、PnP或位姿滤波；本次没有优化这些算法。来源及原vendor快照记录见PROVENANCE.json，原头和SO在v0-sdk/。

`vendor/src/`、旧backend_lifecycle和旧ArUco backend只保留历史，不再编译/链接。真API表必须使用v0-sdk里的匹配头，不能将原SO改名冒充旧backend：二者函数表偏移不同。公开perception SDK C ABI和DAG节点/配置入口保持原用法。

初始化依次调用真实factory、`init("",cfg.json)`、可选保存开关、`resetSession()`；每帧直接调用真实`processLeftNv12`，不逐帧清状态。原SDK完整解析ROI、检测后端、AprilTag、安装几何、PnP、符号守卫和连续有效帧门控等配置。三个活跃recharge_mark.json默认配置与选定ZIP算法参数相同。不同序列需新进程或显式任务边界复位，不得把合法但pose_ok=false的预热结果算作有效零位姿。

`save_result.enabled`是V2适配层的I/O开关：false调用原SDK关闭厂商结果图写入，true恢复原写入能力，未提供时不覆盖原SDK默认。默认显式false，以延续V2不额外保存厂商结果图的行为。该项不改变检测阈值、ROI或publish_overlay；SDK公开的结果保存接口仍照原用法工作。原SDK诊断输出目录需显式指定，测试必须使用独立owned目录。桥不创建Host，因此不执行Host启动清理/ROS发布流程。

原SO仅在启用回充并初始化Engine后加载；关闭回充不加载它。shim按自身路径固定查找同目录原SO，前后核验完整SHA，使用RTLD_NOW|RTLD_LOCAL，不向全局导出原OpenCV符号，缺失/错版本/缺依赖明确失败且没有旧算法回退。shim显式链接真实共享zlib，并只把真实libz提升为GLOBAL，修复原SO未声明gzeof依赖；原SO仍需要真实ROS Humble、common_interface、TBB等运行依赖，不能描述为无ROS依赖。

每个进程只允许一个回充Engine，调用串行。结果直接复制原post.x/y/yaw和ok/pose_ok，不舍入、不重算；overlay复制到Output后由原freeResult释放。现有V2 frameStamp写入的是DAG帧路由时间，不属于算法三浮点一致性。销毁先调用原destroy(false)并delete API；原SDK及libz的加载handle保留至进程结束，避免上游全局日志/退出处理仍引用已卸载代码。二次initialize先销毁上次API，再按相同真实初始化序列重新加载配置。

Linux/X5用R33_WITH_RECHARGE=ON构建；普通SDK的PERCEPTION_ENABLE_LOCAL_CAMERA_SHM必须OFF。本后端不包含本地相机共享内存代码。BUILD_TESTING保留公开C ABI的DAG、帧身份及结果归属测试，并按真实三帧门控处理；新的V0 API/Engine契约测试只检查生命周期/输入验证/结果传递，不将空白帧等同于精度验收。最终一致性必须以X5同一数据和原SDK全量逐FP32位对比结果为准。
