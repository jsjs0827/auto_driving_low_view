# WSL交叉编译与交付检查

- 构建主机：WSL2 Ubuntu 22.04 x86_64。
- 目标：Linux AArch64，使用现有X5 sysroot和aarch64-linux-gnu编译器。
- 业务SDK、DAG配置/运行程序、六组测试程序：编译通过。
- Ubuntu汇总、ROS桥接、RViz结果回放程序：编译通过。
- X5 ELF架构、动态库依赖闭包、版本化依赖符号：检查通过。
- 主机配置、已知合成数据的AP50/mIoU/EPE、重复汇总、PNG/Q8/PLY保存：检查通过。
- 本机ROS DDS→TCP确认式回放：两种视差输入契约及原始时间戳保持检查通过。
- 六组保存结果的RViz配置和rosbag录制：合成数据检查通过，48个结果话题、48条消息。

未连接X5，未执行真实相机/模型推理，没有板端精度、耗时、BPU、CPU、RSS统计或实际RViz画面。合成数据仅验证程序行为。

构建日志见 BUILD_LOGS，逐个ARM ELF与依赖审计见 BUILD_REPORT.json。
