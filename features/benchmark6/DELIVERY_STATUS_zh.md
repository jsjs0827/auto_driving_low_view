# 本版交付状态

本目录新增六组测试的配置生成、独立性能/质量运行、绝对精度与参考一致性汇总、图像和点云派生、RViz 回放及 rosbag 保存。

`runtime` 复用了此前 `perception-ab-benchmark-v9` 的相机、离线、ROS桥接、SDK推理与采样代码，并已适配为两种视差方法、关闭建图、保存逐帧预测、确认式ROS供帧与结果回放。它是经过修改的源码，不能视为原封不动的 v9 交付。

根据后续授权，本版已在本机 WSL2 Ubuntu 22.04 完成 X5 业务程序、X5六组测试程序和Ubuntu配套程序的编译；主机配置、合成精度指标、ROS确认式传输、PNG/Q8/PLY和六组rosbag保存检查已通过。没有连接设备，尚无真实相机/模型精度、板端耗时/资源数据或实际 RViz 截图。项目中继承的 `DAG_VALIDATION_zh.md`、`validation/evidence`、`runtime/VALIDATION.md`、依赖审计及 provenance 等内容仅是对应旧版本的历史记录，不是本版验证结果。

远程 Ubuntu 执行入口和结果路径见项目根目录 `BENCHMARK6_zh.md`。运行后的 `summary.json`、`run_status.tsv`、`visualization_status.txt` 和日志才用于判断本次测试与保存是否完成。
