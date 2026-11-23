mode1 X5 压测脚本说明

板子：ADB [设备序列号已省略]
本机工作目录：/home/st/ini_code/pcl_Ini/work/mode1-stress-x5-20261118
板子专用目录：/userdata/dark_scene_mode1_stress_20261118
二进制：原交付 mode1_live_demo 与 libdark_scene.so，未修改算法。

本次执行顺序：
  1. boardctl.py deploy：新建独立目录、核验原版二进制 SHA256。
  2. boardctl.py start camera-30min 1800：真实相机 30 分钟。
  3. inspect_live.py：只读查询当前结果及资源，冻结结果前缀后核对已落盘输入。
  4. boardctl.py pull camera-30min：只允许已完成测试，逐文件 SHA256 核验。
  5. build_probe.py：使用本机 AArch64 编译器构建测试驱动，链接原 libdark_scene.so。
  6. launch_suite.py：长跑完成后才允许启动；高频 AE、突发、异常、公开
     CameraData 字节接口测试；10 次相机启停；SIGINT/SIGTERM 各一次。
  7. 拉取 suite 目录，核验全部文件 SHA256，读取结束时硬件与进程状态。
  8. analyze.py：只读测试证据，重新计算规则、资源、耗时与报告。

监控：board_supervisor.py 只管理自己创建的子进程。
  工程温度停止线 90°C，MemAvailable 停止线 100 MiB。
  记录 CPU、RSS/PSS、FD、线程、温度、系统内存、DMA 缓冲与 BPU 原始统计。
  CPU 100% 代表一个核心；8 核容量比例 = 单核口径 /8。
  标准输出完整排空并计算 SHA256，仅保存前 256 KiB 和末 1 MiB。
  分类与 AE 原始日志完整保存；不是仅依赖标准输出计算。

注意：
  板上日历时钟没有校准，耗时均使用单调时钟；未修改时间或系统配置。
  仅建议开灯/关灯，不控制硬件灯；ROS 使用隔离域 77/78。
  SDK 输入是真实 Org-left AE；共享内存相关只测公开 CameraData 字节解码接口，
  未启动真实业务共享内存生产者。本工作不包含本地专用共享内存实验模块。
  同一板子不要同时运行其他相机任务；启动脚本发现占用会拒绝运行。
  每次复测必须创建新的实验目录与测试名，不覆盖已有证据。
  默认脚本中的固定根目录针对本次实验，已有目录会明确拒绝覆盖。
  如复测，需要把新工作目录中的 boardctl.py、board_suite.py 根目录统一改为新名称；
  保持所有者标记和安全阈值，随后再部署。不要替换生产 SDK 或停止其他服务。

本机报告重新计算：
  python3 -B /home/st/ini_code/pcl_Ini/work/mode1-stress-x5-20261118/tools/analyze.py
