# V0 感知基线

此分支保留原 ROS 感知模块，使用相机 SDK 的视差，作为检测、分割、回充和三张俯视图的对照版本。

- [原始工程](project/point-cloud-processing-main/)来自本机 `/home/st/download/gitcode/1104/point-cloud-processing-main/`。
- [回充 SDK](project/point-cloud-processing-main/recharge_mark/)来自此前确认的 `charing-main.zip` 基准版。原工程的回充子模块目录为空，分支已补入对照试验使用的 SDK。
- [测试报告](test-reports/README.md)包括历史检测分割、回充及俯视图对照。每个报告记录了实际比较的版本。

编译时把 `point-cloud-processing-main` 放回原 ROS 2 Humble 宿主工作区 `src/ai_node/`，使用该宿主配套的 `common_interface`、第三方库和 X5 工具链。
