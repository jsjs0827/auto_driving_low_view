# V1 感知模块

此分支采用 2026-11-23 的 X5 宿主集成版。工程保留原有地图、回充和待机命令，同时可通过配置选用 V2 SDK 执行 Q8 地图和 GDC 回充。

| 内容 | 路径 |
| --- | --- |
| 完整 V1 工程、模型和配套 SDK | [project/point-cloud-processing-main](project/point-cloud-processing-main/) |
| 生产 ROS 消息及相机共享内存接口 | [dependencies/common_interface-main](dependencies/common_interface-main/) |
| 交叉编译得到的 X5 动态库 | [runtime-x5/lib/ai_node](runtime-x5/lib/ai_node/) |
| 配置切换与运行说明 | [V1 接入 V2 SDK](docs/V1-V2-SDK-Q8-回充接入.md) |
| 本机 Docker 工作区与打包命令 | [2026-11-20 打包流程](docs/X5-打包流程-2026-11-20.md) |
| 检测分割、暗场和 X5 联调报告 | [测试报告](test-reports/README.md) |

周五的打包文档记录 11 月 20 日的工作区和编译产物。这个分支的源代码和板端验证对应 11 月 23 日更新，重新构建时使用本分支的源码和接口。
