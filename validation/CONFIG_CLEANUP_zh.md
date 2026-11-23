# v10 配置整理验证

本次调整配置目录、Python/Shell 入口和说明文档。C++、CMake、package.xml 及相机新 Stereo 默认算法参数与原 v10 源码包逐字节一致，没有改变推理、点云融合或性能统计实现。

- 默认入口为 `config/pipeline.dag.json`；不传配置参数即可启动该文件。
- 相机、ROS、离线、录制对比模板集中在 `config/examples/`。旧 YAML 和旧尺寸比较放入 `config/legacy/`；RViz 放入 `config/rviz/`。
- 旧路径在启动脚本和源码/运行包的 Python 命令行入口中提示迁移位置。已有用户文件优先；不会按相同文件名猜测其他目录。旧相机新算法路径始终指向独立的新算法模板，避免主配置切到原方法后改变旧命令含义。
- `validation/test_pipeline_config.py`：35 项通过，包含全部常用模板校验、无位置参数的实际配置生成、旧路径迁移和用户文件优先检查。路径 fixture 不执行模型。
- `validation/compare_input_test.py`：17 项通过；`validation/test_compare_report.py`：12 项通过。共 64 项 Python 测试。
- WSL Ubuntu 22.04 系统 Python 再次通过 35 项配置测试；所有 Shell 脚本通过 `bash -n`。
- 使用现有 native 构建的真实 CMake 安装规则安装到全新验证目录，全部 19 个配置/说明文件与源码字节一致。顶层只含 `pipeline.dag.json` 和 `README_zh.md`，没有残留旧的平铺模板。
- 原 v10 源码压缩包及 SHA256 保留；整理版重新生成源码清单并校验 UTF-8、无 BOM、LF 和压缩包内字节。

本次没有重新测量 X5 相机、新模型效果或性能；此前验证范围见 [v10 验证记录](V10_VALIDATION_zh.md)。交叉编译流程保持原样。

解压整理版源码时使用一个新目录，避免旧源码目录中的平铺配置残留。已经修改过的主配置应自行保留后再切换模板；运行目录里的自动配置是执行快照，日常继续修改主 JSON。
