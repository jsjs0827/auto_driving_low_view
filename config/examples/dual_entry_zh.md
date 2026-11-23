# 同一 DAG 的两种部署入口

`dual_entry_offline.json` 中的 `features`、`tasks`、`dag` 只保存一份。`deployment.mode` 选择 `standalone` 或 `v0_host`；未配置 `deployment` 的 schema 3–5 配置继续使用已有输入和启动行为，也无需增加 `runtime`。

- `standalone` 使用既有 `input.mode`，例如离线文件、录制回放、相机或 ROS 输入。
- `v0_host` 使用原 V0 宿主加载的 `libpercep_main.so` 兼容插件，接收上游共享内存帧、命令和结果通道。配置器不会启动相机、使用本地 `camera_shm`，也不会订阅 ROS 图像。

两种模式都由同一个配置器生成 `effective_full.json`，使用相同的 SDK、算法配置和 DAG。板端 SDK 初值设为 `mode="stopped"`（命令 21），等待上游有效命令；这项初值不改变图中的算法开关。

先将 `V2_OFFLINE_LEFT`、`V2_OFFLINE_RIGHT`、`V2_SHARED_INTRINSICS` 和 `SDK_ROOT` 指向实际文件/SDK。离线图像需为同次采集的配对输入，标定必须对应这些图像。也可以直接编辑 `dual_entry_offline_input.json` 填相对主配置文件的路径。

```bash
bash tools/run_pipeline.sh --config "$PWD/config/examples/dual_entry_offline.json" \
  --deployment standalone --prepare-only --run-dir /tmp/v2_offline_plan_001
bash tools/run_pipeline.sh --config "$PWD/config/examples/dual_entry_offline.json" \
  --deployment v0_host --prepare-only --run-dir /tmp/v2_host_plan_001
sha256sum /tmp/v2_offline_plan_001/algorithm_contract.json \
  /tmp/v2_host_plan_001/algorithm_contract.json
```

`algorithm_contract.json` 保留有效算法图、模型路径、阈值、融合配置、Stereo 参数及配置中的标定内容；排除部署入口、命令初值、发布/记录节点和运行目录。SHA256 在 `algorithm_contract.sha256`、`resolved.json` 及板端 `v0_host.json` 中。相同 hash 只说明配置一致，不证明模型文件字节、输入帧、运行时内参或实机算法结果一致。

`v0_host` 的准备结果还包含 `v0_host.json`，其中 `sdk_config` 指向有效 SDK JSON，`stereo` 提供模型与输入形状，`algorithm_contract_sha256` 标记公共算法配置。`resolved.json` 明示 `deployment_mode`、`host_config`、`plugin_library` 和实际 argv。

## 原宿主挂接

在 V0 原宿主创建 `dayeFunction` 时，给它的构造配置设置：

```json
{
  "v2_host_config": "/tmp/v2_host_plan_001/v0_host.json"
}
```

`deployment.v0_host` 可选 `dock_pose_topic`、`grid_header_clock`、`parent_frame`、`factory_reset_directory`，这些输出适配字段原样写入 `v0_host.json`，不改变公共算法 contract。`grid_header_clock` 只接受 `legacy_steady_ms` 或 `capture_ns`；未提供时插件沿用兼容时钟策略。显式提供的 `dock_pose_topic` 必须是非空字符串；本示例不编造默认话题，真实启动时必须由已确认的接口在大配置或原宿主 FunctionJSON 中提供，缺失时插件会拒绝启动。`parent_frame` 必须为非空字符串；`factory_reset_directory` 须为字符串，显式空字符串表示禁用命令 9002 的文件清理，目录不会由配置器创建。

配置器不假设原宿主的命令行选项。必须按现场宿主接口将该值传入构造配置，并填入实际 `deployment.v0_host.launch_argv`。空数组允许 `--prepare-only`；实际运行会明确报错。`plugin_library` 默认路径是相对主配置的 `lib/libpercep_main.so`，示例显式改为 `../../lib/libpercep_main.so`；该路径必须对应实际兼容 ABI 构建的插件。

例如，如果现场原宿主确实支持 `--function-config FILE`，可将宿主配置单独存为 `/userdata/v2/function.json`，内容包含上面的 `v2_host_config`，再配置：

```json
{
  "deployment": {
    "mode": "v0_host",
    "v0_host": {
      "plugin_library": "/userdata/v2/lib/libpercep_main.so",
      "launch_argv": ["/userdata/actual-host", "--function-config", "/userdata/v2/function.json"]
    }
  }
}
```

这里的宿主名和 `--function-config` 是接口示意，必须替换成已确认的现场命令。配置器不会调用 shell；每项按原样传给 `execv`。仅当一个完整 argv 项等于 `{host_config}`、`{plugin_library}` 或 `{run_dir}` 时才替换为生成路径；不进行嵌入替换，不自动写入未知宿主的配置。不要把启动命令拼成一条 shell 字符串。

移除 `--prepare-only` 才会调用真实宿主。准备、mock ABI 或离线配置测试通过，均不表示真实上游共享内存、命令或输出通道联调完成。
