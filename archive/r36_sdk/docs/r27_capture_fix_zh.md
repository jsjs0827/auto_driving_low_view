# r27 capture_full 超时：恢复 full-sync 并做短采集对照

这次错误发生在 `capture_full`，尚未进入检测 / 分割性能阶段。`last SDK code=-1` 只说明在应用超时前未取到 RGBD 帧，不足以证明是哪条 SDK 内部链路失败。

源码审查确认一处回归：r27 生成了旧 `full` 副本，没有使用此前 `full-sync` 的两处同步队列修复。历史 r20 板测已有旧 full 同步失配 / 超时、full-sync 完成的对照。RGBD 也经过 `_getVseFrame → _v2_syncVseTs → _getDispFrame → RGBD输出队列`，因此并非只有 GDC 受此问题影响。**这与本次症状相符，但本次根因仍须新日志和受控对照确认。**

修复后的脚本默认生成 `full-sync`：保留去除1:3筛帧及帧号映射，再加入两处“遇到未来时间戳时保留队列记录”的已审计修改。板端原SDK、相机配置和模型不改写。原 C++、检测 letterbox、分割数学和单帧调度完全不变；本补丁不需要交叉编译。

## 1. 在 X5 安装工具补丁

把补丁包及其 `.sha256` 复制到 `/userdata/ai_infer_r12_v27`，执行：

```bash
cd /userdata/ai_infer_r12_v27
sha256sum -c perception-x5-r27-capture-sync-fix-v1.tar.gz.sha256
tar -xzf perception-x5-r27-capture-sync-fix-v1.tar.gz
python3 perception-x5-r27-capture-sync-fix-v1/install_r27_tools.py \
  --sdk-root /userdata/ai_infer_r12_v27/perception-sdk-x5-r12-v27-fullrate-parallel
```

必须指定已解压、含 `SDK_RELEASE.json`、`SHA256SUMS` 和 AArch64 `bin/dag_flow_worker` 的 r27 SDK根目录。安装前验证原 SDK完整清单及数值基线，备份本次改动，安装后重新校验。不要把源码目录或压缩文件路径传给 `--sdk-root`，也不要手动编辑被清单保护的文件。

## 2. 先做短采集定位

确认旧相机程序已退出，然后执行：

```bash
cd /userdata/ai_infer_r12_v27/perception-sdk-x5-r12-v27-fullrate-parallel
python3 tools/diagnose_capture.py \
  --out /userdata/r12_v27_capture_diag \
  --sdk /app/data/metoak/lib/libMoGeneralSDK.so \
  --camera-config /app/data/metoak/demo/config/x5_camera.cfg
```

相机配置需填写原来能出图的实际路径。必要时追加 `--camera-runtime-dir /实际运行库目录`。

诊断使用现有 AArch64 worker，不运行应用检测、分割模型。每项独立打开 / 关闭相机，5帧预热后采集10帧：

| 项目 | SDK副本 | 模式 / 回调 |
| --- | --- | --- |
| original_default | 原SDK | 保持SDK默认、不注册回调 |
| original_mode2 | 原SDK | 显式持续视差2、注册回调 |
| full_default | 旧full | 保持SDK默认、不注册回调 |
| full_mode2 | 旧full | 显式持续视差2、注册回调 |
| full_sync_mode2 | full-sync | 显式持续视差2、注册回调 |

这里的命令值 `disparity-mode=0` 表示不调用模式设置接口，**不表示关闭SDK视差**。模式2仍有SDK内部视差BPU负载；这些是采集诊断，不能当作两算法FPS结果。

普通取帧失败会保留证据，只有确认相机关闭 / SDK卸载正常才继续下一项。清理失败或进程超时被终止时停止，避免后面的测试使用不明相机状态。没有通过重试、丢失败帧或更换输入掩盖错误。

输出位于 `/userdata/r12_v27_capture_diag/capture-diag-日期时间-进程号/`。传回整个目录或其中 `metrics_only.tar.gz` 与校验文件，主要看摘要、各项日志、模式、清理和CMA变化。

如果原SDK模式2成功、旧full模式2失败、full-sync模式2成功，将支持同步队列修复的解释；若原SDK模式2也失败，还需先排查模式 / 运行环境 / 重开资源，不能把错误都归于筛帧补丁。

## 3. 短测通过后重新运行完整 r27

```bash
cd /userdata/ai_infer_r12_v27/perception-sdk-x5-r12-v27-fullrate-parallel
bash tools/run_parallel_x5.sh \
  --out /userdata/r12_v27_res \
  --sdk /app/data/metoak/lib/libMoGeneralSDK.so \
  --camera-config /app/data/metoak/demo/config/x5_camera.cfg \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --sdk-patch-policy full-sync \
  --frames 300 --warmup 30 --capture-frames 10 \
  --verify-repeats 2 --rounds 2 --camera-retry-ms 1
```

默认已经是full-sync，上面显式填写便于复查；旧full只作为诊断对照保留。每次使用新结果目录，旧失败结果不改写、不补记为成功。报告的 `sdk_patch.json`、`run.json.sdk_patch_policy` 和 `sdk_used.full.patch_policy` 记录实际修改策略；`full`组名仍表示全速实验组，不代表实际使用旧full补丁。

Windows生成的脚本和清单均UTF-8无BOM、LF；补丁归档后校验最终字节。安装与测试均尚需在你的X5执行，本机检查不能替代板端采集成功或FPS数据。
