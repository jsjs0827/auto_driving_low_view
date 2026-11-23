# r25 长测验收脚本修复

本次 `Missing original DAG execution evidence: long_exact_time` 来自测试包装器的验收错误。长测给 worker 的 `--frames 100000` 是安全上限，实际按 `--duration-seconds` 停止。旧脚本仍要求 `dag_runs >= 100000`，因此在按时长正常停止、实际帧数较少时误报。

新逻辑按实际 `samples` 检查 DAG 覆盖；固定帧数测试仍必须完整达到请求帧数。长测仍检查 duration_completed、请求参数、CSV 行数、DAG 运行序号、Task/Feature、帧号/时间戳、按需视差与 SDK 清理，不能用本修复绕过缺失证据或实际失败。

此次只修改 Python 测试验收，无需重新交叉编译已安装的 callback-fix-v1 SDK。检测、分割、C++ worker、原 DAG、相机回调修复及模型全部保持。

## X5 安装脚本补丁

将 `perception-x5-r25-longcheck-fix-v1.tar.gz` 及同名 `.sha256` 复制到 X5 `/userdata`，测试程序退出后执行：

```bash
cd /userdata
sha256sum -c perception-x5-r25-longcheck-fix-v1.tar.gz.sha256
tar -xzf perception-x5-r25-longcheck-fix-v1.tar.gz
python3 perception-x5-r25-longcheck-fix-v1/install_r25_tools.py \
  --sdk-root /userdata/ai_infer_r12_v25/perception-sdk-x5-r12-v25-callback-fix-v1

SDK=/userdata/ai_infer_r12_v25/perception-sdk-x5-r12-v25-callback-fix-v1
python3 "$SDK/tools/integrity.py" "$SDK"
```

安装器先验证 SDK 身份、完整现有清单及已知旧脚本散列，再备份和替换三份 Python 文件、一份说明，并更新对应清单和工具补丁标记。失败会回滚；二进制、原模型、相机 SDK/config 和结果目录不修改。遇到散列不匹配会停止并打印文件名，不应自行重建清单跳过错误。所有补丁文本与清单均为 UTF-8、无 BOM、LF。

## 只读复核已经结束的长测

默认结果父目录是 `/userdata/r12_v25_res`。下面命令自动选择其中最近有 `long_exact_time/complete.json` 的结果，并在 JSON 中打印实际选中的 result 路径：

```bash
python3 "$SDK/tools/recheck_long_stage.py" \
  --latest-under /userdata/r12_v25_res
```

若你用了其他结果父目录，替换 `--latest-under` 的路径。也可指定某次准确结果根目录：

```bash
python3 "$SDK/tools/recheck_long_stage.py" --result /userdata/r12_v25_res/你的本次结果目录
```

此命令不启动相机或推理、不写回旧结果。只有完整通过时才打印 `stage_validated: true`，并给出实际 samples、dag_runs、帧数上限、时长、图证据、视差与清理校验。worker 提前退出、CSV/DAG 不完整、帧号错误、视差失败或真实清理错误仍拒绝。

通过意味着该次长测的既有证据通过修正后的验收。它不代表此前中断后尚未执行的相机重开测试也已完成，也不证明不存在资源泄漏；旧 run.json 的失败状态保持，复核 JSON 会单独展示该状态。

## 接着完成重开测试

如果旧长测复核通过，可在原完整命令中设置 `--long-seconds 0 --reopen-count 10`，避免重复已复核的长测。仍会重新做原 A/B 和数值验收，然后完成带模型/纯相机、restore/hold 的相机重开对照。例如沿用默认原模型路径：

```bash
bash "$SDK/tools/run_x5.sh" \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --sdk /app/data/metoak/lib/libMoGeneralSDK.so \
  --camera-config /app/data/metoak/demo/config/x5_camera.cfg \
  --stream gdc --format i420 \
  --frames 120 --warmup 20 --rounds 2 \
  --long-seconds 0 --reopen-count 10 \
  --reopen-frames 20 --long-demand-every 10 \
  --out /userdata/r12_v25_res
```

实际模型路径或原先参数有差异时继续使用原值。此命令生成新的结果子目录；请一起回传旧长测、新重开结果各自的 `metrics_only.tar.gz` 和 `.sha256`，并附上只读复核输出。不同目录的测试不自动合并为一场已完成测试。

如果想重新做一次完整长测和重开，安装补丁后按原命令运行即可；保留原 `--long-seconds 600 --reopen-count 10`。

## 后续源码

完整源码同步为 `perception-x5-dag-exact-r12-v25-callback-fix-v2`。只有需要重新制作完整 SDK 时才使用源码交叉编译；已安装 callback-fix-v1 的用户按上述脚本补丁操作即可。新源码中的 C++ 数值、框架和相机适配器与 callback-fix-v1 按字节一致。
