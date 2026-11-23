# r25 按需视差回调收尾修复 v1

这次报错发生在首个 `demand_n10` 阶段，前面的同帧一致性与固定输入测试已经完成。现有报错只显示 SDK 生命周期返回 `-2`，尚不能仅凭这一行确认具体 API，也不能据此判断推理结果错误或 CMA 泄漏。

已确认一个适配缺陷：原适配器收尾调用 `moGetStereoVisionStatusCallback(handle, nullptr)`。SDK 头文件说 NULL 可取消回调，但检查本机 1.4.4.0 参考库以及此前 X5 下载的 full 副本，发现导出函数相同：回调参数为 NULL 时直接返回 `-2`。前面的 continuous/demand_idle 不注册记录回调，首次 demand_n10 才触发这条收尾路径。这与本次失败位置吻合；最终归因需要核对本次 `camera_lifecycle.jsonl`。

## 本轮修改

1. 关闭前停止回调记录，用有效的进程生命周期空操作函数替换记录回调；随后仍按指定策略恢复视差模式、关闭相机，关闭成功才卸载 SDK。
2. 记录 `callback_register` 和 `callback_replace_noop` 的实际返回码。任何真正的替换、恢复、关闭或卸载失败仍使阶段失败，不把 `-2` 无条件当成功。
3. 首个清理错误保留 API 名称、返回码、相机周期，并写入 `lifecycle_failure.json`。包装器的 `failure_details.json` 保留全部可读取的生命周期事件和清理错误。
4. 增加相机独立的按需视差与重开短检查，仍通过原 DAG 的 input/disparity Task 执行。

精确缩放、实际 BPU 输入写入、检测 letterbox、分割前后处理、原模型、原 DAG／Task／Feature 和单帧无双缓冲保持。没有修改厂商 SO 的回调机器码，也没有覆盖原 SDK 或相机配置。原有 `full-sync` 帧交付实验继续只在结果目录的独立副本上执行。

这是 C++ 修复，需要重新交叉编译。只复制 Python 脚本不能修复旧可执行文件中的 NULL 调用。

## Ubuntu 宿主机编译

把新的源码 tar.gz 和同名 .sha256 放在 `/home/st/ini_code`，在宿主机终端执行，继续复用已运行的 C++ 编译容器：

```bash
cd /home/st/ini_code
sha256sum -c perception-x5-dag-exact-r12-v25-callback-fix-v2.tar.gz.sha256
tar -xzf perception-x5-dag-exact-r12-v25-callback-fix-v2.tar.gz
cd perception-x5-dag-exact-r12-v25-callback-fix-v2
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out /home/st/ini_code/r25_callback_release_v2
```

实际容器名称若为 `x5-build`，只替换 `--container` 的值。不要把镜像名称填在这里。无需进入 Docker，无需下载镜像或重新转模型，不要 `source` 编译脚本。

生成的两个 SDK 文件：

```text
/home/st/ini_code/r25_callback_release_v2/perception-sdk-x5-r12-v25-callback-fix-v2.tar.gz
/home/st/ini_code/r25_callback_release_v2/perception-sdk-x5-r12-v25-callback-fix-v2.tar.gz.sha256
```

源码包和 SDK 包有不同名称。X5 要运行交叉编译生成的 SDK 包。

## X5 解压

将上面两个 SDK 文件复制到 `/userdata/ai_infer_r12_v25_callback`。停止此前相机测试程序后执行：

```bash
mkdir -p /userdata/ai_infer_r12_v25_callback
cd /userdata/ai_infer_r12_v25_callback
sha256sum -c perception-sdk-x5-r12-v25-callback-fix-v2.tar.gz.sha256
tar -xzf perception-sdk-x5-r12-v25-callback-fix-v2.tar.gz
cd perception-sdk-x5-r12-v25-callback-fix-v2
python3 tools/integrity.py .
```

所有文本为 UTF-8、无 BOM、LF，清单按最终字节生成。解压后直接校验，不做批量换行转换，也不重新生成清单来跳过错误。

## 先做短检查

以下检查两个关闭策略，各打开三次相机，每次测量 20 帧，每 10 帧请求一次视差；正常完成后自行退出。模型参数沿用原路径，但该阶段不加载检测／分割模型，不生成推理性能结论。

```bash
bash tools/run_x5.sh \
  --callback-smoke-only \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --sdk /app/data/metoak/lib/libMoGeneralSDK.so \
  --camera-config /app/data/metoak/demo/config/x5_camera.cfg \
  --stream gdc --format i420 \
  --frames 30 --warmup 5 \
  --long-seconds 0 --reopen-count 2 --reopen-frames 20 \
  --long-demand-every 10 --close-policies restore,hold \
  --out /userdata/r12_v25_callback_res
```

通过时打印 `Callback smoke passed (camera lifecycle only)`。结果中的 `callback_smoke_summary.json` 必须为 completed=true；每个周期的 `callback_register`、`callback_replace_noop`、`moCloseCamera`、`dlclose` 都应为 0。restore 还要求恢复模式为 0，顺序为替换回调→恢复模式→关闭→卸载。hold 直接替换回调→关闭→卸载。

短检查不替代加载模型后的资源与稳定性测试。它的 FPS 也不是检测＋分割 FPS。

## 再运行完整 A/B

短检查通过后，在同一 SDK 目录执行：

```bash
bash tools/run_x5.sh \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --sdk /app/data/metoak/lib/libMoGeneralSDK.so \
  --camera-config /app/data/metoak/demo/config/x5_camera.cfg \
  --stream gdc --format i420 \
  --frames 120 --warmup 20 --rounds 2 \
  --long-seconds 0 --reopen-count 0 \
  --out /userdata/r12_v25_callback_res
```

每次生成新结果子目录，不覆盖原失败结果。重新完成输入零字节差、mask 零像素差、检测逐位一致验收后，测试原有 fixed/continuous/demand_idle/demand_n10/demand_n30 矩阵。要做长测和重开，将最后的时长/重开参数改为：

```bash
  --long-seconds 600 --reopen-count 10 \
  --reopen-frames 20 --long-demand-every 10
```

这是原完整测试入口，按有限时长与帧数运行。回传结果目录的 `metrics_only.tar.gz` 和 `.sha256`，其中包含生命周期日志、按需视差、性能、CPU、RSS 与 CMA 元数据。

## 核对原失败结果

```bash
R=/userdata/r12_v25_res/dag-exact-ab-20250903-221458-3834425
cat "$R/control_round1_demand_n10_r21/camera_lifecycle.jsonl"
tail -n 40 "$R/control_round1_demand_n10_r21.log"
tail -n 2 "$R/control_round1_demand_n10_r21/frames.csv"
```

若出现 `callback_unregister` 的 code=-2、随后 moCloseCamera/dlclose 为 0，即确认此次失败来自 NULL 取消回调兼容问题。若 -2 来自恢复模式或关闭相机，仍需按该 API 的日志继续定位，本修复不会忽略它。

可额外只读检查板上 SDK 的已知静态指令：

```bash
python3 tools/check_callback_sdk.py --sdk /app/data/metoak/lib/libMoGeneralSDK.so
```

`known_null_returns_minus_2` 表示识别到上述函数；`unknown` 只表示不匹配这份已检查的指令，不代表另一版本支持 NULL。检查器不加载 SDK、不打开相机、不改文件，不能单凭静态指令确认本次实际执行的路径。

## 本机验证范围

主机测试用真实适配器源码＋模拟厂商 API/加载器覆盖 NULL→-2、有效回调替换、重开、restore/hold、视差 public ID/结果所有权和失败时保留 SDK。另运行原数值回归、框架契约和报告校验；验证记录见 validation。

主机检查不能证明 X5 实际相机关闭资源释放、视差精度或性能。首个 demand_n10 阶段失败后，后续受控矩阵与长测尚未完成；原失败结果仍应保留失败状态。
