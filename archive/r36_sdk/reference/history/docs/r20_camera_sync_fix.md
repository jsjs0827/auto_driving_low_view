> 历史 r20 相机修复记录。r21 已集成此修复，不要给 r21 安装 r20 热修复工具包；编译和运行以本项目 README_zh.md 为准。

# r20 相机同步修复（v2 安装器）

v2 修复了 v1 安装器对宿主机交叉编译输出的误拒绝：宿主机脚本复制源码到 `/tmp/r12-v20-*/source`，旧打包器把临时目录名写成 `source_project: source`。安装器现在接受这个已知别名，仍要求 version=20、architecture=AArch64，且完整清单和原工具哈希必须通过。其他项目或架构继续拒绝，并显示实际字段。无需修改 SDK_RELEASE.json 或重生成哈希。未来打包器使用固定项目标识，另行记录临时目录名。

用户日志显示 `CAMERA_ACQUISITION_FAILED: camera acquisition timed out; last SDK code=-1`，此前持续打印 `Frame id missing in sync map 84,cur 83`、`85,cur 84` 等。该阶段是原始 CPU/BPU **baseline**，未使用精确 CPU 优化或 VSE/fused 路线；同帧验收与三个固定输入性能阶段已经通过。

## 已确认与仍需验证的部分

已确认直接失败是 SDK 无法向应用交付有效帧。没有 BPU 推理失败或内存分配失败的错误证据。开始前 CMA 为 58788 KiB，退出后 52120 KiB；这些整板统计不能证明本次是 CMA 不足或泄漏。

SDK 1.4.4.0 程序集中的 GDC/VSE 同步查找，遇到 `cached_id > requested_id` 时打印上述信息。第一项 84 是同步表条目 ID，`cur 83` 是当前待匹配的图像 ID。原分支返回 -4 时还执行 `read += available`，把尚可供下一帧使用的未来条目一起消费；下一个图像到来时，可能重复发生相同错位。这与当前持续相差一帧的日志吻合。

本次在独立副本上新增两个分支改动：

| 函数 | 指令地址 | 原目标 | 新目标 | 效果 |
| --- | --- | --- | --- | --- |
| `_v2_sync_gdcTs` | `0x165cf0` | `0x165c6c` | `0x165c70` | 跳过未来-ID失配情况下的 `read += available` |
| `_v2_syncVseTs` | `0x165ae8` | `0x165a64` | `0x165a68` | 同上 |

两条指令由 `0x17ffffdf` 改为 `0x17ffffe0`。仍返回 -4 并拒绝当前失配图像；ID 相等时才成功，时间戳、曝光和双目同步检查保留。没有减一 FrameId、把邻帧拼成一帧、重写像素或放宽相等条件。缓存容量、内存布局和帧缓冲归属不变。原来成功匹配及缓存全为旧帧的分支不变。

`full-sync` 包含原来提帧的 13 条指令及这 2 条修复；原 SDK 文件不覆盖。来源仍限定于之前核实的两个 SDK SHA256，并校验完整函数和目标指令。静态验证与消费逻辑测试通过；**实板是否解除当前超时、视差是否正确、实际 FPS 与长期资源释放尚须测试**。

此前 full 提帧成功测试主要使用 GDC 输出精简配置与显式按需视差；r20 的原配置＋保留视差状态组合没有充分验证。因此普通 `run_ab.py` 默认改为 `original`，保留原相机状态；`full-sync` 需显式选用。`--disparity-mode 0` 的含义仍是保留 SDK 状态，不会偷偷关闭视差。

## 已编译的 r20 SDK：不用重新交叉编译

将 `perception-x5-r20-camera-sync-fix-v2.tar.gz` 和 `.sha256` 复制到 X5 `/userdata`。仅通过安装器更新工具，避免手动覆盖后破坏 SDK 校验清单：

```bash
cd /userdata
sha256sum -c perception-x5-r20-camera-sync-fix-v2.tar.gz.sha256
tar -xzf perception-x5-r20-camera-sync-fix-v2.tar.gz

python3 perception-x5-r20-camera-sync-fix-v2/install_r20_tools.py \
  --sdk-root /userdata/ai_infer_r12_v20/perception-sdk-x5-r12-v20-letterbox-seg

cd /userdata/ai_infer_r12_v20/perception-sdk-x5-r12-v20-letterbox-seg
python3 tools/integrity.py .
```

安装器先验证全部原 SHA256SUMS 与待替换工具的已知原始哈希，再校验修复包载荷；备份工具、SDK_RELEASE.json 与旧清单后，只更新这些指定文件及对应哈希。模型、C++ 程序、输入流程、原相机 SDK 和配置不改。未知内容变化拒绝安装，没有 force 模式；安装期间失败会回滚。备份路径由安装器打印。

## 先运行有结束帧数的同步对照

```bash
python3 tools/camera_sync_probe.py \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --stream gdc --format i420 --disparity-mode 0 \
  --frames 60 --warmup 20 \
  --out /userdata/r12_v20_res
```

探针对照：原 SDK 仅采集、原 SDK 在线基线、旧 full 在线基线、full-sync 在线基线、full-sync 在线优化、full-sync 在线基线复测。保留相同原相机配置与请求视差模式，原模型和 letterbox/分割输入流程不改；每个案例都是独立进程。失败案例保留日志和退出码，然后继续其它对照；每个案例都有超时限制，不会一直采集。capture 最多保存 10 帧，推理案例关闭大图保存。

结果在 `/userdata/r12_v20_res/camera-sync-probe-*`，包含 `probe.json`、各案例日志和 `report.md`。full-sync 三个在线案例都完成后，再运行正式验收：

```bash
bash tools/run_x5.sh \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --sdk-policy full-sync \
  --stream gdc --format i420 --disparity-mode 0 \
  --frames 120 --warmup 20 \
  --out /userdata/r12_v20_res
```

若希望先测未经补丁的原流程，把 `--sdk-policy full-sync` 改为 `--sdk-policy original`。若 full-sync 仍超时，保留探针目录分析；不通过延长超时、切换视差或丢弃同步检查来把失败标成通过。

新工具在阶段失败时额外生成 `failure_details.json` 和 `partial_report.md`，保留已经完成的固定输入结果，并把真正的子进程错误显示在命令行。检测和分割一致性验收门槛不变。

## Ubuntu 源码重建

新的完整源码包已包含上述工具；继续使用 README_zh.md 中相同的 `sudo bash tools/build_in_container.sh` 命令。不需要重新编译模型，也不需要重新拉镜像。所有文本与内外校验清单均为 UTF-8、无 BOM、LF。
