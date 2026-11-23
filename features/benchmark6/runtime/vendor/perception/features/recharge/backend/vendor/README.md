# recharge_mark

回充标志识别独立库（`librecharge_mark.so`）。当前 **`apriltag` 分支内部为 ArUco 4×4**，对外是宿主 `Aiper_Recharge::RechargeMarkHost`（Init/Start/Tick/Stop/Destroy 生命周期）与底层 `RechargeMarkApi`。

## 算法

- 检测：OpenCV ArUco `DICT_4X4_50`，目标 id=0，码边长 10cm（`config/recharge_mark.json` 的 `aruco` 段）
- PnP：`SOLVEPNP_ITERATIVE` + `solvePnPRefineLM`（对齐原 arucotag）
- DockPose：`x=-(cam_x-30mm)/1000`（左目偏置），`y=-(码面距-6cm)`，`yaw=heading_y`；左右符号来自 PnP `cam_x` 与四角竖边 `trap_sign`

## 分支与发布

- **`master`**：源码仓，改算法只改这里的 cpp
- **`main`**：只推送头文件 + `librecharge_mark.so`，给感知仓 submodule
- 感知仓不编 cpp，只链 submodule 里的头 + so
- 对外 ABI：宿主 `RechargeMarkHost` / 底层 `RechargeMarkApi`
  （**单头文件** `include/percep_demo/api_recharge_mark.h`，
  感知仓通过 `recharge_mark` submodule 引用该头，不再保留本地副本）
- 在线：感知把 GDC SHM 帧指针交给 `Tick()`；解 NV12 / 内参 / 发布在 so 内

## 目录（本仓）

| 路径 | 说明 |
|------|------|
| `include/percep_demo/api_recharge_mark.h` | 稳定对外 ABI（单头：API + 宿主） |
| `src/*.cpp` | 算法实现（只在本仓编译） |
| `src/recharge_mark_logger.*` | 独立日志（写 `/userdata/log/ai_node/recharge_mark.log`） |
| `scripts/sync_to_ai_ws.sh` | 源码同步到 `ai_ws/src/recharge_mark` 供交叉编译 |
| `scripts/export_sdk.sh` | 把头 + so 拷到本地感知仓（调试用） |
| `scripts/publish_main.sh` | 把头 + so + **VERSION.md（md5/编译时间）** 推到 `origin/main` |
| `scripts/build_and_export.sh` | 同步 + Docker 编译 + 导出 + 推 `main` |

## 日常：改算法并更新感知仓

```bash
# 1. master 改 cpp / 必要时改头文件
cd /home/st/code/recharge_mark_lib
git checkout master

# 2. 交叉编译 so，并推到 origin/main（头 + so + 版本说明）
bash scripts/build_and_export.sh
# main 上查看 VERSION.md：so_md5、compiled_at；感知仓对照：
#   md5sum recharge_mark/lib/aarch64/librecharge_mark.so
#   grep '^so_md5=' recharge_mark/VERSION

# 3. 感知仓 submodule 钉 main（头文件随 submodule 引用，无需手动拷贝）
cd /home/st/code/point-cloud-processing-0.0.4recharge
git submodule update --init --remote recharge_mark
git add recharge_mark
git commit -m "chore: bump recharge_mark submodule (charging main)"
```

感知仓编 `percep_demo` **不必**再编二维码 cpp。CMake 用 `IMPORTED` 链接 submodule 里的 so。

## 接口约定（宿主侧）

一个回充识别任务的完整流程 = **初始化 → 开始任务 → 每帧喂数据 → 停止任务 → 析构**，
感知仓只需要下面 5 步：

```cpp
auto* host = Aiper_Recharge::createRechargeMarkHost();   // 创建
host->Init(so_dir, nullptr, &daye::getHandle());         // ① 初始化
host->Start();                                           // ② 开始任务
//  每帧: host->Tick(&latest_gdc, sizeof(CameraGdcData)); // ③ 喂数据
host->Stop();                                            // ④ 停止任务
Aiper_Recharge::destroyRechargeMarkHost(host);           // ⑤ 析构
```

| 步骤 | 接口 | 含义 |
|------|------|------|
| ① 初始化 | `Init(sdk_so_dir, start_or_cfg, rclcpp_node)` | 一步完成：独立日志打开 → 配置解析 → 回调注册 → ROS 发布器创建 |
| ② 开始任务 | `Start()` | 开 worker + 清滤波状态；对应 `ToPercepCmd cmd=2/3` |
| ③ 喂数据 | `Tick(gdc, bytes, settle_ms)` | 校验→内参→深拷贝→入队一步完成；忙时安全丢帧返回 1 |
| ④ 停止任务 | `Stop()` | 停 worker、丢未处理帧；对应 `cmd=21`；幂等 |
| ⑤ 析构 | `destroyRechargeMarkHost(p)` | Stop + 关日志线程 + 释放 api |

- **无 enabled 总开关**：二维码识别是必需功能，配置里没有 `enabled`；
  `publish_enabled` 只控制是否建 DockPose/overlay 发布器。
- **无 runtimeOn 状态**：任务运行状态由 so 内 worker 自管，感知仓不用查；
  未 Start/忙时 `Tick` 内部直接丢帧返回 1，天然安全。
- **无 requestAbort**：Stop 内部已包含中止等待逻辑，析构直接 `Stop()`。
- **日志独立管理**：so 内异步写 `/userdata/log/ai_node/recharge_mark.log`
  （10MB×5 轮转，行格式仿 ai_node：`[时间][级别] 内容`），含初始化/任务
  起/停/每帧二维码结果；不依赖感知仓日志。`setLogSink` 仅作可选镜像。
- **内参**：默认 `intrinsics_source=dataset`，用仓内标定值，`setCameraK`
  可不调；`sdk_scaled` 模式才需要感知仓把 CameraParam 传进来。
- 所有"save_raw / overlay"等开关都在 `recharge_mark.json`，
  Init 时一次读入，运行期不需要条件判断。
- YOLO charger 辅助裁图（`ai_recharge` 段 / `setExternalCropHint`）已移除。

离线评测工具仍用底层 `Aiper_Recharge::RechargeMarkApi`
（createRechargeMarkApi / init / processBgr / startWorker …）。
