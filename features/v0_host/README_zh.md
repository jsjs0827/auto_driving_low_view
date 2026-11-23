# V0 宿主入口

这个模块只负责输入、触发和输出适配。离线/ROS 入口与它共同使用
`perception::entry::Session`（既有 `ros_so_demo::Caller` 的别名），最终调用同一
`psdk_full_process`、SDK 配置和 DAG；这里没有另一套点云、回充或 Stereo 算法。
相机 producer 仍由外部宿主启动。模块不使用、复制或链接 `local_only/` 的私有共享内存实现。

## 输入和触发合同

- `ToPercepCmd(1)`：接收原 V0 `CameraData`，`original_q8` 使用原 RGBD left + SDK Q8；
  `raft_stereo` 使用同一原始包中的 left/right，复用现有 `StereoProvider` 的 hooks、
  `setPair` 和 `clearPair`。缺右图或 Q8 时明确拒绝，不切换算法来源。
  right 是编译期可选的上游扩展；匹配 ABI 没有 right_data 时 original_q8 仍可构建，选择
  raft_stereo 会在启动时明确报 missing_input，不要求旧 producer 自动产生右图。
- `ToPercepCmd(2)`：只接受 producer 的真实 1280×1088 GDC left；同一张图同时作为
  full-frame left 与 recharge_hires。不会把 640×544 RGBD 放大或改写身份来凑齐 GDC。
- `ToPercepCmd(21)`：停止并清单槽。执行与发布串行，停止响应不会先于旧帧的发布。
  同模式心跳不增加 generation、不清单槽；Session 负责同命令幂等和 pose gate 状态。
- `ToPercepCmd(9002)`：宿主先停止，再清理配置明确指定的
  `factory_reset_directory` 日志叶目录。未指定时通过 App 状态 2 报告执行失败；不把 9002 传给 SDK，
  不隐式删除固定 `/app/daye` 路径。该能力没有在本机执行过。
- `GetPercepStatus(cmd=1)`：返回 -1 异常、0 待机、1 map、2 recharge。
- CameraParam 仍按 V0 校验 fx/fy、cx/cy、毫米 baseline 和 bxf；图像尺寸相同比例时显式缩放
  fx/fy、cx/cy、bxf。换参会清待处理帧，不把新参数应用到已排队旧图。

对已识别的 1、2、21、9002，保持 V0 同步 `ToPercepCmd::Response.result=0` 约定；它表示
识别该命令，不能用来判断实际执行成功。完成结果仍经 ToAppCmd 上报：2001、2002、2021、9002
分别对应这些命令，status=1 成功、status=2 失败（包括 SDK 拒绝、停止/重置失败和控制器关闭）。
不支持的命令采用明确的适配器错误策略：同步 result=-1 并记录错误，不编造 App 事件编号。

接收端只保留一个最新待处理帧；正在计算的帧另持一份所有权。计算和发布期间只持 execution
锁，输入回调和状态查询使用独立短锁，能继续替换待处理槽。所有原始帧号和时间戳均保留；
RGBD left/Q8 或 Stereo left/right 身份不同时拒绝，不在独立 RGBD/GDC 流之间猜同步。

`BoardInput` 是不需要 ROS/daye/模型库的输入核心；`BoardController` 将它串到公共 Session。
`onFullFrame(psdk_full_frame, shared_ptr<const void> owner, error)` 供更完整的真实输入 binder 使用：
它逐值复制 hires/right/rear/Q8/RAW/IMU/wheel/radar 的 descriptor wrappers，`view()` 在队列移动后
重绑到本对象稳定存储；底层数据指针和元数据不变。提交时所有 descriptor 必须有效；`owner`
必须持有全部图像、RAW、样本和雷达数组，直到同步 process 返回或该待处理帧被替换/清除。
不要仅传一个不持有这些 buffer 的空壳 owner。这样 `recharge_hires=&source.left` 也不会因
source descriptor 的值拷贝、移动或离开作用域而悬空。

现有 V0 decoder 只解释已知的 CameraData/CameraGdcData；它不编造 RAW 或其他传感器 ABI。
没有上游 RAW/传感器字段时，这些 optional SDK 槽为空；共用 SDK 保留原来的 missing_input/
valid:false 语义。所有算法 JSON 显式发布到 `/percep_main/algorithm_results`，不会静默改 DAG。
更完整的输入需通过以上通用 binder 提交真实数据，再验证具体板端 ABI。

## 插件与构建

真正插件导出 `REGISTER_FUNCTION`，注册 `percep_main`，产物名 `libpercep_main.so`，由原宿主
加载。原 Function 配置增加 `v2_host_config` 指向配置工具生成的绝对路径 `v0_host.json`；
此文件 schema_version=1，包含 sdk_config、algorithm_contract_sha256、disparity_source、
initial_command=21，以及 RAFT 的现有 stereo 设置。其他 metadata 字段可以保留。
真实接入必须在 Function JSON 或 v0_host.json 中明确提供 `dock_pose_topic`，值须来自待接入
生产系统的实际约定；Function JSON 优先。未提供时启动拒绝，不猜测 DockPose topic。
发布还要求 SDK 的 executed、mark_ok、pose_ok 同时为真。

Function 可选 parent_frame（默认 base_link）。GridMapInfo 默认沿用原 V0 的
`daye::getSteadyTimeMs()` Header 时间语义和毫秒精度；SDK 输入、结果和算法诊断保留原始
capture 身份与时间戳。若明确需要采集时间，可在 Function JSON 或 v0_host.json 配置
`grid_header_clock="capture_ns"`；默认值为 `legacy_steady_ms`，Function JSON 优先。
其他字符串拒绝启动。GridMapInfo topic 继续取真实上游 `kGridMapInfo`。

上述四项输出/宿主参数统一按 Function JSON 显式字段 → 生成的 v0_host.json 字段 → 默认值
解析；配置工具可从顶层 deployment.v0_host 透传它们。dock_pose_topic 无默认值且必须非空；
grid_header_clock 默认 legacy_steady_ms；parent_frame 默认 base_link 且必须非空；
factory_reset_directory 默认空字符串，空表示禁用日志清理。显式错误类型或不允许的空字符串
会拒绝启动，不用下层值覆盖 Function JSON 的显式值。解析配置不会创建或清理目录。

默认 `PERCEPTION_BUILD_V0_HOST_PLUGIN=OFF`。普通 SDK 不依赖 daye/ROS；纯输入 target 为
`perception_v0_host_input`，Session 控制 target 为 `perception_v0_host_core`。插件需同一构建中已有
`sdk_bridge` 和 `stereo_provider`，显式 ON 后还必须提供：

```text
PERCEPTION_V0_FRAMEWORK_ROOT=<匹配原版 thirdparty 根目录>
PERCEPTION_V0_INTERFACE_INCLUDE=<匹配 common_interface include 根目录>
PERCEPTION_V0_BASE_FRAMEWORK_LIBRARY=<匹配架构 libbase_framework.so>
```

依赖 gate 在找不到真实头、发现编码头或旧 SHM 布局时终止；真实 ROS 类型的编译探测继续核对
CameraData/CameraGdcData 和 CameraParam/topic 声明。这里只提供 build-tree 插件，不安装或复制
上游依赖到普通 SDK。不得以 synthetic/stub ABI 绕过 gate。

## 本机证据与尚未验证范围

定向搜索找到了 `/home/st/ini_code/ai_ws/src/thirdparty/include/base_framework/` 下真实的
function_factory.h、node_factory.h、daye_shm.h，及 thirdparty/lib/amd64、aarch64 的真实
libbase_framework.so。但 ai_ws/src、install、obs_x5_run 和 x5_validation 内找到的
camera_struct.h 都是较旧 PicRgdData 内嵌 data 版本，没有 1104 入口需要的 CameraGdcData、
left_data/right_data 与 disp 元数据；匹配的 CameraParam 消息与 kCameraParam/GDC 声明也未找到。

因此当前能验证输入/控制核心和公共 SDK 合同，不能声称已编译或运行真实 V0 插件、已打通真实
SHM 或板端发布。获取匹配上游依赖后，仍需实际编译插件并验证 producer 布局/容量、相机格式、
同帧身份、CameraParam、命令切换、GridMapInfo、DockPose 与可选输入的实机行为。
