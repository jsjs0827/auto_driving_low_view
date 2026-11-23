# point-cloud-processing

# percep_main 感知演示模块
ROS2 感知算法 Demo 工程


## 项目简介
基于 ROS2 Humble 开发的感知功能演示模块，包含 AI 推理库、点云处理、线程管理等功能。
- 采用docker中的交叉工具编译链进行编译
- 基于 C++17 开发
- 依赖 PCL、Eigen、opencv 等库
- 部署至X5板端测试


## 环境依赖
sysroot_docker


## 目录结构
~/ai_root/
├── ai_ws     # 工作空间
├── robot_dev_config      # 编译脚本
├── src/
│   ├── ai_node  # ai节点代码
│   │     ├── ai_node  # 进程节点
│   │     └── point-cloud-processing   # （本项目）算法模块库
│   ├── common_interface  # 通用通信接口
│   ├── script  # 启动脚本
│   └── thirdparty   # 三方库
└── build/install/log    #ros2编译生成文件


## 编译方法
```bash
cd ~/ai_root
./docker.sh #进入docker
./env.sh   #初始化环境变量
cd ~/ai_root/ai_ws/
./robot_dev_config/build.sh -p X5
```


## 回充二维码（submodule 钉 charging.git `main`）

算法源码在 `charging.git` 的 **`master`**。本仓 **submodule 只钉 `main`**（头 + 预编译 so），不编 cpp，也不再静态链 OpenCV：

| 位置 | 职责 |
|------|------|
| `recharge_mark/`（submodule → `main`） | `api_recharge_mark.h` + so（识别 + 异步 worker + `submitFromShm`） |
| 本仓 `percep_main.cpp` | SHM 订阅、`cmd=2/3` 启停、`submitFromShm`、发 `DockPose` / overlay |

首次克隆：

```bash
git submodule update --init --recursive
```

升库后对照 md5：

```bash
md5sum recharge_mark/lib/aarch64/librecharge_mark.so
grep -E '^(so_md5|compiled_at|published_at)=' recharge_mark/VERSION
```

二维码仓改算法：`recharge_mark_lib` 上 `bash scripts/build_and_export.sh` 推 `main`，然后本仓 `git submodule update --remote recharge_mark`。

## 运行方法
```bash
cd ~/ai_root
./adb_push.sh
adb shell  #进入板端
cd /app/daye/bin
./start_percep.sh
```

## 模型部署（DNN 检测 / 分割）

板端模型统一放在 **`/app/daye/model/`**，路径须与配置文件一致：

| 用途 | 板端路径（默认） | `start_config.json` 字段 |
|------|------------------|--------------------------|
| 目标检测| `/app/daye/model/mmyolo0726.bin` | `dnn.detect.model_path` |
| 语义分割| `/app/daye/model/pidnet_26_0630_max.bin` | `dnn.seg.model_path` |

更换模型时：将 `.bin` 放到上述目录，并修改 `start_config.json` 中对应 `model_path` 后重启感知。

配置文件路径：`/app/daye/config/ai_node/start_config.json`

### 障碍点云类别标签（`/percep_main/det_cloud` 的 `label` 字段）

检测与分割 ID 空间独立，点云统一约定：

| 来源 | label 取值 | 说明 |
|------|------------|------|
| 检测 | `0..20` | 即检测 `class_id`（`dnn.detect.class_names` 下标） |
| 分割 | `100 + seg_id` | 分割 `class_id` + 100（`seg_id` = `dnn.seg.class_names` 下标，如 3 类模型 grass=`101`） |

当前默认 `keep_classes=person,cat,dog,charger`。草是否可通行由 `label_fusion.passable_classes` 按**名字**匹配，换分割类别表时只要仍叫 `grass` 即可，不必改 fusion。

换分割模型：只改 `dnn.seg.model_path` 与 `dnn.seg.class_names`（可写 `0:no_grass,1:grass,2:flagstone`）；`num_classes` 可省略，启动时以 bin 输出通道数为准。

## 版本特性
## V0.0.1
0.感知模块配置文件路径：/app/daye/config/ai_node/start_config.json

1.由中间件控制感知初始化，暂停，恢复工作，感知启动后会作为服务端阻塞等待中间件通信消息，初始化完成后自动进入计算发布局部感知地图

2.可配置是否保存输出日志地图，黄色为未知，黑色为地面（非草），红色为障碍物

3.增加独立相机自动标定功能，配置文件可开关，可进行高度(z)，翻滚角（roll）,俯仰角（pitch）三维标定，其余三维X,Y,yaw需进行人工手动标定，
若开启自动标定，则不会输出感知结果，终端实时输出标定结果，需手动填入配置文件中的外参

4.当前版本地图标签：`0=未知, 1=地面/可通行, 2=障碍`（仅 PatchWork 几何分割，AI 不写栅格）；带 AI 标签点云经 `last_semantic_cloud_` 交给下游写栅格
地图格式可在配置文件更改，MapResolution默认为0.02m，MapWidth默认为4.0m,MapHeight默认为2.5m
地图原点在底边的中心，为车体中心坐标系

5.相机感兴趣区域FOV范围可通过配置文件进行配置(相机坐标系)

6.GridMapInfo地图输出接口，header中存放时间戳，rows为行数，cols为列数，resolution_mm为毫米分辨率，
单位栅格：avg_h_cm为的厘米平均高度，min_h_cm为厘米最低高度，max_h_cm为厘米最高高度，type为感知标签（见上）

7.配置文件可配置实时可视化功能开关，默认为关闭，打开后仍需配置外部通信环境，多机通信需保证网段一致且ROS_DOMAIN_ID一致


## V0.0.2
1.配置文件可更改在线或离线模式，在线模式从相机获取实时数据，离线模式从本地离线录制数据获取数据，默认为在线,完善了离线回放功能，增加交互式键盘按键控制回放，
将可视化分离为左中右三路显示，左边为RGB投影点云，中间为后处理点云，右边为栅格地图投影，
目前支持的数据格式：矫正后的nv12图像，视差图，分辨率均为640*544，格式均为jpg
离线数据目录：/app/daye/bin/offline_data/rgbd_disp,   /app/daye/bin/offline_data/rgbd_left

2.增加视差图可视化

3.增加中间件控制相机sdk的启停功能，并增加中间件查询摄像头工作状态接口

4.更改中间件通信接口，改为感知自动初始化并进入待机状态，中间件发送启动感知地图发布与启动基站二维码发布两种信号，分别启动不同模块进行数据发布，暂停与恢复工作不变
当前逻辑：感知与相机解耦，感知启动后若相机未打开自动进入待机状态，感知启停与相机启停均由中间件控制，感知与相机状态均可通过中间件查询

5.更改地图默认参数 MapWidth: 6.0,MapHeight: 3.5


## V0.0.3
1.合并感知二维码识别定位功能

2.合并中间件通信逻辑，增加独立控制感知地图发布与感知二维码位姿发布功能
终端自测指令：
```bash
ros2 service call /to_percep_cmd common_interface/srv/ToPercepCmd "{cmd: 1}"  //感知启动感知地图发布模块
ros2 service call /to_percep_cmd common_interface/srv/ToPercepCmd "{cmd: 2}"  //感知启动基站二维码发布模块
ros2 service call /to_percep_cmd common_interface/srv/ToPercepCmd "{cmd: 21}" //感知停止工作，进入待机

ros2 service call /to_camera_cmd common_interface/srv/ToCameraCmd "{cmd: 1}" //查询摄像头工作状态  //-1 - 异常，0 - 关闭，1 - 打开
ros2 service call /to_camera_cmd common_interface/srv/ToCameraCmd "{cmd: 2}" //请求打开摄像头
ros2 service call /to_camera_cmd common_interface/srv/ToCameraCmd "{cmd: 3}" //请求关闭摄像头

ros2 service call /get_percep_status common_interface/srv/GetPercepStatus "{cmd: 1}"   // -1 - 异常，0 - 待机中，1 - 感知地图发布中，2 - 基站二维码发布中
```
3.修改timer轮询时间为10ms

4.增加self_test自测配置选项，打开后可以不发送指令跑整个感知所有在线功能

5.定位opencv resize崩溃问题，4.5.4与地平线x5不兼容，需要更换版本，目前采用手写最近邻缩放解决

6.修复保存可视化日志时间戳异常问题,路径异常问题,但是网格与坐标系无法绘制

7.去除相机控制逻辑，改为初始化自动打开，析构时关闭

## V0.0.4
1.增加AI模块的初始化与推理

2.增加点云与AI栅格地图融合功能,目前采用各自点云投影地图后单位栅格融合

3.新增人，猫，狗，基站，四种类别的合入

4.将可视化日志改为生产消费线程，异步存储最新帧

5.优化整体耗时，目前总耗时在150ms内(开启分割并行)

6.优化点云与地图后处理，加强去噪效果，同时膨胀草坪去除空洞

7.优化可视化地图保存，增加网格线与坐标系绘制（采用最新的sysroot版本）

8.优化自动标定，每间隔五帧计算平均值自动写入文件，拆分外参json文件

9.解耦相机，与从中间件共享内存获取相机数据，始终获取队列中的最新帧

10.增加FOV内的点云视差存储，增加相机相关参数存储

## V1.0.1-2026.09.2-EB-20260930
1.重构整个感知模块，包括点云，AI，二维码等子模块

2.优化双目拖影去除算法

3.增加健康码系统

4.增加坡道检测功能

5.开放高度阈值调整以检测低矮障碍物

6.更新二维码子仓库 - 
## 2026-09-20T11:39:37+08:00  108129b4ce78  md5=4ab83f3bf3c7c8162d36e7cb2ee0f5bc
- 5hzgdc版本稳定5hz识别并发布,10hzgdc版本识别5-6hz,cpu板端170左右(测试环境不同，最新环境实测约9hz)

## V1.1.1-2026.10.1-EB-20261030