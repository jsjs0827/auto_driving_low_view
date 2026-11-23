# 输入内参文件

相机直取模式自动从相机 SDK 获取矫正后内参，用户无需另外填写文件。本项目 ROS 相机节点也用 SDK 内参发布 `CameraInfo`，算法 ROS 输入默认接收同帧 `CameraInfo`。这里的文件用于离线图像或录制数据的几何处理：独立视差和点云都需要。

## 离线图像

在 `features/offline/config.json` 显式配置内参文件路径：

```json
"intrinsics_file": "calibration/intrinsics.json"
```

该路径相对主 `pipeline.json` 所在目录。将本目录的 `intrinsics.example.json` 复制为 `intrinsics.json`，保留 `width/height/fx/fy/cx/cy/bxf`，把所有 `null` 换成这组矫正后图像的真实标定值。模板本身不能直接用于运行。

| 字段 | 含义与单位 |
|---|---|
| `width`、`height` | 内参对应的处理图像尺寸，当前离线输入为 640×544 |
| `fx`、`fy` | 水平/垂直焦距，像素，必须为正 |
| `cx`、`cy` | 主点坐标，像素 |
| `bxf` | 焦距像素 × 双目基线毫米，必须为正；不是基线米 |

离线点云需要成对的校正后左右图，数量一一对应，并在 offline 配置明确 `rectified=true`、`calibration_confirmed=true`。图像经过缩放或裁剪后，内参也必须按同一变换调整；文件尺寸和当前 640×544 处理几何不符时应先修正，不能只改 `width/height` 绕过检查。Stereo 模型内部缩放到 640×352 不改变源图内参文件尺寸，推理输出会恢复到源图空间。

如果图中只运行检测/分割，未启用视差或点云，`intrinsics_file` 可以保持 `null`；仍需要有效左图。

## 同步录制与 A/B 回放

相机录制器的 `metadata.json` 包含 SDK 自动读取的 `intrinsics` 对象。可直接把该文件当作内参文件使用，无需另抄一遍数据。在主 `pipeline.json` 中：

```json
"recording": {
  "directory": "data/capture",
  "intrinsics_file": "data/capture/metadata.json",
  "limit": 100,
  "warmup_frames": 5,
  "repeat": 3,
  "save_every_n": 1,
  "viewer_max_points": 20000,
  "order": "original,stereo"
}
```

录制视差/点云回放同样要求显式填写 `recording.intrinsics_file`，不因为目录中恰好有元数据就隐式选择参数。文件可以是独立内参 JSON，也可以是带嵌套 `intrinsics` 的真实录制 `metadata.json`；加载器读取所需数值。`directory` 和 `intrinsics_file` 都相对主配置，不相对该功能目录或终端工作目录。

原视差/新 Stereo 比较应使用同一录制里的左右图、Q8.8 和同一内参。手工填写的独立内参必须有可核验来源，不能用不匹配的相机参数判断算法效果。

内参文件与点云外参是两类配置：这里负责像素和深度到相机坐标的转换，SDK 的 `extrinsics.json` 负责后续坐标变换，二者不能互换。

此目录名不代表图中的 `calibration` 功能开关；填写离线内参不要求启用自动标定节点，图中的可选标定阶段仍按原功能配置执行。

运行目录的 `resolved.json.calibration` 记录选用来源；离线文件和录制回放还会记录文件绝对路径及加载后的七个字段。相机实际读取和缩放后的数值、ROS 同帧数值可在结果 JSON 的 `calibration_source` 和 `intrinsics` 中查看；只运行推理且未用内参时来源为 `not_required`，不会用默认焦距伪装标定。
