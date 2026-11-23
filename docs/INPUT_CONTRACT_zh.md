> **历史材料（v1/v2）**：本文保留用于理解原算法与历史验证。v3 的配置、路径、构建及运行入口已经改为纯 C++；本文旧命令和旧 DAG 配置不能直接用于本版。请以 [09_纯CPP输入与Graph迁移](09_纯CPP输入与Graph迁移.md) 为准。

# 直接传帧的 C ABI 填写

本页描述 SDK 帧缓冲契约。v7 的 camera 输入保留 SDK 原始采集身份；ros 输入保留 header.stamp 并生成配对序号；offline 输入生成明确标记的离线序号和主机时间戳，不表示硬件采集身份。

`psdk_full_frame` 必须先清零，填写 `struct_size=sizeof(frame)`、`abi_version=PSDK_FULL_ABI_VERSION`。

`left` 同样填写struct_size及PSDK_ABI_VERSION；format使用PSDK_FORMAT_NV12或PSDK_FORMAT_I420，width/height640/544，stride是实际Y stride，data/bytes指向完整YUV420存储。frame_id和timestamp_ns来自原采集帧。

地图需要真实 `psdk_disparity_q8`：PSDK_FULL_ABI_VERSION、width/height、实际stride_bytes、uint16 Q8数据和bytes、同采集frame_id/timestamp_ns；reserved保持0。Q8数值不做米/毫米深度转换。

回充需要真实同帧 `recharge_hires`：GDC-left NV12 1280×1088、实际stride、相同身份。不要把未矫正原图替代GDC图。

intrinsics为640×544左图对应内参；bxf=焦距像素×基线毫米，SDK投影时仅转换一次单位。

图片、GDC和Q8所有者必须活到process返回。Result必须活到全部读取/复制完成；结果释放后不能使用mask、detections、maps、pose overlay及trace的借用指针。

既有ROS调用方直接使用真实采集ID；只有标准ROS2 Image订阅示例因消息缺少数字序号而生成本地配对ID，JSON对此明确标注。
