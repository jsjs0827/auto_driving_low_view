X5 双目视差 SDK

include/percep_main/stereo_depth_runner.hpp：不依赖ROS/PCL/OpenCV的应用接口声明。
src/libstereo_depth.so：AArch64 C++17实现；由根交付包的sdk_source/stereo_depth构建。
CMakeLists.txt：导入StereoDepth::SDK并安装到lib/ai_node。

应用创建perception::StereoDepthRunner，先init(StereoDepthConfig)，再同步inferPair(left,right,K,&result)。
init仅加载一个model_path；应用根据crop开关选择对应full544或crop384模型。
config/stereo.example.json使用主应用的stereo.full_model_path和stereo.crop_model_path；应用将所选路径赋给SDK的StereoDepthConfig::model_path。
crop=false要求模型元数据等于原左右图尺寸。
crop=true只支持底部全宽裁剪，模型元数据必须等于source_width×crop_height；默认从640×544取(0,160,640,384)。
左右输入必须已双目校正；没有隐式图像resize、RGB往返转换、视差缩放或主点重置。
输出q8始终是原source_width×source_height的自有uint16数组，值/256为像素视差；无效值及crop外区域为0。
K是原图网格的内参，bxf_px_mm单位为像素×毫米；下游反投影Z米=bxf_px_mm/(d像素×1000)。

输入元数据：如果两侧frame_id都为0，表示未知；任何一侧非0时要求两侧相等。
timestamp_ns执行相同规则；两侧0不代表已证明物理同步。调用方应来自同一双目采集包或严格配对离线图像。
stride描述Y和NV12 UV行距；I420为相同Y行距与一半U/V行距。左右buffer在inferPair返回前须保持有效且不变。
init与inferPair内部串行；析构前调用方必须停止并join所有使用者。
失败时输出result清空，lastError给出具体原因；不会用旧帧结果或旧模型静默兜底。

计时为host steady_clock：pre=输入复制/裁剪+张量pack/cache clean；BPU=真实submit+wait（含runtime/设备排队）；post=输出invalidate/decode/release+原网格Q8生成。
total包含本接口检查/锁与其他host开销；不含应用的点云投影、过滤或地图任务。
本SDK仍需X5匹配的OpenCV、Horizon runtime和libstdc++ ABI；CMake依赖路径由交叉编译环境提供。
SDK验证不等同完整ROS地图节点已编译/上板通过；完整节点仍需要原工程匹配的相机消息和共享内存依赖。
