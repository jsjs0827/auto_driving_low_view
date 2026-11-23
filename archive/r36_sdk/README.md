# r12-v33 full shared SDK

See [Chinese build and integration guide](README_SHARED_zh.md).

Ports the original local recharge, disparity consumer, PCL/Patchwork++, semantic fusion and three top-view maps into the preserved DAG / Task / Feature framework. The full C ABI retains the r32 inference API. Exact original detection letterbox and segmentation preprocessing/postprocessing remain in place; no cross-frame double buffer.

Native Windows ORT/OpenCV integration is tested. Full mapping requires genuine target PCL and remains pending X5 build and equivalence acceptance. The Ubuntu HOST build script generates the actual AArch64 SO and SDK, without unavailable ROS/SHM development headers.
