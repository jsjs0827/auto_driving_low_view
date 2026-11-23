本包为 v2，包含 Tensor::validate 链接修复，编译与运行请优先阅读 r22_link_fix_zh.md。

# r22 FMA 修复 v1：检测 U 像素 127→126

修复对象是 r22 的精确缩放校准，不改模型、检测 letterbox、分割原图直接缩放、检测解码或分割后处理，不引入跨帧双缓冲。本包是完整源码，需要重新交叉编译，不能只替换 X5 的 Python 脚本。

## 已确认的故障

失败于 `verify_same_frames`，该阶段比较原始标量与 r21 输入路径，尚未测试新 cached/direct。FrameId 36 的检测 prepared 和 BPU 输入存储在同一偏移 `481404` 各有 1 字节差异，所以 CSV 总计为 2。该位置是检测 NV12 的 U(78,77)，位于 letterbox 有效区域，不是补边或对齐尾部。原始值为 127，r21 为 126。分割输入、分割 mask 和检测结果均相同，但仍不满足零字节差验收。

该像素源 U 四角是 `[127,126,124,129]`，权重 wx=0.44326257705688477、wy=0.1666717529296875。旧校准选中的 mode 3 得到 126.49999237060547，加 0.5f 后转换成 126；mode 0/1/2/4/7 得到 126.5，转换成 127。不能仅凭单个像素把整个检测固定为 mode 4。

旧校准从另一编译单元的标量副本选择一个全图 FMA 模式。该副本校验通过，不足以证明实际 runner 的 Y/U/V 运算次序均相同。运行库与模型构建版本警告不能解释推理前已保存的 CPU 输入字节差；本修复保持原运行库与模型用于受控对比。

## 代码修复及验收

- 真实 runner 的三条输入路径改用 runner 内原始标量 helper 进行初始化校准，主机脱离 runner 的测试才默认使用独立参考副本。原始标量 helper 内容保持原哈希。校准样本按打包之前的原始 I420/NV12 格式与步长进入标量 helper，避免 NV12 与 I420 调用分支的编译收缩顺序混用；格式转换仅用于初始化校准。
- Y、U、V 分别从 9 种 FP32 水平/垂直顺序中取交集，覆盖首帧、两组全范围样本、三组低对比度样本和本次临界四角样本；回调、原始格式/步长、几何或舍入模式变化时重新校准。无精确匹配就拒绝运行。
- r21 仍使用原 4 字节缩放、每帧补边和输入复制，但校准机制已修复；不是旧 r21 可执行文件。旧 r21 缩放源码另存为 reference/r21_legacy_preprocess.cpp/.hpp，受哈希守卫。
- cached/direct 保留 8 字节 NEON、16/32 字节查表、补边缓存和直接 tensor 写入；UV 顺序一致时走原专用循环，不一致时按 U/V 选择结果。每帧 cache clean 保留。
- 新增 det/seg_math_y/u/v CSV 字段。items/*/differences.json 保存 prepared 与 storage 分开的差异数、分量、坐标及字节值。零输入差、零 mask 像素差和检测位级一致门槛不变。
- ARM exact_input_tests 连接实际 runner helper，要求本次临界 U 值仍为 127，并验证修正 r21、cached/direct 与它一致。不能改写基线为 126 来隐藏差异。

本机是 Windows，主机测试和 ARM64 前端检查不能代替板端 NEON/BPU 验收，实际加速仍需本包 X5 A/B。完整记录见 validation/r22_fma_fix_host.json。

## Ubuntu 宿主机编译

将本包和同名 SHA256 文件放在 /home/st/ini_code。沿用已运行的 obs-v004-x5-build 容器，不进入 Docker、不拉镜像、不重新编译模型。

```bash
cd /home/st/ini_code
sha256sum -c perception-x5-exact-input-r12-v22-fma-fix-v2.tar.gz.sha256
tar -xzf perception-x5-exact-input-r12-v22-fma-fix-v2.tar.gz
cd perception-x5-exact-input-r12-v22-fma-fix-v2
sudo bash tools/build_in_container.sh \
  --container obs-v004-x5-build \
  --sysroot /work/obs_v004_x5/sysroot_docker \
  --out /home/st/ini_code/r22_fma_v2_release
```

输出 SDK 包和校验文件：perception-sdk-x5-r12-v22-exact-input-fma-fix-v2.tar.gz / .tar.gz.sha256。复制两者到 X5 /userdata/ai_infer_r12_v22/，使用新目录，不覆盖旧失败证据。

```bash
cd /userdata/ai_infer_r12_v22
sha256sum -c perception-sdk-x5-r12-v22-exact-input-fma-fix-v2.tar.gz.sha256
tar -xzf perception-sdk-x5-r12-v22-exact-input-fma-fix-v2.tar.gz
cd perception-sdk-x5-r12-v22-exact-input-fma-fix-v2
```

## 先重放旧失败批次，再测在线性能

冻结数据使用原失败批次的 capture_original/source.tsv。下面会先验收原始/r21及三条输入，再做固定输入性能，不打开相机；不是跳过验收。路径存在则可直接执行。

```bash
bash tools/run_x5.sh \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --input-list /userdata/r12_v22_res/input-ab-20250903-080800-3560979/capture_original/source.tsv \
  --schedule free --frames 30 --warmup 5 --rounds 1 \
  --out /userdata/r12_v22_fma_replay
```

重放全零验收通过后，运行完整相机 A/B。disparity-mode 0 仍保留原视差设置，不自动关闭视差。

```bash
bash tools/run_x5.sh \
  --det-model /userdata/ai_infer/data/mmyolo0828.bin \
  --seg-model /userdata/ai_infer/data/pidnet_0825_3classes.bin \
  --sdk-policy full-sync --stream gdc --format i420 \
  --disparity-mode 0 --schedule free \
  --frames 120 --warmup 20 --rounds 2 \
  --out /userdata/r12_v22_fma_res
```

有限测试完成后退出。回传 input-ab-* 全目录。若失败，failure_details.json、失败阶段 frames.csv / 日志、items/*/differences.json 已能直接指出字节及通道模式；保持原始输入文件以便复现。

源码、脚本、配置及校验清单按最终 UTF-8、无 BOM、LF 字节生成并逐文件核对 tar，无需 dos2unix，也不重写旧失败 SDK 的 SHA256。
