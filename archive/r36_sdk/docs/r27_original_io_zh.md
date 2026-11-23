# r27：当前检测、分割与原始项目的输入输出核对

本报告核对的是原项目**地图任务**的检测与分割入口，以及当前精确 CPU 路径。结论是：原始帧的选择规则、检测 letterbox、分割直接缩放、模型、检测解码和分割输出语义保持原契约；优化改变的是运算组织、内存写入和调度。当前测试保存的是 runner 层结果，不能将其称为原产品经过类别过滤、点云融合与发布后的全部业务结果。r27 暂不包含 JSON 配置简化版。

静态代码核对记录见根目录 `R27_IO_CONTRACT.json`。X5 上的新一轮全速/并行结果尚未产生，以下区分代码事实、r26 已有实测和待测事项。

## 1. 原始项目实际上有两个入口

| 入口 | 原始项目输入 | 运行内容 | 当前对照范围 |
|---|---|---|---|
| 地图任务 | `latest_rgb.left` / `latest_rgb.left_data`，RGBD 左目 640×544 | 检测＋分割 | **本轮对照的原始入口** |
| 单独回充、二维码辅助 | 优先 `latest_gdc.left_data`，GDC 1280×1088 | 只检测，不运行分割 | 不属于本轮输入等价声明 |

原始选择逻辑在 `reference/original_product/src/percep_demo.cpp` 的 `want_dnn` 分支。640×544 不是当前程序将 GDC 1280×1088 缩小后冒充原图，而是原地图任务本身接收的 RGBD 左目。原项目的 `image_pub.width=864`、`height=480` 是发布图设置，不是地图 AI 原图分辨率。回充 GDC 分支应另行对照，不能混用它的输入与本轮结果。

## 2. 原始字节及 NV12 的解释

原业务配置 `reference/original_product/config/start_config.json` 明确设置 `dnn.left_yuv_format="nv12"`。`dnn_runtime.cpp::ensureLeftYuvFormat` 按这一配置锁定布局，然后把原字节、宽高、stride 传给 `HorizonDnnRunner::inferNv12`。当前路径保留同一规则，不根据颜色猜测格式，也不自动切换 I420。

实际按 NV12 访问的含义是：前 `stride×544` 字节为 Y 平面；后面是 272 行 UV 交错平面，行步长仍为 `stride`。对色度格 `(u,v)`，读取 `UV[v×stride+2u]` 为 U、下一字节为 V。紧凑 stride=640 时原帧为 **522,240 字节**。检测和分割都直接对 Y、U、V 做双线性采样；此入口没有 NV12→RGB→NV12 往返，没有额外 RGB 归一化或通道互换。

须区分“原程序按 NV12 解释”与“SDK 物理输出已被独立证实为 NV12”：SDK 接口/原源码有 `yuv420p` 注释，单凭注释或未建立映射的 `StreamType`、`RGBImageType` 等字段不足以证明色度物理布局。本版首先保持原程序解释，避免以纠正颜色为由改变模型输入。若要独立证明与运行中的旧产品程序取到同一份数据，需要同时 dump 旧程序 `latest_rgb.left_data` 和当前 SDK 返回帧，并用源时间戳/帧身份对齐后逐字节核对。r26 的冻结帧验证尚未完成这项跨进程同帧核对。

当前契约还比原 runner 更严格：必须是 640×544、NV12、共同偶数 stride，字节数恰好 `stride×544×3/2`；异常输入直接报错，不回退到 GDC。stride=640 时快速打包只返回原指针；大于640时按行去掉行尾填充，保留全部有效 Y/UV 像素。I420 转 NV12 的通用实现仍存在，但不是本轮允许的输入。

## 3. 检测输入：letterbox 几何及数值保持原规则

| 项目 | 原始 runner | 当前 `compact_direct` |
|---|---|---|
| 原图 | 640×544 NV12 | 同一冻结原帧 |
| 模型 | `mmyolo0828.bin`，24类 | 相同 BIN，以 SHA256 核对 |
| 模型输入 | 864×480 NV12，622,080字节 | 相同尺寸与实际 BPU 输入字节 |
| 缩放 | 原 FP32 双线性，Y 与 UV 分别计算 | 缓存坐标/权重、NEON 紧凑查表，按原 FMA/舍入顺序计算 |
| 补边 | Y=127；U=128、V=128 | 相同；固定补边复用 |
| 写入 | 临时 resized→canvas→BPU 输入 | 直接写 BPU 输入；每帧 cache clean 保留 |

原几何计算为：

```text
r = min(480/544, 864/640) = 15/17 ≈ 0.88235294（FP32）
new_w = int(640*r) & ~1 = 564
new_h = int(544*r) & ~1 = 480
x_shift = (864-564)/2 = 150
y_shift = 0
```

所以检测画布有效区域为 `x=[150,714)`、`y=[0,480)`；左右各补150列。色度有效格为282×240，UV行左右各补150字节。这里不是把原图直接拉伸成864×480。

有效缩放的采样坐标仍按输出宽564、高480计算：`sx=(x+0.5)*640/564-0.5`、`sy=(y+0.5)*544/480-0.5`，边界钳位；UV以320×272→282×240独立计算。最后按原 FP32 `value+0.5` 转成 uint8。r22 曾在半整数附近出现127/126一字节差异；目前逐通道校准对齐的是实际 ARM runner 的 FMA 顺序，不是允许误差阈值。

**坐标还原保留原细节：** 原程序用理想缩放系数 `r=15/17` 和偏移150还原检测框，并未改用 `564/640`。两者因有效宽取偶数而略有差别，当前刻意保留原行为。XYXY 为 float，钳位到 `[0,640]×[0,544]`，不是整数像素坐标，也不是归一化坐标。对原规则的几何修正将改变输出，不属于本轮加速。

## 4. 检测输出：模型解码一致，业务过滤层有范围差异

原解码与当前解码函数正文逐字一致，见 JSON 的函数 SHA256 核对：

- 三尺度步长8/16/32，网格分别108×60、54×30、27×15；按模型 shape 匹配6个分类/回归张量。
- 每格取最高类别 logit，分数 sigmoid；阈值0.25，对应 logit 阈值约−1.0986123。
- 四方向各16个 DFL bin，原 exp/加权期望和坐标公式保持。
- 分数从高到低，**同类别** NMS，IoU **大于**0.7才抑制；保留总量最多300，不是每类300。
- class_id、float score、float XYXY 和类名保持。当前 DAG 的通用 `DetectionResult` 保存数字字段，原 `DnnDetection` 含类名并保存在 `exact.detections` / `detections.json`，属于封装差异。

**原产品还有下一层：** `reference/original_product/src/dnn_runtime.cpp::runDnnInference` 在 runner 返回后执行 `keep_classes="person,cat,dog,charger"`，其后还可能做点云标签融合、回充框使用和可视化发布。当前测试包保存的是 **keep_classes 之前的 runner 检测结果**，因此可能出现原产品最终列表中过滤掉的其他20类。模型、阈值、NMS未变；输出层级不同。仅比较类别过滤后的业务列表时，应对当前结果应用同样名称过滤。没有在数值/性能基准中悄悄加入新的过滤阈值。

## 5. 分割输入输出及保留的后处理

| 项目 | 原始 runner | 当前精确路径 |
|---|---|---|
| 原图 | 640×544 NV12 | 同一原始帧 |
| 前处理 | **直接缩放**至832×480，不加 letterbox | 相同采样、FMA/舍入规则；缓存表＋直接写输入 |
| BPU 输入 | 832×480 NV12，599,040字节 | 相同实际输入 |
| 模型 | `pidnet_0825_3classes.bin` | 同一 BIN，未改成融合模型 |
| 低分辨率输出 | 3×60×104 logits，按真实 tensor layout/量化读取 | 相同；统一解码成 CHW float32，共18,720个值 |
| 类别 | 0=`no_grass`、1=`grass`、2=`flagstone` | 相同 |
| 最终输出 | 640×544 uint8类别 mask，348,160字节 | 相同尺寸、类别含义和 > 比较的 tie 规则 |

前处理是半像素坐标直接双线性拉伸：Y的横纵比例为832/640=1.3、480/544≈0.88235294；UV从320×272变成416×240。这里没有改变成 letterbox，也没有使用 VSE。

后处理仍是 `argmax_mode=bilinear`、`align_corners=true`：输出像素 `(x,y)` 对应低分辨率坐标 `ox=x*103/639`、`oy=y*59/543`。从相邻四角读取每类 FP32 logits，先横向、再纵向插值，最后做 ArgMax；不是对低分辨率类别 mask 做双线性，也不是先 ArgMax 再最近邻放大。

原 runner 已有数学等价的 shortcut：四角同类或退化为同类单点/边时直接写类别；否则执行每类完整插值。当前保留这些条件，把共享四角的横向像素合成 run、缓存 X/Y 映射、用 `memset` 批量填充同类区域、复用 logits/coarse mask 的 workspace。**复杂边界的完整插值分支仍然存在，帧间边界数量变化仍可能导致耗时波动。** 本版不是将 Resize→ArgMax 编译入 BPU 的融合路线。

原业务配置中 `label_fusion.seg_mask_postprocess=false`；runner 之后的产品函数仍有调用位置。当前测试的输出层是 runner mask，没有加入开闭运算、腐蚀、点云连通域或语义融合后的空间结果。不能以 runner mask 一致来声称所有下游点云/地图结果已验证一致。

## 6. 已有实测证据与它能支持的结论

来源：r26 `original-input-ab-20250904-053244-4052063`，修订 `r26-arm-calibration-fix-v1`；不是新 JSON 简化版，也不是 r27 新全速测试。

- 10个唯一相机源帧，每组各20次比较；r21↔original、compact_direct↔original、compact_direct↔r21，共60次同帧比较。
- prepared NV12 与实际 BPU input storage 的240对文件全为零字节差；60对分割原生 BPU 输出、解码 logits、最终 mask 为零差；60对检测结果 bit一致。
- ARM 的原输入契约/校准通过；合成分割后处理168例零像素差，累计覆盖35,434,455个完整边界插值像素。这证明实现有边界覆盖测试，不能将它当作真实相机多场景测试。
- 真实10帧的低分辨率 ArgMax和最终mask全部是类0；检测仅见类18 charger。真实边界分支未触发，没有 grass/flagstone 的真实精度覆盖。
- 没有人工标注，不能给出 mIoU/mAP，也不能把“零差”说成“准确率100%”。

因此，目前可以说**所测同帧、同输入、同 BIN 下的 runner 结果一致，精确优化的实现与契约相符**；还不能说原产品所有场景、所有业务分支和输出已完全等价。

## 7. r27 全速与视差并行测试仍需验收的部分

本版新增 **mode 1＋RGBD** 的显式能力探测。SDK 静态控制流显示：mode 1仍把640×544左右色图交付给RGBD接口，同时将同步VSE色图对复制到按需视差cache；连续视差分支跳过推理。mode 1下RGBD返回的视差ID、时间戳、尺寸等字段可能全部为0，这属于视差未生成，不能当成左右色图不同步。左右色图的有效性与三元组视差元数据有效性必须分别记录。静态证据见 `validation/r27_sdk_mode1_static.json`；分析用的参考SDK与板端记录SHA256不同，故静态证据只支持实现probe，不能替代该板端的成功验收。

`moCalculateDisparityByFrameId` 按公共帧号定位cache，原SDK用公共ID×3，全速隔离补丁需同步改成ID直映射。此API返回码、像素和尺寸，**不返回输出自带的帧号/时间戳**。包装结果中的frame_id是请求cache key，不是从视差输出中读出的独立身份。因此，同帧配对依据只能明确记录为“RGBD返回ID＋已审计的cache映射＋实际调用成功”，不能把包装ID相等夸大为输出身份的独立证明。按需调用也不经过连续模式callback包装；用自身API调用区间、DAG分支区间、成功码、尺寸及输出hash验证，不能要求每次按需调用都出现连续callback。

1. 保持 RGBD 左目640×544原字节；解除1:3帧选择后，不用GDC缩放替代输入。验证源时间戳、帧号映射、左右同步及三个新增相位。全速后的新帧自然不再与原三选一的帧集合相同，这属于交付策略变化，不能要求不同曝光时刻图片相同。
2. 在冻结的**同一批全速源帧**上重复 original/r21/compact_direct 的输入、mask与检测零差验收，再测性能。全速 SDK 与原产品跨进程数据同一性仍需配对dump，单独runner回放不能替代这项证明。
3. 检测＋分割并行、检测＋分割＋视差并行都保持 batch=1、无双缓冲、无跨帧流水线。`dag_exact_depth_parallel.json` 在输入之后并列检测、分割和`stereo.demand`三任务，三worker执行；“并行”指同帧任务/API区间重叠，不等于三个BPU内核必然同时执行。原检测/分割数学未改；新增`RunOptions.depth_parallel`只选择统计口径：AI耗时是检测/分割后处理完成端点的较大值，完整`graph_ms`覆盖三个分支，不能从重叠图耗时中直接减去视差耗时。
4. 需要真实草地、石板与混合边界；观察完整插值像素数和P95/P99，避免用全类0场景推断通用FPS上限。FPS上限分别报告固定帧计算吞吐与新鲜帧在线吞吐。
5. 关闭det/seg并不等于关闭SDK内部BPU视差。仅检测＋分割组必须证明SDK视差实际关闭且RGBD仍交付原输入，否则应报告该条件无法成立；三算法组也必须记录视差实际调用与输出，而非只看模式请求值。

代码来源：`reference/original_product/src/percep_demo.cpp`、`reference/original_product/src/dnn_runtime.cpp`、`reference/original_product/config/start_config.json`、`reference/horizon_dnn_runner.cpp`、`src/runner.cpp`、`src/legacy_preprocess.cpp`、`src/exact_input.cpp`、`src/seg_post.cpp`、`src/dag_pipeline.cpp`。数值和实现的机器可读核对见 `R27_IO_CONTRACT.json`。
