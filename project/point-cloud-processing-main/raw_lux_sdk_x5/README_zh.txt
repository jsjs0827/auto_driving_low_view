RAW lux1 独立 SDK

public接口：include/percep_main/raw_lux_runner.hpp；实现：src/libraw_lux.so。
主工程CMake imported target：RawLux::SDK。SDK目录与工程src同级。
本精简包不含实现CPP；可审计实现和未修改原算法/模型在交付包sdk_source/raw_lux。

固定输入合同
左右各1088(宽)x1280(高)，原生方向BGGR Bayer，未打包小端uint16；stride=2176、
每眼恰好2785280字节。DN范围0..1023，全帧除首个uint16均检查范围，首值为嵌入数据而排除。
ROI固定为[435,128,1033,1152)原始存储坐标，不可随stereo crop、RGB或GDC旋转/缩放。
不扣黑电平、不做位移/归一化、不demosaic，也不接受YUV或packed RAW10。
数据buffer在同步调用返回前必须保持存活且内容不变；两个眼不能用同一buffer代替。
SDK不按像素内容是否相同拒绝黑场左右图，但会拒同地址、缺眼和已知不同帧身份。
每对已知非零frame_id/timestamp_ns要求两眼一致；两边都0仅代表未知，不证明同步。
同session内不能复用地址/重复已知frame_id/timestamp，避免把一次capture重复当多帧。

模型和元数据
raw_hist_org_logistic（默认主模型）：必须显式配置metadata_source=org_proxy或verified_raw。
每眼exposure_us以微秒提供；原PicData/MoFrame.exp_time是ms时调用方乘1000一次。
analog_gain为线性倍率，不换算成dB。缺失/非正/NaN/Inf的必要元数据会失败，不自动换模型。
raw_only_logistic（显式备选）：metadata_source为空或not_used，不使用曝光/增益。
主模型训练来源是Org代理；verified_raw表示调用者声明输入元数据来自该RAW，原模型对这种
来源迁移尚未验证，结果metadata_source_transfer_unvalidated会为true。
stereo_same_exposure_verified始终保持原CLI的false，它不能由ID/时间戳匹配自动证明。

每眼逐帧提取P10/P50和64个归一化直方图bin。predictSession先在各眼各特征取帧中位数，
再左右等权平均，最后对曝光/增益取log2；顺序与原raw_lux1::predict完全一致。
predictPair是1对样本的predictSession，single_sample_accuracy_unvalidated=true。
predictEyeSession(group_id,left_frames,right_frames,result)保留原CLI会话可左右帧数不同的能力，
例如左2帧/右1帧；每眼各自median后左右等权平均，不按数组位置匹配或证明逐对同步。
max_session_pairs也是该API的每眼上限；身份只在同眼内去重，跨眼相同capture ID/time允许。

最小调用
  #include "percep_main/raw_lux_runner.hpp"
  perception::RawLuxConfig config;
  config.model_name = "raw_hist_org_logistic";
  config.metadata_source = "org_proxy";
  perception::RawLuxRunner runner;
  if (!runner.init(config)) { /* 处理runner.lastError() */ }
  perception::RawLuxImageView left, right;
  // 各自填写data,bytes,width,height,stride_bytes和原始身份，以及exposure_us、analog_gain。
  perception::RawLuxResult result;
  bool ok = runner.predictPair("capture_group", left, right, &result);
  // 必须先检查ok及result.valid，失败时predicted_class="UNKNOWN"并清除上一帧结果。

输出NOT_LOW_LIGHT / LOW_LIGHT / VERY_LOW_LIGHT三分类及未校准概率，不是连续lux测量。
label_definitions沿用原模型：>=10、1..<10、<1 lux，标签来自手机App1会话级弱标签。
outside_training是训练特征范围诊断，不是置信度门槛；所有原模型警示保留在warning字段。
本模块只提供测量/分类结果，接入工程不会由它直接改变地图或车辆/补光控制。

计时为CPU host steady-clock：feature_ms覆盖输入校验、全帧DN扫描和特征构造；
classification_ms覆盖会话聚合与logistic/softmax；total_ms还包括锁和结果组装。
没有BPU推理或模型文件加载。持久Runner内部串行化，调用者先结束工作线程再销毁实例。

来源与验证
原3个文件vendor/raw_lux1.hpp、models.generated.hpp、predict_raw.cpp逐字节保留，
来源/明文SHA在sdk_source/raw_lux/PROVENANCE.json。内存适配使用相同统计算术，分类直接调用原predict。
host通过177项检查：两份随包expected JSON、全部buffer/file特征、12个全幅随机RAW、
多帧与不均衡2:1会话顺序/median再log2、错误输入与元数据、20并发调用。
Docker交叉编译与X5、整节点状态以交付包根目录验证报告为准，不能把host检查当成整节点验证。

2026-11-04 等价性能优化
发布库使用neon4：NEON批量全帧DN校验、ROI连续扫描、4路uint32直方图后精确合并。
依然提取左右两眼，固定ROI完整612352像素/眼；首word排除、ROI外DN检查、模型与API不变。
X5同一真实RAW对，预热10次、各模式测100次：双目feature约21.38→5.43ms，SDK约21.45→5.49ms。
主工程和public头无需修改，替换本目录src/libraw_lux.so后按原方法调用。
原177项、生产IO164项、扩展652项检查及7组原CLI对照通过；完整ROS节点仍缺原有依赖，未声称运行通过。
详情及性能范围见完整交付根目录RAW_LUX_性能优化报告.txt。
