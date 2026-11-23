# X5 检测/分割/点云精简 SDK：尺寸与模式可配置版（2026-11-05）

public头在include，core/adapter在src，必要Horizon运行库在runtime。SDK与主工程src同级；实现单独放完整包sdk_source。

本修订支持检测、分割分别选择resize_type=0（直接resize）或1（letterbox）。默认检测1、分割0。C API通过既有ds_model_contract.detection_resize_type / segmentation_resize_type选择；C++ adapter通过既有DnnConfig.detect/seg.resize_type选择。旧结构、ABI、成员布局及init/infer接口保持不变。两份SO需配套更新。

JSON由完整工程解析，SO不会自动读JSON：
- dnn.detect.resize_type、dnn.seg.resize_type：独立0/1选项。
- dnn.preprocess.source_width/height、detect_output_width/height、seg_output_width/height：各对0/0自动，否则需为合法偶数并与实际帧或BIN输入尺寸相符。
- seg.argmax_mode控制后处理，不是前处理模式。配置不能改变固定BIN shape。

实际模型尺寸由ds_get_model_input_size / adapter.modelInputSize查询；源尺寸和输出预期在应用侧验证。非默认预处理模式应与模型训练/部署约定匹配，不保证切模式精度相同。

原NEON/缓存数值算术未改；检测框与mask按模式还原至原图，供点云融合使用。完整配置、Docker/X5测试和ROS依赖限制见完整包根目录前处理尺寸与模式配置说明.txt。
