#ifndef __MO_LOCAL_SDK_INTERFACE_H
#define __MO_LOCAL_SDK_INTERFACE_H

#include "MoCommon.h"
#include "MoTypes.h"
#include "MoException.h"

#include <stddef.h>
#include <vector>
#ifdef __cplusplus

#ifndef IN
#define IN
#endif  // IN

#ifndef OUT
#define OUT
#endif  // OUT

#ifndef INOUT
#define INOUT
#endif  // INOUT

extern "C" {
    #define DLL_API_C_DECL extern "C" __attribute__((visibility("default")))
#endif

// @brief 获取当前SDK版本信息
// @param pSdkVer 输出sdk版本号，"x.x.x.x"，x为数字，包含换行符的字符串长度为8。
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetSdkVersion(OUT char *const pSdkVer);

// @brief 设置配置文件路径
// @param cfg_path 配置文件路径
// @return 0--成功，其他--失败
DLL_API_C_DECL int moSetConfigPath(IN const char *cfg_path);



/**
 * @brief 
 * 
 * @param Handle 相机句柄
 * @param pBase 基线值
 * @param pBxF 基线*等效焦距
 * @return 0：成功从模组加载参数
 *         1：模组不支持参数获取，从X5配置文件加载参数
 *         其他：失败
 */
DLL_API_C_DECL int moGetBaseAndBxf(IN const int Handle, OUT double* const pBase, OUT double* const pBxF);



/**
 * @brief 获取参考图（矫正后输出图像）参数信息
 * 
 * @param nHandle 相机句柄
 * @param pParams 矫正图参数（畸变系数为0）
 * @param pBxf 基线*等效焦距
 * @param pBaseline 基线
 * @param pResolutionWidth 图像宽
 * @param pResolutionHeight 图像高
 * @param pDspMaxValue 最大视差值
 * @return 0：成功从模组加载参数
 *         1：模组不支持参数获取，从X5配置文件加载参数
 *         其他：失败
 */
DLL_API_C_DECL int moGetCameraParams(IN const int nHandle, OUT MoMonoCameraParams *pParams, OUT double *pBxf, OUT double *pBaseline, 
OUT int *pResolutionWidth, OUT int *pResolutionHeight, OUT int *pDspMaxValue);

// @brief 获取指定错误代码的错误描述信息
// @param ErrorCode 错误代码
// @return 错误描述信息
DLL_API_C_DECL char *moGetErrorString(IN const int ErrorCode);

// @brief 打开相机。
// @param pHandle 输出相机句柄
// @return 0--成功，其他--失败
DLL_API_C_DECL int moOpenCamera(INOUT int *pHandle);

// @brief 关闭相机。
// @param nHandle 相机句柄
// @return 0--成功，其他--失败
DLL_API_C_DECL int moCloseCamera(IN const int nHandle);

// @brief 获取原始图接口，输出宽高为1088x1280。16位raw格式。需及时调用moReleaseRawFrame() 接口对图像数据进行释放。
// @param nHandle 相机句柄
// @param pLeftFrame 输出左图，1088x1280分辨率，raw格式
// @param pRightFrame 输出右图，1088x1280分辨率，raw格式
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetOneRawFrame(IN const int nHandle, OUT MoFrame *const pLeftFrame, OUT MoFrame *const pRightFrame);

// @brief 原始图数据释放接口
// @param nHandle 相机句柄
// @param pLeftFrame 左图
// @param pRightFrame 右图
// @return 0--成功，其他--失败
DLL_API_C_DECL int moReleaseRawFrame(IN const int nHandle, IN MoFrame *const pLeftFrame, IN MoFrame *const pRightFrame);



// @brief 获取彩色原图接口，输出宽高为1280x1088。yuv420p格式。需及时调用moReleaseOrgFrame() 接口对图像数据进行释放。
// @param nHandle 相机句柄
// @param pLeftFrame 输出左图，1280x1088分辨率，yuv420p格式
// @param pRightFrame 输出右图，1280x1088分辨率，yuv420p格式
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetOneOrgFrame(IN const int nHandle, OUT MoFrame *const pLeftFrame, OUT MoFrame *const pRightFrame);

// @brief 彩色原图数据释放接口
// @param nHandle 相机句柄
// @param pLeftFrame 左图
// @param pRightFrame 右图
// @return 0--成功，其他--失败
DLL_API_C_DECL int moReleaseOrgFrame(IN const int nHandle, IN MoFrame *const pLeftFrame, IN MoFrame *const pRightFrame);


// @brief 获取矫正图接口，输出宽高为1280x1088。yuv420p格式。需及时调用moReleaseGdcFrame() 接口对图像数据进行释放。
// @param nHandle 相机句柄
// @param pLeftFrame 输出左图，1280x1088分辨率，yuv420p格式
// @param pRightFrame 输出右图，1280x1088分辨率，yuv420p格式
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetOneGdcFrame(IN const int nHandle, OUT MoFrame *const pLeftFrame, OUT MoFrame *const pRightFrame);

// @brief 矫正图数据释放接口
// @param nHandle 相机句柄
// @param pLeftFrame 左图
// @param pRightFrame 右图
// @return 0--成功，其他--失败
DLL_API_C_DECL int moReleaseGdcFrame(IN const int nHandle, IN MoFrame *const pLeftFrame, IN MoFrame *const pRightFrame);


// @brief 获取到数据后，需及时调用moReleaseRGBDFrame() 接口对图像数据进行释放。
// @param nHandle 相机句柄
// @param pLeftFrame 输出左图，640x544分辨率，yuv420p格式
// @param pRightFrame 输出右图，640x544分辨率，yuv420p格式
// @param pDispFrame 输出视差图，640x544分辨率，uint16_t类型
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetOneRGBDFrame(IN const int nHandle, OUT MoFrame *const pLeftFrame, OUT MoFrame *const pRightFrame, OUT MoFrame *const pDispFrame);

// @brief 分辨率为640x544的左右图和视差图数据释放接口
// @param nHandle 相机句柄
// @param pLeftFrame 左图
// @param pRightFrame 右图
// @param pDispFrame 视差图
// @return 0--成功，其他--失败
DLL_API_C_DECL int moReleaseRGBDFrame(IN const int nHandle, IN MoFrame *const pLeftFrame, IN MoFrame *const pRightFrame, IN MoFrame *const pDispFrame);

// @brief 根据视差图获取点云数据
// @param nHandle 相机句柄
// @param pDispFrm 视差图
// @param ptCloudVec 输出点云数据
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetPointCloudData(IN const int nHandle, IN MoFrame *pDispFrm, OUT std::vector<MoPoint>& ptCloudVec);

// @brief 根据视差图获取深度数据
// @param nHandle 相机句柄
// @param pDispFrm 视差图
// @param depthVec 输出深度数据
// @return 深度数据数量，其他--失败
DLL_API_C_DECL int moGetDepthFromDispImg(IN const int nHandle, IN MoFrame *pDispFrm, OUT std::vector<MoDepth>& depthVec);

// @brief 获取原图图像帧率
// @param nHandle 相机句柄
// @param pFrameRateOrg 输出原图图像帧率
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetFrameRateOrg(IN const int nHandle, OUT double *const pFrameRateOrg);

// @brief 获取视差图图像帧率
// @param nHandle 相机句柄
// @param pFrameRateDisp 输出视差图图像帧率
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetFrameRateDisp(IN const int nHandle, OUT double *const pFrameRateDisp);

// @brief 获取IMU是否开启
// @param nHandle 相机句柄
// @param pEnable 输出IMU是否开启
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetIMUEnable(IN const int Handle, OUT bool *const pEnable);

// @brief 设置IMU是否开启
// @param nHandle 相机句柄
// @param Enable 是否开启
// @return 0--成功，其他--失败
DLL_API_C_DECL int moSetIMUEnable(IN const int Handle, IN const bool Enable);

// @brief 获取IMU数据
// @param nHandle 相机句柄
// @param pIMUData 输出IMU数据
// @param pCount 输出IMU数据数量
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetIMUData(IN const int Handle, OUT MoIMUData* const pIMUData, OUT int *pCount);

/**
 * @brief 获取IMU到左相机的外参：旋转R_i2c、平移T_i2c以及time_shift信息
 *        Point_cam = R_i2c * Point_imu + T_i2c
 * @param Handle 
 * @param ppImuNoiseModel  
 * @return DLL_API_C_DECL 
 */
DLL_API_C_DECL int moGetImu2LeftCameraExtrinsic(IN const int Handle,OUT const Imu2LeftCameraExtrinsic**  ppImu2LeftCamExtrinsic);
/**
 * @brief 获取IMU噪声模型参数信息
 * 
 * @param Handle 
 * @param ppImuNoiseModel 
 * @return DLL_API_C_DECL 
 */
DLL_API_C_DECL int moGetImuNoiseModel(IN const int Handle,OUT const ImuNoiseModel**  ppImuNoiseModel);


/**
 * @brief 读取相机序列号
 * 
 * @param nHandle 相机句柄
 * @param ppSerailNumber 相机序列号
 * @param pSize 相机序列号长度
 * @return  0--成功，其他--失败
 */
DLL_API_C_DECL int moGetCameraSerialNumber(IN const int nHandle, OUT unsigned  const char **  ppSerailNumber, OUT int * const pSize);


/**
 * @brief 读取双目相机（原图）参数
 * 
 * @param nHandle 
 * @param ppMbcp 双目相机（原图）参数
 * @return int  0--成功，其他--失败
 */
DLL_API_C_DECL int moGetOrgBinocularCameraParameter(IN const int nHandle, OUT const MoBinocularCameraParams ** ppMbcp);

/**
 * @brief 获取NPU占用率
 * 
 * @param pRatio 输出NPU占用率
 * @return int 0--成功，其他--失败
 */
DLL_API_C_DECL int moGetBpuRatio(IN const int nHandle, OUT int32_t *pRatio);

/**
 * @brief StereoVision状态回调函数类型定义
 * @param status 状态值：1表示StereoVision_Infer开始执行，0表示执行结束
 */
typedef void (*StereoVisionStatusCallback)(int status);

/**
 * @brief 设置StereoVision状态回调函数
 * @param nHandle 相机句柄
 * @param callback 回调函数指针，传入NULL表示取消回调
 * @return 0--成功，其他--失败
 */
DLL_API_C_DECL int moGetStereoVisionStatusCallback(IN const int nHandle, IN StereoVisionStatusCallback callback);

/**
 * @brief 根据矫正图的帧号获取视差图数据
 * @param nHandle 相机句柄
 * @param frameId 帧号
 * @param pDispData 输出视差图数据
 * @param pWidth 输出视差图宽度
 * @param pHeight 输出视差图高度
 * @return 0--成功，其他--失败
 */
DLL_API_C_DECL int moCalculateDisparityByFrameId(IN const int nHandle, IN const int frameId, OUT uint16_t *pDispData, OUT int *pWidth, OUT int *pHeight);



// @brief 获取系统温度
// @param nHandle 相机句柄
// @param temperatures 输出系统温度
// @return 0--成功，其他--失败
DLL_API_C_DECL int moGetSystemTemperatures(IN const int nHandle, OUT int32_t *temperatures);
/**
 * @brief 设置视差计算工作模式
 * @param nHandle 
 * @param mode 
 *             1：关闭SDK内部计算，使能moCalculateDisparityByFrameId接口，由用户自行决定视差计算时机
 *             2：默认工作模式，由SDK内部直接计算视差
 * @return DLL_API_C_DECL 
 */
DLL_API_C_DECL int moSetDisparityCalculationMode(IN const int nHandle, IN const int mode);

/**
 * @brief 获取视差计算工作模式
 * 
 * @param nHandle 
 * @param pMode 
 * @return DLL_API_C_DECL 
 */
DLL_API_C_DECL int moGetDisparityCalculationMode(IN const int nHandle, OUT int *pMode);

#ifdef __cplusplus
}
#endif



#endif
