#ifndef __MO_TYPES_H
#define __MO_TYPES_H

#include <stdint.h>
#include <stdio.h>

#include "MoCommon.h"


typedef struct __MoFrame
{
    unsigned long long TimeStamp;           //time (ns) when this frame generate

    unsigned long long FrameId;             //id of this frame
    unsigned int StreamType;                //image type of this frame
    unsigned int SensorId;                  //the id of the sensor that the outpout this frame
    unsigned short ResolutionWidth;         //resolution - width of this frame
    unsigned short ResolutionHeight;        //resolution - height of this frame
    unsigned int BitsPerPixel;              //bytes per pixel

    unsigned int BytesUsed;                 //frame size in bytes

    int FillLightStatus;                    //fill light status when this frame generate                  1 on, 0 off, 2(other) unknown
    int FillLightBrightness;                //fill light bright (1%~100%) when this frame generate

    int RGBImageType;
    float exp_time;                         //sensor exposure time(in ms). Based on hbn_isp_exposure_attr
    float analog_gain;                      //sensor analog_gain. Based on hbn_isp_exposure_attr

    uint8_t Reserved[64];                   //reserved 

    void *Data;                             //image data
}MoFrame;



typedef struct __MoIMUData
{
//    uint64_t u64ImageFrameNum;  // Corresponding Image Frame number
    uint64_t Timestamp;      // ns
    uint64_t HWTimestamp;    // 160 diff

    float Temperature;  // Celsius degree

    float LinearAccelerationX;  // meter per second^2
    float LinearAccelerationY;  // meter per second^2
    float LinearAccelerationZ;  // meter per second^2

    float AngularVelocityX;  // rad per second
    float AngularVelocityY;  // rad per second
    float AngularVelocityZ;  // rad per second
}MoIMUData;
//imu data 

typedef struct __MoPoint
{
    double X;
    double Y;
    double Z;
}MoPoint;


typedef struct _mo_mono_camera_params 
{
    float fx;
    float fy;
    float cx;
    float cy;
    // TODO 相机径向畸变参数
    float radial_distortion_k[6];
    float tangential_distortion_p[2];
} MoMonoCameraParams;


typedef struct {
    float R_i2c[3][3];  // 3x3旋转矩阵 
    float T_i2c[3];  // 平移向量 [x, y, z] on meter
    float time_shift;     // t_imu = t cam + time_shift 
} Imu2LeftCameraExtrinsic; //IMU to Left Camera Extrinsic Parameter. 
                           //Point_cam = R_i2c * Point_imu + T_i2c

//Note: 其中R_i2c[3][3],需要从EEPROM中读取旋转向量v[3]，再通过Rodrigues转换为3x3的矩阵

typedef struct{
    float accelerometer_noise_density;
    float accelerometer_random_walk;
    float gyroscope_noise_density;
    float gyroscope_random_walk;
} ImuNoiseModel;

/**
 * @brief 双目基础参数
 *
 */
typedef struct _mo_binocular_camera_params {
    MoMonoCameraParams leftCamera;
    MoMonoCameraParams rightCamera;
    float rotation_r[3];
    float translation_t[3];
    uint16_t resolution_width;
    uint16_t resolution_height;
} MoBinocularCameraParams;


struct MoDepth
{
    float depth;
    uint16_t u; // horizontal line index of image, 0 based
    uint16_t v; // vertical line index of image, 0 based
};

typedef struct _mo_frame_info
{
    uint64_t frame_id;
    uint64_t timestamps;
    uint64_t sys_timestamps;
    uint64_t sif_timestamps;
    uint64_t trigger_timestamps;
    uint64_t netlink_timestamps;
    uint32_t frame_done;
    int32_t bufferindex;
} MoFrameInfo;

#endif
