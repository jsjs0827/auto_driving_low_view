#ifndef __MO_EXCEPTION_H
#define __MO_EXCEPTION_H

#include <stdint.h>

#define MO_EXCEPTION_TYPE_NO_IMAGE          0x01
#define MO_EXCEPTION_TYPE_TEMPERATURE       0x02
#define MO_EXCEPTION_TYPE_LENS_OCCLISION    0x04
#define MO_EXCEPTION_TYPE_DOE_FALL          0x08

#define MO_NO_IMAGE_REASON_NO_ERR           0
#define MO_NO_IMAGE_REASON_INIT_FAIL        1
#define MO_NO_IMAGE_REASON_STREAM_ON_FAIL   2
// #define MO_NO_IMAGE_REASON_DATA_WRONG       3
// #define MO_NO_IMAGE_REASON_TIME_OUT         4
#define MO_NO_IMAGE_REASON_UNKOWN           3 



typedef struct __MoNoImageExceptionInfo
{
    int  SensorId;                  //0~3
    uint64_t StartTime;             //from 1970.1.1 in us
    int Duration;          //in s
    int Reason;
    int Reserved;
}MoNoImageExceptionInfo;

typedef struct __MoTemperaturExceptionInfo
{
    double CurTemperature;           //centigrade
    uint64_t StartTime;             //from 1970.1.1 in us
    int Duration;                    //s
    bool IfCameraSuspend;          //if the temprature is too high and continus for some time, SDK will shutdown simor and Projector
}MoTemperaturExceptionInfo;

#define MO_OCCLUSION_TYPE_UNKOWN    0
#define MO_OCCLUSION_TYPE_LEFT      1
#define MO_OCCLUSION_TYPE_RIGHT     2
#define MO_OCCLUSION_TYPE_BOTH      3

typedef struct __MoLensOcclusionExceptionInfo
{
    int SensorId;
    uint64_t StartTime;             //from 1970.1.1 in us
    int Duration;          //in s
    int Confidence;                   //3~5
    double BlockRatio;              //%
    int OcclusionType;
}MoLensOcclusionExceptionInfo;

// typedef struct __MoLowDisparityExceptionInfo
// {
//     int SensorId;
//     uint64_t StartTime;             //from 1970.1.1 in us
//     int Duration;          //in s
//     double CurDispDensity;          //%
// }MoLowDisparityExceptionInfo;

typedef struct __MoDOEFallExceptionInfo
{
    int Duration;          //in s
    uint64_t StartTime;             //from 1970.1.1 in us
    int DoeFallLevel;
}MoDOEFallExceptionInfo;

typedef struct __MoExceptionInfo
{
    int CameraHandle;
    int Exceptiontype;                  //
    void *ExceptionInfo;                //one of above exception info, according to the exception type
}MoExceptionInfo;

typedef int (*mo_exception_callback) (MoExceptionInfo *pInfo);


#endif