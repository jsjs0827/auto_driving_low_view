#ifndef __MO_COMMON_H
#define __MO_COMMON_H

#define MO_MAX_SN_LEN           16
#define MO_MAX_DEV_NAME_LEN     256
#define MO_MAX_CAMERA_NUM       16

#define MO_MAX_IMU_CNT_IN_QUEUE  128

#define MO_MAX_SENSOR_NUM_IN_CAMERA     4
// #define MO_MAX_VIDEO_DEVICE_IN_CAM      4

#define MO_STREAM_TYPE_RGB_IMAGE                  0x0001        // the normal RGB CMOS output, RGB888 bpp 24 
#define MO_STREAM_TYPE_STEREO_RGBD                0x0010        // stereo cmos output, Disparity + YUV420, 24 bpp
#define MO_STREAM_TYPE_STEREO_ORIGIN_RGBD         0x0080        // stereo cmos output, original disparity + YUV_I420, 24 bpp

//sensor stream upload enable
#define MO_SENSOR0_DATA_UPLOAD_ENABLE   0x01
#define MO_SENSOR1_DATA_UPLOAD_ENABLE   0x02
#define MO_SENSOR2_DATA_UPLOAD_ENABLE   0x04
#define MO_SENSOR3_DATA_UPLOAD_ENABLE   0x08


#define MO_MAX_USB_DEV_NUM       127
#define MO_MAX_FRAMES_BUF        8
#define MO_MAX_STREAM_INFO_NUM   4

#define MO_ILLEGAL_HANDLE       -1

//function return value
#define MO_REV_OK               0
#define MO_REV_DEV_NAME_ERR     -1
#define MO_REV_PARAM_ERR        -2
#define MO_REV_CAM_NOT_WORKING  -3
#define MO_REV_BE_LOCKED         -4
#define MO_REV_DEV_OPERATION_FAIL   -5
#define MO_REV_DEV_DATA_WRONG    -6
#define MO_REV_DEV_NO_SAVED_DATA    -7
#define MO_REV_BUF_ALLOC_FAIL    -8
#define MO_REV_NOT_SUPPORT      -9
#define MO_REV_NOT_FOUND        -10
#define MO_REV_NOT_READY        -11
#define MO_REV_STATUS_ERR       -12
#define MO_REV_TOO_MANY_DEV     -13
#define MO_REV_OTHER_ERR        -14
#define MO_REV_NO_NEED_UPGRADE  -15
#define MO_REV_NOT_INIT        -16
#define MO_REV_SHUTDOWN         -20
#define MO_REV_ISP_NOT_WORK     -21
#define MO_REV_EEPROM_RESOLUTION -22   //  分辨率
#define MO_REV_EEPROM_RECOVERY_OK -23   //  分辨率
#define MO_REV_DISP_CALC_DISABLED -24   //  视差计算对外接口被禁用
#define MO_REV_FRAME_ID_NOT_FOUND -25   //  帧号未找到
#define MO_REV_FRAME_ID_EXPIRED -26     //  帧号已过期
#define MO_REV_DISP_CALC_CONFLICT -27   //  视差计算冲突（内部计算已开启）


//fill light type
#define MO_FT_OFF    1                       // Fill light  always off
#define MO_FT_EXPOSURE_SYNC      2           // Fill light synchronizes with exposure
#define MO_FT_ON_OFF_ALTERNATION 3           // Fill light alternates between on and off

// fill light status 
#define MO_FILL_LIGHT_STATUS_OFF                   0
#define MO_FILL_LIGHT_STATUS_ON                    1
#define MO_FILL_LIGHT_STATUS_UNKNOWN               2

//external trigger mode
#define MO_EXTERN_TRIGGER_MODE_NONE                 0               
#define MO_EXTERN_TRIGGER_MODE_MASTER               1
#define MO_EXTERN_TRIGGER_MODE_SLAVER               2

#define MO_BACKUP_EPR_TO_BOAD_PATH         "/home/metoak"
#endif