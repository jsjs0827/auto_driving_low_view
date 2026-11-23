# RTK+VSLAM initialization state.
uint8 INIT_NONE=0
uint8 INIT_RTK_NOT_FIXED=1
uint8 INIT_RTK_FIXED_READY=2
uint8 INIT_INITIALIZING=3
uint8 INIT_FINISHED=4

# RTK+VSLAM fusion quality.
uint8 FUSION_INVALID=0
uint8 FUSION_DEGRADED=1
uint8 FUSION_NORMAL=2
uint8 FUSION_GOOD=3

# sensor_status error bit masks. A cleared bit means that sensor is healthy.
uint8 SENSOR_GNSS_ERROR=1
uint8 SENSOR_VISION_ERROR=2
uint8 SENSOR_ODOMETER_ERROR=4
uint8 SENSOR_M720_IMU_ERROR=8
uint8 SENSOR_S3162_IMU_ERROR=16

# gnss status.
uint8 GNSS_NONE=0
uint8 GNSS_NORAML=1
uint8 GNSS_GODD=2

uint8 init_status
uint8 fusion_status
uint8 sensor_status
uint8 gnss_status
# sat_num: the number of satellites used in GNSS.
uint8 sat_num
