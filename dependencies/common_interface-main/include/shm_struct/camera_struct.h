#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#define WIDTH 640
#define HEIGHT 544
#define PIC_SIZE (WIDTH*HEIGHT*3/2)
#define DISP_SIZE (WIDTH*HEIGHT)

inline constexpr const char *kCameraFrameShm = "daye_shm_camera_frame";
inline constexpr const int kFrameShmSize = (2 * 1024 *1024);

#define GDC_WIDTH 1280
#define GDC_HEIGHT 1088
#define GDC_PIC_SIZE (GDC_WIDTH*GDC_HEIGHT*3/2)

#define RAW_WIDTH 1280
#define RAW_HEIGHT 1088
#define RAW_PIC_SIZE (RAW_WIDTH*RAW_HEIGHT)

inline constexpr const char *kCameraRawFrameShm = "daye_shm_camera_raw_frame";
inline constexpr const int kRawFrameShmSize = (3 * 1024 *1024);

inline constexpr const char *kCameraGdcFrameShm = "daye_shm_camera_gdc_frame";
inline constexpr const int kGdcFrameShmSize = (5 * 1024 *1024);

inline constexpr const char *kCameraImuShm = "daye_shm_camera_imu_data";
inline constexpr const int kCameraImuShmSize = 1024;

#pragma pack(push, 1)

typedef struct {
  uint64_t time_ns;      // time (ns) when this frame generate
  uint64_t frame_id;     // id of this frame
  uint32_t stream_type;  // image type of this frame
  uint32_t sensor_id;    // the id of the sensor that the outpout this frame
  uint16_t width;        // resolution - width of this frame
  uint16_t height;       // resolution - height of this frame
  uint32_t bits_per_pixel; // bytes per pixel
  uint32_t bytes_used;     //frame size in bytes
  int32_t  filllight_status;  //fill light status when this frame generate   1 on, 0 off, 2(other) unknown
  int32_t  filllight_brightness;  //fill light bright (1%~100%) when this frame generate
  int32_t  rgd_image_type;
  float exp_time;           //sensor exposure time(in ms). Based on hbn_isp_exposure_attr
  float analog_gain;        //sensor analog_gain. Based on hbn_isp_exposure_attr
  int64_t cur_time_us;     // Timestamp when getting images from the SDK
  uint8_t reserved[56];
} PicData;

typedef struct {
  PicData left;
  uint16_t left_data[RAW_PIC_SIZE];
  // PicData right;
  // uint16_t right_data[RAW_PIC_SIZE];
} CameraRawData;

typedef struct {
  PicData left;
  uint8_t left_data[PIC_SIZE];
  PicData right;
  uint8_t right_data[PIC_SIZE];
  PicData disp;
  uint16_t disp_data[DISP_SIZE];
  uint64_t device_ns;
} CameraData;


typedef struct {
  PicData left;
  uint8_t left_data[GDC_PIC_SIZE];
  PicData right;
  uint8_t right_data[GDC_PIC_SIZE];
} CameraGdcData;


typedef struct {
  uint64_t time_ns;      // ns
  uint64_t hw_timestamp;    // 160 diff
  float temperature;  // Celsius degree

  float linear_acceleration_x;  // meter per second^2
  float linear_acceleration_y;  // meter per second^2
  float linear_acceleration_z;  // meter per second^2

  float angular_velocity_x;  // rad per second
  float angular_velocity_y;  // rad per second
  float angular_velocity_z;  // rad per second
} CameraImuData;

#pragma pack(pop)
