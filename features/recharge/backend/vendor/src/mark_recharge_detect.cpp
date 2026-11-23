#include "percep_demo/mark_recharge_detect.h"

namespace aiper::recharge {

cv::Mat DefaultCameraMatrix() {
  cv::Mat K = (cv::Mat_<double>(3, 3) << 718.809082031, 0.0, 626.972595215, 0.0,
               718.905212402, 551.551086426, 0.0, 0.0, 1.0);
  return K;
}

}  // namespace aiper::recharge
