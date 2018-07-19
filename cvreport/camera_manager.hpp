#ifndef CVREPORT_CAMERA_MANAGER_H_
#define CVREPORT_CAMERA_MANAGER_H_

#include <string>
#include <tuple>

#include <opencv2/Videoio.hpp>

typedef std::tuple<cv::VideoCapture, cv::VideoCapture> StereoCapture;

class CameraManager {
 public:
  CameraManager() = delete;
  static void RecordStereoVideo(std::string basename);
  static StereoCapture GetStereoCapture();
 private:
  static cv::Size DefaultImageSize;
};

#endif  // CVREPORT_CAMERA_MANAGER_H_
