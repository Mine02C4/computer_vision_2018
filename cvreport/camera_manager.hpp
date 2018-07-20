#ifndef CVREPORT_CAMERA_MANAGER_H_
#define CVREPORT_CAMERA_MANAGER_H_

#include <filesystem>
#include <string>
#include <tuple>

#include <opencv2/Videoio.hpp>

namespace fs = std::filesystem;

typedef std::tuple<cv::VideoCapture, cv::VideoCapture> StereoCapture;

class CameraManager {
 public:
  CameraManager() = delete;
  static void RecordStereoVideo(std::string basename);
  static StereoCapture GetStereoCapture();
  static fs::path parameter_root_;

 private:
  static cv::Size default_image_size_;
};

class Camera {
 public:
 private:
};

class CameraParameter {
 public:
  cv::Mat cameraMatrix, distCoeffs;
  CameraParameter(std::string name);
  bool ReadFile();

 private:
  fs::path path_;
};

class StereoIntrinsicsParameter {
 public:
  cv::Mat cameraMatrix[2], distCoeffs[2];
  StereoIntrinsicsParameter(std::string name);
  bool ReadFile();
  void SetByCameraParameters(CameraParameter left, CameraParameter right);

 private:
  fs::path path_;
};

class StereoExtrinsicsParameter {
 public:
  cv::Mat R, T, R1, R2, P1, P2, Q;
  cv::Rect validRoi[2];
  StereoExtrinsicsParameter(std::string name);
  bool ReadFile();

 private:
  fs::path path_;
};

#endif  // CVREPORT_CAMERA_MANAGER_H_
