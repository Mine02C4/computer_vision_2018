#include "camera_manager.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "image_manager.hpp"

cv::Size CameraManager::default_image_size_ = cv::Size(640, 480);
fs::path CameraManager::parameter_root_ = "camera";

void CameraManager::RecordStereoVideo(std::string basename) {
  using namespace cv;
  using namespace std;
  auto path_pair = ImageManager::GetStereoVideoPath(basename);
  auto fourcc = VideoWriter::fourcc('F', 'M', 'P', '4');
  VideoWriter left_writer(get<0>(path_pair), fourcc, 15.0, default_image_size_);
  VideoWriter right_writer(get<1>(path_pair), fourcc, 15.0,
                           default_image_size_);
}

StereoCapture CameraManager::GetStereoCapture() {
  using namespace cv;
  VideoCapture left_cap, right_cap;
  left_cap.open(0);
  left_cap.set(CV_CAP_PROP_FRAME_WIDTH, default_image_size_.width);
  left_cap.set(CV_CAP_PROP_FRAME_HEIGHT, default_image_size_.height);
  right_cap.open(1);
  right_cap.set(CV_CAP_PROP_FRAME_WIDTH, default_image_size_.width);
  right_cap.set(CV_CAP_PROP_FRAME_HEIGHT, default_image_size_.height);
  return std::make_tuple(left_cap, right_cap);
}

IntrinsicsParameter::IntrinsicsParameter(std::string name) {
  path_ = CameraManager::parameter_root_ / (name + "_intrinsics.xml");
}

bool IntrinsicsParameter::ReadFile() {
  using namespace cv;
  FileStorage fs(path_.string(), FileStorage::READ);
  if (fs.isOpened()) {
    fs["M1"] >> cameraMatrix[0];
    fs["M2"] >> cameraMatrix[1];
    fs["D1"] >> distCoeffs[0];
    fs["D2"] >> distCoeffs[1];
    return true;
  } else {
    return false;
  }
}

ExtrinsicsParameter::ExtrinsicsParameter(std::string name) {
  path_ = CameraManager::parameter_root_ / (name + "_extrinsics.xml");
}

bool ExtrinsicsParameter::ReadFile() {
  using namespace cv;
  FileStorage fs(path_.string(), FileStorage::READ);
  if (fs.isOpened()) {
    fs["R"] >> R;
    fs["T"] >> T;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;
    fs["vroi0"] >> validRoi[0];
    fs["vroi1"] >> validRoi[1];
    return true;
  } else {
    return false;
  }
}
