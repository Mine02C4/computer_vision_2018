#include "camera_manager.hpp"

#include <opencv2/highgui/highgui.hpp>

#include "image_manager.hpp"

cv::Size CameraManager::DefaultImageSize = cv::Size(640, 480);

void CameraManager::RecordStereoVideo(std::string basename) {
  using namespace cv;
  using namespace std;
  auto path_pair = ImageManager::GetStereoVideoPath(basename);
  auto fourcc = VideoWriter::fourcc('F', 'M', 'P', '4');
  VideoWriter left_writer(get<0>(path_pair).string(), fourcc, 15.0,
                          DefaultImageSize);
  VideoWriter right_writer(get<1>(path_pair).string(), fourcc, 15.0,
                           DefaultImageSize);
}

StereoCapture CameraManager::GetStereoCapture() {
  using namespace cv;
  VideoCapture left_cap, right_cap;
  left_cap.open(0);
  left_cap.set(CV_CAP_PROP_FRAME_WIDTH, DefaultImageSize.width);
  left_cap.set(CV_CAP_PROP_FRAME_HEIGHT, DefaultImageSize.height);
  right_cap.open(1);
  right_cap.set(CV_CAP_PROP_FRAME_WIDTH, DefaultImageSize.width);
  right_cap.set(CV_CAP_PROP_FRAME_HEIGHT, DefaultImageSize.height);
  return std::make_tuple(left_cap, right_cap);
}
