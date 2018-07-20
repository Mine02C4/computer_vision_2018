#ifndef CVREPORT_STEREO_H_
#define CVREPORT_STEREO_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "main.hpp"

class StereoTest : public SubProcedure {
 public:
  void Run();

 private:
  static std::string GetLeftImagePath(int i);
  static std::string GetRightImagePath(int i);
};

class Stereo {
 public:
  static int CalcDisparity(cv::Mat& left_img, cv::Mat& right_img,
                           cv::Mat& disparity);
  static void RenderDisparity(cv::Mat& disparity, int numberOfDisparities);
  void RenderGLWindow();
  void SetData(cv::Mat& img3d);
  Stereo();

 private:
  int gl_window_;
  cv::Mat img3d_;
};
#endif  // CVREPORT_STEREO_H_
