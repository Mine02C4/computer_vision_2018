#ifndef CVREPORT_CALIBRATION_H_
#define CVREPORT_CALIBRATION_H_

#include <vector>
#include <opencv2/core/core.hpp>

#include "main.hpp"

class SingleCalibration : public SubProcedure
{
public:
	void Run();
private:
};

class Calibration
{
public:
  static bool DetectChessboardCorners(cv::Mat& image, cv::Size board_size, std::vector<cv::Point2f>& image_corners);
private:
};

#endif  // CVREPORT_CALIBRATION_H_
