#ifndef CVREPORT_RECOGNITION_H_
#define CVREPORT_RECOGNITION_H_

#include <vector>
#include <tuple>

#include <opencv2/features2d/features2d.hpp>

typedef std::tuple<std::vector<cv::KeyPoint>, cv::Mat> FeatureDescription;

class Recognition {
public:
  static void DescribeFeatures(cv::Mat &image, FeatureDescription &fd);
  static void MatchKeyPoints(
    FeatureDescription &fd1,
    FeatureDescription &fd2,
    std::vector<cv::DMatch> &good_matches,
    std::vector<cv::Point2f> &match_point1,
    std::vector<cv::Point2f> &match_point2
  );
  static void DetectSkin(cv::Mat &hsv, cv::Mat &mask);
  static void DetectTargetBlue(cv::Mat &hsv, cv::Mat &mask);
private:
};

#endif  // CVREPORT_RECOGNITION_H_
