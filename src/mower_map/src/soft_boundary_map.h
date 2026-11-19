#ifndef MOWER_MAP_SOFT_BOUNDARY_MAP_H
#define MOWER_MAP_SOFT_BOUNDARY_MAP_H

#include <opencv2/imgproc.hpp>

namespace mower_map {

// The images use 0 for free and 255 for occupied.
inline cv::Mat makeSoftBoundaryMap(const cv::Mat& navigation, const cv::Mat& obstacles, double resolution,
                                   double width) {
  CV_Assert(navigation.type() == CV_8UC1 && obstacles.type() == CV_8UC1);
  CV_Assert(navigation.size() == obstacles.size() && resolution > 0.0 && width > 0.0);

  cv::Mat result = navigation.clone();
  if (cv::countNonZero(navigation == 0) > 0) {
    cv::Mat distance;
    cv::distanceTransform(navigation, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    // 204/255 becomes an occupancy cost of approximately 80/100.
    result.setTo(204, (navigation != 0) & (distance <= width / resolution));
  }
  result.setTo(255, obstacles != 0);
  return result;
}

}  // namespace mower_map

#endif
