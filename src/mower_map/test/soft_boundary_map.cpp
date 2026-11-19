#include "../src/soft_boundary_map.h"

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>
#include <gtest/gtest.h>

TEST(SoftBoundaryMap, LimitsBandAndKeepsObstacleCellsLethal) {
  cv::Mat navigation(11, 11, CV_8UC1, cv::Scalar(255));
  navigation.at<unsigned char>(5, 5) = 0;
  cv::Mat obstacles = cv::Mat::zeros(navigation.size(), CV_8UC1);
  obstacles.at<unsigned char>(5, 5) = 255;
  obstacles.at<unsigned char>(5, 6) = 255;

  cv::Mat result = mower_map::makeSoftBoundaryMap(navigation, obstacles, 0.05, 0.10);
  EXPECT_EQ(255, result.at<unsigned char>(5, 5));
  EXPECT_EQ(255, result.at<unsigned char>(5, 6));
  EXPECT_EQ(204, result.at<unsigned char>(5, 7));
  EXPECT_EQ(204, result.at<unsigned char>(6, 6));
  EXPECT_EQ(255, result.at<unsigned char>(5, 8));
  EXPECT_EQ(255, result.at<unsigned char>(7, 7));
}

TEST(SoftBoundaryMap, DoesNotOpenAnEmptyMap) {
  cv::Mat navigation(5, 5, CV_8UC1, cv::Scalar(255));
  cv::Mat obstacles = cv::Mat::zeros(navigation.size(), CV_8UC1);
  cv::Mat result = mower_map::makeSoftBoundaryMap(navigation, obstacles, 0.05, 1.0);
  EXPECT_EQ(25, cv::countNonZero(result == 255));
}

TEST(SoftBoundaryMap, FreeAreaRemainsFree) {
  cv::Mat navigation(5, 5, CV_8UC1, cv::Scalar(0));
  cv::Mat obstacles = cv::Mat::zeros(navigation.size(), CV_8UC1);
  cv::Mat result = mower_map::makeSoftBoundaryMap(navigation, obstacles, 0.05, 0.1);
  EXPECT_EQ(0, cv::countNonZero(result));
}

TEST(SoftBoundaryMap, FootprintAcceptsSoftCostButRejectsLethalObstacle) {
  costmap_2d::Costmap2D costmap(20, 20, 0.1, 0.0, 0.0, costmap_2d::FREE_SPACE);
  base_local_planner::CostmapModel model(costmap);
  std::vector<geometry_msgs::Point> footprint(4);
  footprint[0].x = -0.2;
  footprint[0].y = -0.2;
  footprint[1].x = 0.2;
  footprint[1].y = -0.2;
  footprint[2].x = 0.2;
  footprint[2].y = 0.2;
  footprint[3].x = -0.2;
  footprint[3].y = 0.2;
  costmap.setCost(12, 10, 203);
  EXPECT_GE(model.footprintCost(1.05, 1.05, 0.0, footprint, 0.2, 0.3), 0.0);
  costmap.setCost(12, 10, costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(-1.0, model.footprintCost(1.05, 1.05, 0.0, footprint, 0.2, 0.3));
}

TEST(SoftBoundaryMap, FootprintFitsNarrowVirtualCorridorButNotObstacle) {
  cv::Mat navigation(31, 31, CV_8UC1, cv::Scalar(255));
  navigation.rowRange(13, 18).setTo(0);
  cv::Mat obstacles = cv::Mat::zeros(navigation.size(), CV_8UC1);
  const cv::Mat softened = mower_map::makeSoftBoundaryMap(navigation, obstacles, 0.05, 0.3);
  costmap_2d::Costmap2D costmap(31, 31, 0.05, 0.0, 0.0, costmap_2d::FREE_SPACE);
  for (int y = 0; y < navigation.rows; ++y) {
    for (int x = 0; x < navigation.cols; ++x) {
      costmap.setCost(x, y, navigation.at<unsigned char>(y, x) == 255 ? costmap_2d::LETHAL_OBSTACLE : 0);
    }
  }
  base_local_planner::CostmapModel model(costmap);
  std::vector<geometry_msgs::Point> footprint(4);
  footprint[0].x = -0.3;
  footprint[0].y = -0.2;
  footprint[1].x = 0.3;
  footprint[1].y = -0.2;
  footprint[2].x = 0.3;
  footprint[2].y = 0.2;
  footprint[3].x = -0.3;
  footprint[3].y = 0.2;
  EXPECT_EQ(-1.0, model.footprintCost(0.775, 0.775, 0.0, footprint, 0.2, 0.4));
  for (int y = 0; y < softened.rows; ++y) {
    for (int x = 0; x < softened.cols; ++x) {
      const auto value = softened.at<unsigned char>(y, x);
      costmap.setCost(x, y, value == 255 ? costmap_2d::LETHAL_OBSTACLE : value == 204 ? 203 : 0);
    }
  }
  EXPECT_GE(model.footprintCost(0.775, 0.775, 0.0, footprint, 0.2, 0.4), 0.0);
  costmap.setCost(15, 11, costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(-1.0, model.footprintCost(0.775, 0.775, 0.0, footprint, 0.2, 0.4));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
