#pragma once

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

namespace acrobat::localization {

class KeyFrame;

class MapPoint {
  public:
    MapPoint();
    ~MapPoint();

  private:
    cv::Mat                              descriptor_;
    std::unordered_map<KeyFrame, size_t> observations_;
    Eigen::Vector3d                      position_;

    rclcpp::Time last_observation_timestamp_;
    double       last_reporjection_error_;

    bool is_candidate;
};

} // namespace acrobat::localization