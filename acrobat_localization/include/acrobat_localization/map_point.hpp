#pragma once

#include <Eigen/Dense>
#include <boost/smart_ptr/make_shared_array.hpp>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

namespace acrobat::localization {

class Frame;

class MapPoint {
  public:
    using SharedPtr   = std::shared_ptr<MapPoint>;
    using Calibration = gtsam::Cal3Fisheye;
    using SmartFactor = gtsam::SmartProjectionPoseFactor<Calibration>;
    using NoiseModel  = gtsam::noiseModel::Isotropic;

    MapPoint(const rclcpp::Time&            stamp,
             const std::shared_ptr<Frame>&  frame,
             size_t                         idx,
             const NoiseModel::shared_ptr&  meas_noise,
             const Calibration::shared_ptr& camera_cal);
    ~MapPoint();

  private:
  public:
    // Averaged descriptors
    cv::Mat descriptor_;

    // A map of the observations
    // Key is the keyframe which the point is observed from
    // Value is the index in that keyframe
    std::unordered_map<std::shared_ptr<Frame>, size_t> observations_;

    // 3d position of the point in world space
    Eigen::Vector3d         position_;
    SmartFactor::shared_ptr factor_;

    rclcpp::Time last_observation_timestamp_;
    double       last_reprojection_error_;

    size_t factor_graph_idx_;

    bool need_recompute_descriptor;

    void add_observation(const std::shared_ptr<Frame>& frame, size_t index);

    void release();

    size_t map_point_id_;
    size_t first_frame_id_;
    size_t last_frame_id_;

    bool mark_erase;
    bool mark_deproject;

    void add_to_graph();
};

} // namespace acrobat::localization