#include "acrobat_localization/visual_odometry.hpp"
#include <acrobat_localization/frame.hpp>
#include <acrobat_localization/map_point.hpp>
#include <acrobat_localization/visual_odometry.hpp>
#include <gtsam/geometry/Point2.h>
#include <memory>

namespace acrobat::localization {
MapPoint::MapPoint(const rclcpp::Time&            stamp,
                   const std::shared_ptr<Frame>&  frame,
                   size_t                         idx,
                   const NoiseModel::shared_ptr&  meas_noise,
                   const Calibration::shared_ptr& camera_cal)
    : last_observation_timestamp_(stamp),
      last_reprojection_error_(0.0),
      need_recompute_descriptor(true) {
    factor_                    = boost::make_shared<SmartFactor>(meas_noise, camera_cal);
    static size_t map_point_id = 0;
    factor_graph_idx_          = VisualOdometry::backend_->graph_->size();
    VisualOdometry::backend_->graph_->add(factor_);
    map_point_id++;
}

MapPoint::~MapPoint() {}

void MapPoint::add_observation(const std::shared_ptr<Frame>& frame, size_t index) {
    // Add measurement to the smart factor
    // After all measurements are added to smart factor add to graph
    // (will the smart factor ever be added to the graph already, will we need to
    //  re-add it?)
    const auto pt = frame->keypoints().at(index).pt;
    factor_->add(gtsam::Point2(pt.x, pt.y), gtsam::symbol('x', frame->frame_id_));
}

void MapPoint::release() { VisualOdometry::backend_->graph_->remove(factor_graph_idx_); }

} // namespace acrobat::localization