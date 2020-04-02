#pragma once

#include <deque>

#include <acrobat_common/lie_groups/lie_groups.hpp>

namespace acrobat::localization {

class ImuPropagator {
  public:
    ImuPropagator();

    void add_measurement(lie_groups::SE3::Tangent&& measurement);

  private:
    std::deque<lie_groups::SE3>          poses_;
    std::deque<lie_groups::SE3>          relative_poses_;
    std::deque<lie_groups::SE3::Tangent> imu_samples_;
};

} // namespace acrobat::localization