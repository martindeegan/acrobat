#include <acrobat_common/time/rate.hpp>
#include <acrobat_localization/frontend.hpp>

using namespace std::chrono_literals;

namespace acrobat::localization {

Frontend::Frontend(rclcpp::Logger         logger,
                   Visualizer::SharedPtr  visualizer,
                   cv::Ptr<cv::Feature2D> detector)
    : running_(true),
      state_(TrackingState::Uninitialized),
      logger_(logger),
      detector_(detector),
      visualizer_(visualizer) {}

Frontend::~Frontend() {}

void Frontend::run() {
    time::Rate loop_rate(100);
    while (rclcpp::ok() && running_) { loop_rate.sleep(); }
}

void Frontend::stop() noexcept { running_ = false; }

void Frontend::add_image(Frame::SharedPtr& frame) {
    frame->extract_features(detector_);
    if (visualizer_) { visualizer_->display_frame(frame); }
    frame->release_image();

    frames_.push_back(frame);
}

} // namespace acrobat::localization
