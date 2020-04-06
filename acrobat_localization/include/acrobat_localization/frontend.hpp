#pragma once

#include <atomic>
#include <mutex>

#include <opencv2/features2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <acrobat_localization/frame.hpp>
#include <acrobat_localization/visualizer.hpp>

namespace acrobat::localization {

enum class TrackingState { Uninitialized, Tracking, LostTracking };

class Frontend {
  public:
    using SharedPtr = std::shared_ptr<Frontend>;

    Frontend(rclcpp::Logger         logger,
             Visualizer::SharedPtr  visualizer,
             cv::Ptr<cv::Feature2D> detector);
    ~Frontend();

    template<class DetectorType>
    static SharedPtr create(rclcpp::Logger        logger,
                            Visualizer::SharedPtr visualizer,
                            cv::Ptr<DetectorType> detector = DetectorType::create());

    virtual void run();
    virtual void stop() noexcept;

    virtual void add_image(Frame::SharedPtr& frame);

  private:
    std::atomic_bool running_;
    TrackingState    state_;

    std::vector<Frame::SharedPtr> frames_;

    rclcpp::Logger logger_;

    cv::Ptr<cv::Feature2D> detector_;
    Visualizer::SharedPtr  visualizer_;
};

template<class DetectorType>
Frontend::SharedPtr Frontend::create(rclcpp::Logger        logger,
                                     Visualizer::SharedPtr visualizer,
                                     cv::Ptr<DetectorType> detector) {
    Frontend::SharedPtr frontend = std::make_shared<Frontend>(logger, visualizer, detector);
    return frontend;
}

} // namespace acrobat::localization