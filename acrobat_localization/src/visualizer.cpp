#include <acrobat_localization/visualizer.hpp>

namespace acrobat::localization {

Visualizer::~Visualizer() { cv::destroyWindow(window_name_); }

Visualizer::SharedPtr Visualizer::create(rclcpp::Logger logger, std::string window_name) {
    Visualizer::SharedPtr viz = std::make_shared<Visualizer>(logger, window_name);
    return viz;
}

Visualizer::Visualizer(rclcpp::Logger logger, std::string window_name)
    : logger_(logger), window_name_(window_name) {
    cv::namedWindow(window_name_);
}

void Visualizer::display_frame(const Frame::SharedPtr& frame) const {
    cv::Mat image_with_keypoints;
    cv::drawKeypoints(frame->image(), frame->keypoints(), image_with_keypoints, {0.0, 255.0, 0.0});
    cv::imshow(window_name_, image_with_keypoints);
    cv::waitKey(1);
}

} // namespace acrobat::localization
