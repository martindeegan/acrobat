#include "acrobat_localization/map_point.hpp"
#include <acrobat_localization/visualizer.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

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

void Visualizer::display_frame(const Frame::SharedPtr&  frame,
                               std::vector<cv::DMatch>& matches,
                               std::vector<bool>&       valid) const {
    cv::Mat image_with_keypoints;
    cv::cvtColor(frame->image(), image_with_keypoints, cv::COLOR_GRAY2RGB);

    for (size_t i = 0; i < frame->keypoints().size(); i++) {
        cv::Scalar color;
        if (valid[i]) {
            color = {0.0, 255.0, 0.0};
        } else {
            color = {255.0, 0.0, 0.0};
        }
        cv::circle(image_with_keypoints, frame->keypoints()[i].pt, 3, color);
    }
    cv::imshow(window_name_, image_with_keypoints);
    cv::waitKey(1);
}

} // namespace acrobat::localization
