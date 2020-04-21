#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <arducam/image_viewer.hpp>

using std::placeholders::_1;

namespace acrobat::image_viewer {

ImageViewer::ImageViewer(const rclcpp::NodeOptions& options)
    : Node("image_viewer", options), window_name_("arducam"), image_(cv::Mat()) {
    subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/acrobat/camera", 10, std::bind(&ImageViewer::receiveImage, this, _1));
    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE); // Create a window for display.
}

ImageViewer::~ImageViewer() { cv::destroyWindow(window_name_); }

void ImageViewer::receiveImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
        image_ = cv::Mat(msg->height, msg->width, CV_8UC1, &msg->data[0]);
    } else {
        throw(cv::Exception());
    }

    cv::imshow(window_name_, image_);
    cv::waitKey(1);
}

} // namespace acrobat::image_viewer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(acrobat::image_viewer::ImageViewer)
