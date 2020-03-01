#include "arducam/image_viewer.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

using std::placeholders::_1;

ImageViewer::ImageViewer(const rclcpp::NodeOptions& options)
    : Node("image_viewer", options), image_(cv::Mat()) {
    subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "/acrobat/camera", 10, std::bind(&ImageViewer::receiveImage, this, _1));
}

void ImageViewer::receiveImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
        image_ = cv::Mat(msg->height, msg->width, CV_8UC1, &msg->data[0]);
    } else {
        throw(cv::Exception());
    }

    cv::namedWindow("Arducam", cv::WINDOW_AUTOSIZE); // Create a window for display.
    cv::imshow("Arducam", image_);
    cv::waitKey(1);
}