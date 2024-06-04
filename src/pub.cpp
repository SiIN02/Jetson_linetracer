#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "dxl/dxl.hpp"
#include <memory>
#include <chrono>
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

std::string src = "nvarguscamerasrc sensor-id=0 ! \
 	video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
     format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
     width=(int)640, height=(int)360, format=(string)BGRx ! \
 	videoconvert ! video/x-raw, format=(string)BGR ! appsink";

int main(int argc, char * argv[])
{
    //cv::VideoCapture cap("line7.mp4");  //영상 파일 불러오기 영상으로 테스트할 경우

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("node_dxlpub");
    auto cam_node = std::make_shared<rclcpp::Node>("cam_pub");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("/topic_dxlpub", qos_profile );
    auto cam_pub = cam_node->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile ); //cam topic
    geometry_msgs::msg::Vector3 vel;
    vel.x = 10;
    vel.y = 10;
    vel.z = 0;

    rclcpp::WallRate loop_rate(30.0);
    
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::CompressedImage::SharedPtr msg;
    cv::VideoCapture cap(src, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open video!");
        rclcpp::shutdown();
        return -1;
    }    //입력영상 에러처리
    cv::Mat frame;

    //영상처리 복사
   
    while(rclcpp::ok())
    {
        cap >> frame;

        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
        cam_pub->publish(*msg);

        //mypub->publish(vel);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
