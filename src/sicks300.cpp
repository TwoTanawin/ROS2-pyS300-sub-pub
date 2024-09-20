#include "s300_ros2/sicks300.h"
// #include "sicks300.h"
#include <chrono>
#include <thread>

SickS300::SickS300() : Node("sick300_driver_node") {
  this->declare_parameter<std::string>("frame", "base_laser_link");
  this->declare_parameter<bool>("send_transform", true);
  this->declare_parameter<std::string>("devicename", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 115200);  // Updated baud rate to 115200

  this->get_parameter("frame", frame_id_);
  this->get_parameter("send_transform", send_transform_);
  this->get_parameter("devicename", device_name_);
  this->get_parameter("baudrate", baud_rate_);

  scan_data_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laserscan", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  connected_ = -1;
}

SickS300::~SickS300() {
  serial_comm_.disconnect();
}

void SickS300::update() {
  if (connected_ != 0) {
    RCLCPP_INFO(this->get_logger(), "Opening connection to Sick300-laser...");
    connected_ = serial_comm_.connect(device_name_, baud_rate_);
  }

  if (connected_ == 0) {
    int status = serial_comm_.readData();
    if (status == -2) {
      RCLCPP_ERROR(this->get_logger(), "Error in communication, closing connection...");
      serial_comm_.disconnect();
      connected_ = -1;
    } else if (status == 0) {
      scan_data_publisher_->publish(scan_data_);
    }
  }
}

void SickS300::broadcast_transform() {
  if (send_transform_) {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = frame_id_;
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transformStamped);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SickS300>();

  rclcpp::Rate rate(10);
  while (rclcpp::ok()) {
    node->update();
    node->broadcast_transform();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
