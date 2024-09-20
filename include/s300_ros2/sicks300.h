#ifndef __SICKS300_H__
#define __SICKS300_H__

#include "serialcomm_s300.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class SickS300 : public rclcpp::Node {
public:
  SickS300();
  ~SickS300();

  void update();
  void broadcast_transform();

private:
  SerialCommS300 serial_comm_;
  sensor_msgs::msg::LaserScan scan_data_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_data_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string frame_id_;
  bool send_transform_;
  std::string device_name_;
  unsigned int baud_rate_;
  int connected_;
};

#endif // __SICKS300_H__
