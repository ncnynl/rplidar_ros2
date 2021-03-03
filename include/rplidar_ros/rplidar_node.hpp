/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef RPLIDAR_ROS__RPLIDAR_ROS_HPP_
#define RPLIDAR_ROS__RPLIDAR_ROS_HPP_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include "rplidar.h"

namespace rplidar_ros
{

class RPlidarNode : public rclcpp::Node
{
public:
  explicit RPlidarNode(
    const std::string & name = "rplidar_node",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~RPlidarNode();

  bool get_device_info();

  bool check_health();

  void stop_motor(
    const std::shared_ptr<rmw_request_id_t> request_header,
    std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response);

  void start_motor(
    const std::shared_ptr<rmw_request_id_t> request_header,
    std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response);

  float get_angle(const rplidar_response_measurement_node_hq_t & node);

  void spin();

private:
  std::unique_ptr<rp::standalone::rplidar::RPlidarDriver> driver_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_server_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_server_;

  rclcpp::Clock clock_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string channel_type_;

  std::string tcp_ip_;

  int64_t tcp_port_;

  std::string serial_port_;

  int64_t serial_baudrate_;

  std::string frame_id_;

  bool inverted_;

  bool angle_compensate_;

  double max_distance_;

  std::string scan_mode_;

  int angle_compensate_multiple_;

  void declare_parameters();

  void get_parameters();

  void connect_driver();

  void check_scan_mode();

  void publish_scan(
    rplidar_response_measurement_node_hq_t * nodes,
    size_t node_count, rclcpp::Time & start, double scan_time,
    float angle_min, float angle_max);
};

}  // namespace rplidar_ros

#endif  // RPLIDAR_ROS__RPLIDAR_ROS_HPP_
