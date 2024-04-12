/*************************************************************************************
 * Description:
 * Version:
 * Autor:
 * Date: 2022-08-16 10:34:57
 * @LastEditors: Do not edit
 * @LastEditTime: 2024-03-09 13:20:47
 *************************************************************************************/

#include "sensor_sim.h"

SensorSim::SensorSim() : Node("sensor_sim_node") {
 this->declare_parameter<std::string>("laser_frame_id", "laser_link");
 this->declare_parameter<std::string>("laser_topic_name", "/scan");
 this->declare_parameter<double>("min_ang", -2.35619449);
 this->declare_parameter<double>("max_ang", 2.35619449);
 this->declare_parameter<double>("angle_increment", 0.00582);
 this->declare_parameter<double>("time_increment", 0.00006167129);
 this->declare_parameter<double>("range_min", 0.05);
 this->declare_parameter<double>("range_max", 100);
 this->declare_parameter<double>("init_range", 100);
 this->declare_parameter<std::string>("odom_frame_id", "base");
 this->declare_parameter<std::string>("odom_topic_name", "/odom");
 this->declare_parameter<std::string>("laserpose_frame_id", "base");
 this->declare_parameter<std::string>("laserpose_topic_name",  "/pose");

 this->get_parameter<std::string>("laser_frame_id", laser_frame_id_);
 this->get_parameter<std::string>("laser_topic_name", laser_topic_name_);
 this->get_parameter<double>("min_ang", min_angle_);
 this->get_parameter<double>("max_ang", max_angle_);
 this->get_parameter<double>("angle_increment", increment_angle_);
 this->get_parameter<double>("time_increment", increment_time_);
 this->get_parameter<double>("range_min", min_range_);
 this->get_parameter<double>("range_max", max_range_);
 this->get_parameter<double>("init_range", init_range_);
 this->get_parameter<std::string>("odom_frame_id", odom_frame_id_);
 this->get_parameter<std::string>("odom_topic_name", odom_topic_name_);
 this->get_parameter<std::string>("laserpose_frame_id", laserpose_frame_id_);
 this->get_parameter<std::string>("laserpose_topic_name", laserpose_topic_name_);

  init_range_size_ = int((max_range_ -  min_range_) / increment_angle_ + 1);

  scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(laser_topic_name_, 10);
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 10);
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(laserpose_topic_name_, 10);
}

SensorSim::~SensorSim() {  }

void SensorSim::Run() {
    rclcpp::Node::SharedPtr node_(this);
  while (rclcpp::ok()) {
    PubScan();
    PubOdom();
    PubLaserPose();
    rclcpp::spin_some(node_);
  }
}

void SensorSim::PubScan() {
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = rclcpp::Clock().now();
  scan.header.frame_id = laser_frame_id_;
  scan.angle_increment = increment_angle_;
  scan.time_increment = increment_time_;
  scan.angle_min = min_angle_;
  scan.angle_max = max_angle_;
  scan.range_min = min_range_;
  scan.range_max = max_range_;
  scan.ranges.resize(init_range_size_, init_range_);
  scan.intensities.resize(init_range_size_, init_range_);
  scan_publisher_->publish(scan);
}

void SensorSim::PubOdom() {
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rclcpp::Clock().now();
  odom.header.frame_id = odom_frame_id_;
  odom.pose.pose.position.x = 0;
  odom.pose.pose.position.y = 0;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0;
  odom.pose.pose.orientation.w = 1;
  odom_publisher_->publish(odom);
}

void SensorSim::PubLaserPose() {
  geometry_msgs::msg::PoseStamped laserpose;
  laserpose.header.stamp = rclcpp::Clock().now();
  laserpose.header.frame_id = laserpose_frame_id_;
  laserpose.pose.position.x = 0;
  laserpose.pose.position.y = 0;
  laserpose.pose.position.z = 0;
  laserpose.pose.orientation.x = 0;
  laserpose.pose.orientation.y = 0;
  laserpose.pose.orientation.z = 0;
  laserpose.pose.orientation.w = 1;
  pose_publisher_->publish(laserpose);
}
