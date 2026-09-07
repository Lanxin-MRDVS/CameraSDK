/*************************************************************************************
 * @Description:
 * @Version: 1.0
 * @Autor: Do not edit
 * @Date: 2024-10-23 15:48:33
 * @LastEditors: Do not edit
 * @LastEditTime: 2024-10-23 17:45:40
 *************************************************************************************/
#include "lx_localization/sensor_sim.h"

SensorSim::SensorSim() : nh_("~") {
  this->nh_.param<std::string>("laser_frame_id", laser_frame_id_, "laser_link");
  this->nh_.param<std::string>("laser_topic_name", laser_topic_name_, "/scan");
  this->nh_.param<double>("min_ang", min_angle_, -2.35619449);
  this->nh_.param<double>("max_ang", max_angle_, 2.35619449);
  this->nh_.param<double>("angle_increment", increment_angle_, 0.00582);
  this->nh_.param<double>("time_increment", increment_time_, 0.00006167129);
  this->nh_.param<double>("range_min", min_range_, 0.05);
  this->nh_.param<double>("range_max", max_range_, 100);
  this->nh_.param<double>("init_range", init_range_, 100);
  init_range_size_ = int((max_range_ - min_range_) / increment_angle_ + 1);

  this->nh_.param<std::string>("odom_frame_id", odom_frame_id_, "base");
  this->nh_.param<std::string>("odom_topic_name", odom_topic_name_, "/odom");

  this->nh_.param<std::string>("laserpose_frame_id", laserpose_frame_id_,
                               "base");
  this->nh_.param<std::string>("laserpose_topic_name", laserpose_topic_name_,
                               "/pose");

  scan_publisher_ =
      nh_.advertise<sensor_msgs::LaserScan>(laser_topic_name_, 10);
  odom_publisher_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_name_, 10);
  pose_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>(laserpose_topic_name_, 10);
}

SensorSim::~SensorSim() { ROS_INFO(">>>SensorSim::~SensorSim"); }

void SensorSim::Run() {
  ros::Rate rate(30);
  while (ros::ok()) {
    PubScan();
    PubOdom();
    PubLaserPose();
    rate.sleep();
    ros::spinOnce();
  }
}

void SensorSim::PubScan() {
  sensor_msgs::LaserScan scan;
  scan.header.stamp = ros::Time::now();
  scan.header.frame_id = laser_frame_id_;
  scan.angle_increment = increment_angle_;
  scan.time_increment = increment_time_;
  scan.angle_min = min_angle_;
  scan.angle_max = max_angle_;
  scan.range_min = min_range_;
  scan.range_max = max_range_;
  scan.ranges.resize(init_range_size_, init_range_);
  scan.intensities.resize(init_range_size_, init_range_);
  scan_publisher_.publish(scan);
}

void SensorSim::PubOdom() {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = odom_frame_id_;
  odom.pose.pose.position.x = 0;
  odom.pose.pose.position.y = 0;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0;
  odom.pose.pose.orientation.w = 1;
  odom_publisher_.publish(odom);
}

void SensorSim::PubLaserPose() {
  geometry_msgs::PoseStamped laserpose;
  laserpose.header.stamp = ros::Time::now();
  laserpose.header.frame_id = laserpose_frame_id_;
  laserpose.pose.position.x = 0;
  laserpose.pose.position.y = 0;
  laserpose.pose.position.z = 0;
  laserpose.pose.orientation.x = 0;
  laserpose.pose.orientation.y = 0;
  laserpose.pose.orientation.z = 0;
  laserpose.pose.orientation.w = 1;
  pose_publisher_.publish(laserpose);
}
