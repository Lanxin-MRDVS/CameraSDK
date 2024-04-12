/*************************************************************************************
 * Description:
 * Version:
 * Autor:
 * Date: 2022-08-16 10:34:48
 * @LastEditors: Do not edit
 * @LastEditTime: 2024-03-09 13:21:03
 *************************************************************************************/

#ifndef __SENSOR_SIM_H__
#define __SENSOR_SIM_H__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>

class SensorSim {
public:
  SensorSim();
  ~SensorSim();
  void Run();

private:
  void PubScan();
  void PubOdom();
  void PubLaserPose();

private:
  ros::NodeHandle nh_;
  ros::Publisher scan_publisher_;
  ros::Publisher odom_publisher_;
  ros::Publisher pose_publisher_;

  std::string laser_frame_id_;
  std::string laser_topic_name_;
  double min_angle_;
  double max_angle_;
  double increment_angle_;
  double increment_time_;
  double min_range_;
  double max_range_;
  double init_range_;
  int init_range_size_;

  std::string odom_frame_id_;
  std::string odom_topic_name_;

  std::string laserpose_frame_id_;
  std::string laserpose_topic_name_;
};

#endif
