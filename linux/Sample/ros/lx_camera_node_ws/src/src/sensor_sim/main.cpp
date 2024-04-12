/*************************************************************************************
 * Description:
 * Version:
 * Autor:
 * Date: 2022-08-16 10:53:11
 * @LastEditors: Do not edit
 * @LastEditTime: 2024-03-09 13:20:55
 *************************************************************************************/

#include "sensor_sim.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_sim_node");
  SensorSim sensor_sim_node;
  sensor_sim_node.Run();
  ros::shutdown();
  return 0;
}