/*************************************************************************************
 * Description:
 * Version:
 * Autor:
 * Date: 2022-08-16 10:53:11
 * @LastEditors: Do not edit
 * @LastEditTime: 2024-03-09 13:20:55
 *************************************************************************************/

#include "sensor_sim.h"
using namespace std;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorSim>());
  return 0;
}