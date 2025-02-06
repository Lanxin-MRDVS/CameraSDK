#include "sensor_sim.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorSim>());
  return 0;
}
