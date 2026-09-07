#include "lx_localization/sensor_sim.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_sim_node");
  SensorSim sensor_sim_node;
  sensor_sim_node.Run();
  ros::shutdown();
  return 0;
}
