#include "lx_camera/lx_camera.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "lx_camera_node", ros::init_options::NoSigintHandler);
  LxCamera camera_node;
  ros::shutdown();
  return 0;
}
