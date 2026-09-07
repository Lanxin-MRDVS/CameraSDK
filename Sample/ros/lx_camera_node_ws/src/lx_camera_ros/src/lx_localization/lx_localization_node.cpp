#include "lx_localization/lx_localization.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "lx_localization_node",
            ros::init_options::NoSigintHandler);
  LxLocalization lx_localization_node;
  lx_localization_node.Run();
  ros::shutdown();
  return 0;
}
