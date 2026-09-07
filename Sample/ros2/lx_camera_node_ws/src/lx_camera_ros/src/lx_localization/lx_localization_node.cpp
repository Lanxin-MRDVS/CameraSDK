#include "lx_localization/lx_localization.h"

int main(int argc, char **argv) {
  DcLib lib;
  if (!DynamicLink(&lib))
    return 0;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LxLocalization>(&lib));
  DisDynamicLink(&lib);

  return 0;
}
