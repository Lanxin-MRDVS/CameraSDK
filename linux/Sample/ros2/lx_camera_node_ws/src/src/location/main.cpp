#include <chrono>
#include <memory>
#include <iostream>
#include "location_node.h"
using namespace std;
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
	DcLib lib;
	if(!DynamicLink(&lib))return 0;
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Location>(&lib));
	DisDynamicLink(&lib);
	
	return 0;
}
