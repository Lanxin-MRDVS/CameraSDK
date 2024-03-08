#include "LxCameraRos.h"
#include <chrono>
#include <memory>
#include <iostream>
using namespace std;
using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LxCamera>());

	return 0;
}

