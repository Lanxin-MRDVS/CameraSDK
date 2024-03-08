#include "LxCameraRos.h"
#include <signal.h>

void MySignalhandle(int sig)
{
	ros::shutdown();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lx_camera_node", ros::init_options::NoSigintHandler);

	LxCamera camera_node;
	camera_node.run();

	ros::NodeHandle nh;
	signal(SIGINT, MySignalhandle);
	ros::spin();

	return 0;
}
