#include "location_node.h"
#include <signal.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "location_node", ros::init_options::NoSigintHandler);
	LxCamera camera_node;
	camera_node.Run();
	ros::shutdown();
	return 0;
}
