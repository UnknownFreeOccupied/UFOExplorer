// UFO
#include <ufoexplorer/explorer.h>

// ROS
#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ufoexplorer");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	// ros::AsyncSpinner spinner(2);
	// spinner.start();

	ufoexplorer::Explorer graph(nh, nh_priv);

	ros::spin();

	// ros::waitForShutdown();
	return 0;
}