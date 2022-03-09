// ROS
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ufoexplorer_trajector");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	std::string source_frame = nh_priv.param<std::string>("source_frame", "base_link");
	std::string target_frame = nh_priv.param<std::string>("target_frame", "map");
	ros::Duration timeout(nh_priv.param("tf_timeout", 0.1));

	ros::Publisher path_pub = nh_priv.advertise<nav_msgs::Path>("trajectory", 10, true);

	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener(tf_buffer);

	nav_msgs::Path path;
	path.header.frame_id = target_frame;

	ros::Rate rate(nh_priv.param("rate", 1.0));
	while (nh.ok()) {
		path.header.stamp = ros::Time::now();
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = source_frame;
		pose.header.stamp = path.header.stamp;
		pose.pose.orientation.w = 1.0;
		if (tf_buffer.canTransform(target_frame, source_frame, pose.header.stamp, timeout)) {
			pose = tf_buffer.transform(pose, target_frame);
			path.poses.push_back(pose);
			path_pub.publish(path);
		}
		rate.sleep();
	}

	return 0;
}