#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
using namespace ros;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
        //on = msg->mid_toggle;
	ROS_INFO("%f", msg->angle_min);
}

int main(int argc, char **argv)
{
	init(argc,argv,"scan_hough");
	NodeHandle n;
	Subscriber sub = n.subscribe("/scan", 1, scanCallback);

	ros::spin();
}
