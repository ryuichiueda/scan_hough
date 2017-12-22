#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
using namespace ros;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
        //on = msg->mid_toggle;
}

int main(int argc, char **argv)
{
	init(argc,argv,"scan_hough");
	NodeHandle n;
	Subscriber sub = n.subscribe("/scan", 1, scanCallback);
}
