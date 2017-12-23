#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <limits>
using namespace ros;

class HoughSpace{
public:
	short int space[36][10]; //5deg, 10steps of log_e(rho)
};

HoughSpace hough;

double rho(double x, double y, double theta)
{
	return x*cos(theta) + y*sin(theta);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int step = (int)floor( ( msg->angle_max - msg->angle_min ) / msg->angle_increment );

	for(int i=0;i<step;i++){
		if(std::isnan(msg->ranges[i]))
			continue;

		double ang = msg->angle_min + msg->angle_increment*step;
		double x = msg->ranges[i]*cos(ang);
		double y = msg->ranges[i]*sin(ang);

		//ROS_INFO("%f %f", x, y);
		
		for(i=0;i<36;i++){
		}
	}

}

int main(int argc, char **argv)
{
	init(argc,argv,"scan_hough");
	NodeHandle n;
	Subscriber sub = n.subscribe("/scan", 1, scanCallback);

	ros::spin();
}
