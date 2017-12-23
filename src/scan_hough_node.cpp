#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <limits>
using namespace ros;

class HoughSpace{
public:
	unsigned short int space[36][20]; //5deg, 10steps of log_e(rho)

	void init(void)
	{
		for(int j=0;j<20;j++)
			for(int i=0;i<36;i++)
				space[i][j] = 0;
	}

	void print(void)
	{
		for(int j=0;j<20;j++){
			for(int i=0;i<36;i++){
				std::cout << space[i][j] << ' ';
			}
			std::cout << std::endl;
		}
	}
};

HoughSpace hough;

double rho(double x, double y, double theta)
{
	return x*cos(theta) + y*sin(theta);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int step = (int)floor( ( msg->angle_max - msg->angle_min ) / msg->angle_increment );

	hough.init();
	ROS_INFO("START");

	for(int i=0;i<step;i++){
		if(std::isnan(msg->ranges[i]))
			continue;

		double ang = msg->angle_min + msg->angle_increment*step;
		double x = msg->ranges[i]*cos(ang);
		double y = msg->ranges[i]*sin(ang);

		//ROS_INFO("%f %f", x, y);
		
		for(int j=0;j<36;j++){
			double theta = 3.141592*j*5/180;
			double rho = x*cos(theta) + y*sin(theta);
			int log_rho = rho > 0 ? (int)log(rho) + 10 : (int)log(-rho);
			hough.space[j][log_rho]++;
		}
	}

	ROS_INFO("END");
	hough.print();

}

int main(int argc, char **argv)
{
	init(argc,argv,"scan_hough");
	NodeHandle n;
	Subscriber sub = n.subscribe("/scan", 1, scanCallback);

	ros::spin();
}
