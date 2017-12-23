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

	void set(double x, double y)
	{
		for(int i=0;i<36;i++){
			double theta = 3.141592*i*5/180;
			double rho = x*cos(theta) + y*sin(theta);
			if(rho >= 1.0){
				int log_rho = (int)log(rho) + 10;
				space[i][log_rho]++;
			}else if(rho <= -1.0){
				int log_rho = 10 - (int)log(-rho);
				space[i][log_rho]++;
			}else
				space[i][0]++;

		}
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

	unsigned long int compare(HoughSpace *ref)
	{
		unsigned long int score = 0;
		for(int j=0;j<20;j++){
			for(int i=0;i<36;i++){
				int diff = space[i][j] - ref->space[i][j];
				score += diff*diff;
			}
		}
		return score;
	}
};

HoughSpace ref;
HoughSpace latest;


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	static bool first = true;
	int step = (int)floor( ( msg->angle_max - msg->angle_min ) / msg->angle_increment );

	HoughSpace &hough = first ? ref : latest;

	hough.init();

	ROS_INFO("START");

	for(int i=0;i<step;i+=2){
		if(std::isnan(msg->ranges[i]))
			continue;

		double ang = msg->angle_min + msg->angle_increment*step;
		double x = msg->ranges[i]*cos(ang)*1000;
		double y = msg->ranges[i]*sin(ang)*1000;
		hough.set(x, y);
	}

	if(not first){
		unsigned long int s = ref.compare(&hough);
		ROS_INFO("END %f",(double)s);
	}

	first = false;
}

int main(int argc, char **argv)
{
	init(argc,argv,"scan_hough");
	NodeHandle n;
	Subscriber sub = n.subscribe("/scan", 1, scanCallback);

	ros::spin();
}
