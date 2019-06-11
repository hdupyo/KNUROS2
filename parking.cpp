#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

double angle;
double distance=10; // detect range
double spd = 0.15;

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	for (unsigned int i=0;i< scan_msg->ranges.size();i++)
	{   
		if (scan_msg->ranges[i]<distance)
		{
			distance=scan_msg->ranges[i];  
			angle = ((double)i*scan_msg->angle_increment)+scan_msg->angle_min;
		}
		std::cout << distance <<" ------ "<< angle << std::endl;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "parking");
	ros::NodeHandle n;
	ros::Publisher vel_pub_=n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Subscriber scan_sub = n.subscribe("scan", 10, scanCallback);
	geometry_msgs::Twist vel;
	ros::Rate loop_rate(500);
	bool done = true;
	while(n.ok() && done)
	{
		if (distance >= 0.7 && distance <= 0.9)
		{
			if (angle <= 0.0)
			{
				vel.linear.x=0.2;
				vel.angular.z=0.6;
				vel_pub_.publish(vel);
			}
			else
			{
				if (angle>0)
				{
					vel.linear.x=0.2;
					vel.angular.z=-0.6;
					vel_pub_.publish(vel);
				}
			}
		}
		else if(distance <= 0.5)
		{
			done = false;
			vel.linear.x=0.0;
			vel_pub_.publish(vel);
		}
		else
		{
			vel.linear.x=spd;
			vel.angular.z=0.0;
			vel_pub_.publish(vel);
		}
		distance=10;     
		ros::spinOnce(); 
		loop_rate.sleep();
	}
	return 0;
}
