#include "knuros.h"

extern gSign;

extern double angle;
extern double distance;

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
bool parking(ros::Publisher &vel_pub_)
{
    
    geometry_msgs::Twist vel;
    ros::Rate loop_rate(500);
    bool done = true;
    while(ros::ok() && done)
    {
        
        //check gSign
        if(gSign == STOP) continue;
        
        
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
        else if(distance <= 0.5) // 장애물 앞에서 멈추는 거리 조절
        {
            done = false;
            vel.linear.x=0.0;
            vel_pub_.publish(vel);
        }
        else
        {
            vel.linear.x=0.15;
            vel.angular.z=0.0;
            vel_pub_.publish(vel);
        }
        distance=10;
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return done;
}
