//
//  main.cpp
//  opencv_project
//
//  Created by 김만기 on 28/04/2019.
//  Copyright © 2019 김만기. All rights reserved.
//

#include "knuros.h"

using namespace std;
using namespace cv;

MGSign gSign;

double angle;
double distance=10; // detect range

boost::mutex mtx[2];
nav_msgs::Odometry g_odom;
float pre_dAngleTurned;
sensor_msgs::LaserScan g_scan;

int main(int argc, const char * argv[])
{
    ros::init(argc, argv, "happy");
    ros::NodeHandle nh;
    
    // setting for open_cv dectection
    image_transport::ImageTranspot it(nh);
    image_transport::Subscriber subRGB = it.subscribe("/raspicam_node/image", 1, &postMessageReceivedRGB, ros::VoidPtr(), image_transport::TransportHints("compressed"));
    
    
    // setting for parking
    ros::Publisher vel_pub_=nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);
    
    
    // setting for autodriving
    ros::Subscriber subOdom = nh.subscribe("/odom", 50, &odomMsgCallback);
    ros::Subscriber subScan = nh.subscribe("/scan", 10, &scanMsgCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    
    ros::Rate loop_rate(500);
    
    
    
    /* main code */
    
    // 2. if found PARKING_AREA, break
    // if(gSign == PARKING_AREA) return;
    
    // 3. auto race to find PARKING_SIGN
    autodriving();
    
    // 4. if found PARKING_SIGN, break
    // if(gSign == PARKING_SIGN) return;
    
    // 5. parking function
    parking(vel_pub_);
   
    
    ros::spin();
    
    return 0;
}
