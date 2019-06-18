//
//  knuros.h
//  opencv_project
//
//  Created by 김만기 on 12/06/2019.
//  Copyright © 2019 김만기. All rights reserved.
//

#ifndef knuros_h
#define knuros_h

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>

#include <boost/thread/mutex.hpp>
#include <climits>

#define toRadian(degree)    ((degree) * (M_PI / 180.))
#define toDegree(radian)    ((radian) * (180. / M_PI))

using namespace std;
using namespace cv;

enum MGColor {RED, GREEN, BLUE, NOCOLOR};
enum MGSign {STOP, GO, PARKING_AREA, PARKING_SIGN, NIL};

const double delta = 1;
const double phi = 0.3;
const double theta = 0.1;

// recognition functions
double angle(Point pt1, Point pt2, Point pt0);
void find_polygon(Mat image, vector<vector<Point> >& squares, vector<vector<Point> >& triangles);
void find_largest_triangle(const vector<vector<Point> >& triangles, vector<Point>& biggest_triangle);
void find_largest_square(const vector<vector<Point> >& squares, vector<Point>& biggest_square);
Rect crop_img(Mat& img, vector<Point>& largest_square);
MGColor get_color(Mat& img);
MGSign get_sign(Mat& img);
void draw_recognition_display(vector<vector<Point> >& squares, vector<vector<Point> >& triangles, Mat& img);
void postMessageRecievedRGB(const sensor_msgs::ImageConstPtr& msg);

//autodriving
template<typename T>
bool isnan(T value);
void odomMsgCallback(const nav_msgs::Odometry &msg);
void scanMsgCallback(const sensor_msgs::LaserScan& msg);
void convertOdom2XYZRPY(nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy);
void convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs);
void saveCurrentPosition(Vec3d &xyz, vector<Vec3d> &trajectory, double dMinDist);
void transform(vector<Vec3d> &laserScanXY, double x, double y, double theta);
double average(vector<Vec3d> &laserScanXY, Point cur);
tf::Transform getCurrentTransformation(void);
tf::Transform getInitialTransformation(void);
bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed);
void autodriving(ros::Publisher &pub);

//parking
void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
bool parking(ros::Publisher &vel_pub_);

#endif
