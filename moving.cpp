#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <climits>
#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))

using namespace cv;
using namespace std;
const double delta = 1000;
const double phi = 0.3;
const double theta = 0.1;

boost::mutex mtx[2];
nav_msgs::Odometry g_odom;
float pre_dAngleTurned;
sensor_msgs::LaserScan g_scan;

template<typename T>
inline bool isnan(T value)
{
    return value != value;
}

tf::Transform
getCurrentTransformation(void)
{
	tf::Transform transformation;

	nav_msgs::Odometry odom;

	mtx[0].lock(); {
		odom = g_odom;
	} mtx[0].unlock();

	transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

	transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

	return transformation;
}

tf::Transform
getInitialTransformation(void)
{
	tf::Transform transformation;

	ros::Rate loopRate(100.0);

	while (ros::ok()) {
		ros::spinOnce();

		transformation = getCurrentTransformation();

		if (transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
			break;
		}
		else {
			loopRate.sleep();
		}
	}

	return transformation;
}

void odomMsgCallback(const nav_msgs::Odometry &msg)
{
    mtx[0].lock();
	{
        g_odom = msg;
    } mtx[0].unlock();
}

void scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
    mtx[1].lock();
	{
        g_scan = msg;
    } mtx[1].unlock();
}

void convertOdom2XYZRPY(nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy)
{
    xyz[0] = odom.pose.pose.position.x;
    xyz[1] = odom.pose.pose.position.y;
    xyz[2] = odom.pose.pose.position.z;

    tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3(rotationQuat).getEulerYPR(rpy[2], rpy[1], rpy[0]);
}

void convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs)
{
    int nRangeSize = (int)lrfScan.ranges.size();
    XYZs.clear();

    for(int i=0; i<nRangeSize; i++)
	{
        double dRange = lrfScan.ranges[i];
	
        if(isnan(dRange))
		{
	    XYZs.push_back(Vec3d(0., 0., 0.));

        }
		else
		{
            double dAngle = lrfScan.angle_min + i*lrfScan.angle_increment;
            if(!(-phi <= dAngle && dAngle <= phi)) continue;
	    
	    XYZs.push_back(Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.));
        }
    }
}

void saveCurrentPosition(Vec3d &xyz, vector<Vec3d> &trajectory, double dMinDist)
{
    int nSize = (int) trajectory.size();

    if(nSize <= 0) 
	{
        trajectory.push_back(xyz);
    } else {
        Vec3d diff = trajectory[nSize-1] - xyz;
        double len = sqrt(diff.dot(diff));

        if(len > dMinDist)
		{
            trajectory.push_back(xyz);
        }
    }
}

void transform(vector<Vec3d> &laserScanXY, double x, double y, double theta)
{
    Vec3d newPt;
    double cosTheta = cos(theta);
    double sinTheta = sin(theta);
    int nRangeSize = (int)laserScanXY.size();

    for(int i=0; i<nRangeSize; i++)
	{
        newPt[0] = cosTheta*laserScanXY[i][0] + -1.*sinTheta*laserScanXY[i][1] + x;
        newPt[1] = sinTheta*laserScanXY[i][0] + cosTheta*laserScanXY[i][1] + y;
        newPt[2];
        laserScanXY[i] = newPt;
    }
}

void initGrid(Mat &display, int nImageSize)
{
    const int nImageHalfSize = nImageSize/3;
    const int nAxisSize = nImageSize/8;
    const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);
    display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0]+nAxisSize, imageCenterCooord[1]), Scalar(0, 0, 255), 2);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0], imageCenterCooord[1]+nAxisSize), Scalar(0, 255, 0), 2);
}

void drawTrajectory(Mat &display, vector<Vec3d> &trajectory, double dMaxDist)
{
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);

    int nSize = (int) trajectory.size();

    for(int i=0; i<nSize; i++) {
        int x = imageHalfSize[0] + cvRound((trajectory[i][0]/dMaxDist)*imageHalfSize[0]);
        int y = imageHalfSize[1] + cvRound((trajectory[i][1]/dMaxDist)*imageHalfSize[1]);

        if(x >= 0 && x < display.cols && y >= 0 && y < display.rows)
		{
            display.at<Vec3b>(y, x) = Vec3b(0, 255, 255);
        }
    }
}

void drawCurrentPositionWithRotation(Mat &display, Vec3d &xyz, Vec3d &rpy, double dMaxDist)
{
    printf("_r = %.3lf, _p = %.3lf, _y = %.3lf\n", toDegree(rpy[0]), toDegree(rpy[1]), toDegree(rpy[2]));

    const int nHeadingSize = 30;
    Vec2i headingDir = Vec2i(nHeadingSize*cos(rpy[2]), nHeadingSize*sin(rpy[2]));
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);

    int x = imageHalfSize[0] + cvRound((xyz[0]/dMaxDist)*imageHalfSize[0]);
    int y = imageHalfSize[1] + cvRound((xyz[1]/dMaxDist)*imageHalfSize[1]);

    if(x >= 0 && x < display.cols && y >= 0 && y < display.rows)
	{
        circle(display, Point(x, y), nHeadingSize, CV_RGB(255, 0, 255), 1, CV_AA);
        line(display, Point(x, y), Point(x+headingDir[0], y+headingDir[1]), CV_RGB(255, 0, 255), 1, CV_AA);
    }
}

void drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
    int nRangeSize = (int)laserScanXY.size();

    for(int i=0; i<nRangeSize; i++)
	{
        int x = imageHalfSize[0] + cvRound((laserScanXY[i][0]/dMaxDist)*imageHalfSize[0]);
        int y = imageHalfSize[1] + cvRound((laserScanXY[i][1]/dMaxDist)*imageHalfSize[1]);

        if(x >= 0 && x < display.cols && y >= 0 && y < display.rows)
		{
            display.at<Vec3b>(y, x) = Vec3b(255, 255, 0);
        }
    }
}

void drawLRFScanMulti(Mat &display, vector< vector<Vec3d> > &laserScanXYMulti, double dMaxDist)
{
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
    int nNumOfScan = (int)laserScanXYMulti.size();

    for(int i=0; i<nNumOfScan; i++)
	{
        int nRangeSize = (int)laserScanXYMulti[i].size();

        for(int j=0; j<nRangeSize; j++)
		{
            int x = imageHalfSize[0] + cvRound((laserScanXYMulti[i][j][0]/dMaxDist)*imageHalfSize[0]);
            int y = imageHalfSize[1] + cvRound((laserScanXYMulti[i][j][1]/dMaxDist)*imageHalfSize[1]);

            if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
                display.at<Vec3b>(y, x) = Vec3b(128, 128, 128);
            }
        }
    }
}

double average(vector<Vec3d> &laserScanXY, Point cur)
{
    double ret = 0;
    
    for(int i = 0; i < laserScanXY.size(); i++)
    {
        double xdiff = laserScanXY[i][0] - cur.x;
        double ydiff = laserScanXY[i][1] - cur.y;
        ret += sqrt(xdiff * xdiff + ydiff * ydiff);
    }
    
    if(laserScanXY.size() == 0) return INT_MAX;
    return ret / laserScanXY.size();
}

bool doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.linear.y = 0.0;

    if(dRotation < 0.)
	{
        baseCmd.angular.z = -dRotationSpeed;
    }
	else
	{
        baseCmd.angular.z = dRotationSpeed;
    }

    bool bDone = false;
    ros::Rate loopRate(100.0);



    while(ros::ok() && !bDone)
	{

        ros::spinOnce();
		tf::Transform currentTransformation = getCurrentTransformation();
		tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
        tf::Quaternion rotationQuat = relativeTransformation.getRotation();
        
		double dAngleTurned = rotationQuat.getAngle();

		if( fabs(dAngleTurned) > fabs(dRotation)) 
		{
            bDone = true;
            break;
        } 
		else
		{
            pubTeleop.publish(baseCmd);
            loopRate.sleep();
        }
    }

    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    pubTeleop.publish(baseCmd);

    return bDone;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_position_lrf_view");
    ros::NodeHandle nh;

    ros::Subscriber subOdom = nh.subscribe("/odom", 100, &odomMsgCallback);
    ros::Subscriber subScan = nh.subscribe("/scan", 10, &scanMsgCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.5;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0;


    nav_msgs::Odometry odom;

    sensor_msgs::LaserScan scan;

    Vec3d xyz, rpy;

    vector<Vec3d> laserScanXY, trajectory;
    vector<vector<Vec3d> > laserScanXYMulti;

    Mat display;
    initGrid(display, 801);
    const double dGridMaxDist = 3.0;
    vector<Point> points;

    while(ros::ok())
	{
        ros::spinOnce();
        pub.publish(cmd);

        mtx[0].lock();
		{
           odom = g_odom;
        } mtx[0].unlock();

        convertOdom2XYZRPY(odom, xyz, rpy);

        saveCurrentPosition(xyz, trajectory, 0.01);

        mtx[1].lock();{
           scan = g_scan;
        } mtx[1].unlock();


        convertScan2XYZs(scan, laserScanXY);

        transform(laserScanXY, xyz[0], xyz[1], rpy[2]);

		initGrid(display, 801);
		drawTrajectory(display, trajectory, dGridMaxDist);
		drawCurrentPositionWithRotation(display, xyz, rpy, dGridMaxDist);
		drawLRFScanMulti(display, laserScanXYMulti, dGridMaxDist);
		drawLRFScan(display, laserScanXY, dGridMaxDist);

        transpose(display, display);
        flip(display, display, 0);
        flip(display, display, 1);

        imshow("moving", display);
        int nKey = waitKey(50) % 255;

        if(nKey == 27)
		{

            break;
        }
		else if(nKey == ' ')
		{
            laserScanXYMulti.push_back(laserScanXY);
        }
		else if(nKey == 'c' || nKey == 'C')
		{
            initGrid(display, 801);
            laserScanXYMulti.clear();
            trajectory.clear();
        }
        
        double avg = average(laserScanXY, Point(odom.pose.pose.position.x, odom.pose.pose.position.y));
		printf("size: %d, avg: %.2lf\n", laserScanXY.size(), avg);

		for(int i = 0; i < laserScanXY.size(); i++) printf("(%.2lf, %.2lf) ", laserScanXY[i][0], laserScanXY[i][1]); printf("\n");

		if(isnan(avg)) continue;
        if(avg > delta) continue;
	
        
        tf::Transform initialTransformation = getInitialTransformation();
        doRotation(pub, initialTransformation, theta, 0.75);

    }

    return 0;
}

