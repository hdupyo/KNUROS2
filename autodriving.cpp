#include "knuros.h"

using namespace cv;
using namespace std;

extern gSign;

extern boost::mutex mtx[2];
extern nav_msgs::Odometry g_odom;
extern float pre_dAngleTurned;
extern sensor_msgs::LaserScan g_scan;

template<typename T>
bool isnan(T value)
{
    return value != value;
}

void
odomMsgCallback(const nav_msgs::Odometry &msg)
{
    mtx[0].lock(); {
        g_odom = msg;
    } mtx[0].unlock();
}

void
scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
    mtx[1].lock(); {
        g_scan = msg;
    } mtx[1].unlock();
}

void
convertOdom2XYZRPY(nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy)
{
    xyz[0] = odom.pose.pose.position.x;
    xyz[1] = odom.pose.pose.position.y;
    xyz[2] = odom.pose.pose.position.z;
    
    tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Matrix3x3(rotationQuat).getEulerYPR(rpy[2], rpy[1], rpy[0]);
}

void
convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs)
{
    int nRangeSize = (int)lrfScan.ranges.size();
    XYZs.clear();
    
    for(int i=0; i<nRangeSize; i++) {
        double dRange = lrfScan.ranges[i];
        
        if(isnan(dRange)) {
            XYZs.push_back(Vec3d(0., 0., 0.));
            
        } else {
            double dAngle = lrfScan.angle_min + i*lrfScan.angle_increment;
            if(!(-phi <= dAngle && dAngle <= phi)) continue;
            
            XYZs.push_back(Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.));
        }
    }
}

void
saveCurrentPosition(Vec3d &xyz, vector<Vec3d> &trajectory, double dMinDist)
{
    int nSize = (int) trajectory.size();
    
    if(nSize <= 0) {
        trajectory.push_back(xyz);
    } else {
        Vec3d diff = trajectory[nSize-1] - xyz;
        double len = sqrt(diff.dot(diff));
        
        if(len > dMinDist) {
            trajectory.push_back(xyz);
        }
    }
}

void
transform(vector<Vec3d> &laserScanXY, double x, double y, double theta)
{
    Vec3d newPt;
    double cosTheta = cos(theta);
    double sinTheta = sin(theta);
    int nRangeSize = (int)laserScanXY.size();
    
    for(int i=0; i<nRangeSize; i++) {
        newPt[0] = cosTheta*laserScanXY[i][0] + -1.*sinTheta*laserScanXY[i][1] + x;
        newPt[1] = sinTheta*laserScanXY[i][0] + cosTheta*laserScanXY[i][1] + y;
        newPt[2];
        laserScanXY[i] = newPt;
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
    
    ros::Rate loopRate(2000.0);
    
    while(ros::ok()) {
        ros::spinOnce();
        
        transformation = getCurrentTransformation();
        
        if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
            break;
        } else {
            loopRate.sleep();
        }
    }
    
    return transformation;
}
bool
doRotation(ros::Publisher &pubTeleop, tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.linear.y = 0.0;
    
    if(dRotation < 0.) {
        baseCmd.angular.z = -dRotationSpeed;
    } else {
        baseCmd.angular.z = dRotationSpeed;
    }
    
    bool bDone = false;
    ros::Rate loopRate(2000.0);
    
    
    
    while(ros::ok() && !bDone) {
        
        ros::spinOnce();
        
        tf::Transform currentTransformation = getCurrentTransformation();
        
        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;
        tf::Quaternion rotationQuat = relativeTransformation.getRotation();
        
        
        
        double dAngleTurned = rotationQuat.getAngle();
        
        if( fabs(dAngleTurned) > fabs(dRotation))
        {
            bDone = true;
            break;
        } else {
            pubTeleop.publish(baseCmd);
            loopRate.sleep();
        }
    }
    
    baseCmd.linear.x = 0.0;
    baseCmd.angular.z = 0.0;
    pubTeleop.publish(baseCmd);
    
    return bDone;
}


void autodriving()
{
    geometry_msgs::Twist cmd, stop_cmd;
    cmd.linear.x = 0.3; stop_cmd.linear.x = 0;
    cmd.linear.y = 0.0; stop_cmd.linear.y = 0;
    cmd.angular.z = 0; stop_cmd.angular.z = 0;
    
    
    nav_msgs::Odometry odom;
    
    sensor_msgs::LaserScan scan;
    
    Vec3d xyz, rpy;
    
    vector<Vec3d> laserScanXY, trajectory;
    vector<vector<Vec3d> > laserScanXYMulti;
    
    const double dGridMaxDist = 3.0;
    vector<Point> points;
    
    while(ros::ok()) {
        ros::spinOnce();
        
        // check gSign
        if(gSign == STOP)
        {
            pub.publish(stop_cmd);
            continue;
        }
        
        pub.publish(cmd);
        
        mtx[0].lock(); {
            odom = g_odom;
        } mtx[0].unlock();
        
        convertOdom2XYZRPY(odom, xyz, rpy);
        
        saveCurrentPosition(xyz, trajectory, 0.001);
        
        mtx[1].lock(); {
            scan = g_scan;
        } mtx[1].unlock();
        
        
        convertScan2XYZs(scan, laserScanXY);
        
        transform(laserScanXY, xyz[0], xyz[1], rpy[2]);
        
        
        double avg = average(laserScanXY, Point(odom.pose.pose.position.x, odom.pose.pose.position.y));
        
        for(int i = 0; i < laserScanXY.size(); i++) printf("(%.2lf, %.2lf) ", laserScanXY[i][0], laserScanXY[i][1]); printf("\n");
        
        if(isnan(avg)) continue;
        if(avg > delta) continue;
        
        
        tf::Transform initialTransformation = getInitialTransformation();
        doRotation(pub, initialTransformation, theta, 0.75);
        
    }
}
