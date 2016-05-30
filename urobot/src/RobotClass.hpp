#ifndef ROBOT_CLASS_H
#define ROBOT_CLASS_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>
#include <string.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

#include <cv.hpp>


class RobotClass {
public:



    //variables
    std_msgs::Int32 leftSensor;
    std_msgs::Int32 rightSensor;
    std_msgs::Int32 backSensor;
    std_msgs::Int32 battery;
    geometry_msgs::Point position;
    geometry_msgs::Twist twistIn;
    geometry_msgs::TwistStamped twistOut;
    std_msgs::Int32 LEDState;
    tf::TransformBroadcaster* br;
    int myID;




    //constructor
    RobotClass(ros::NodeHandle nh, int ID, bool shouldListen);

    //pub functions
    void publishTwist(geometry_msgs::TwistStamped twist);
    geometry_msgs::Twist getTwist();
    
    void publishTF(cv::Point2f led, cv::Point2f head, ros::Time time);
    float getAngle(cv::Point2f led, cv::Point2f head);
private:
    void rightSensorCB(const std_msgs::Int32ConstPtr &msg);
    void leftSensorCB(const std_msgs::Int32ConstPtr &msg);
    void backSensorCB(const std_msgs::Int32ConstPtr &msg);
    void LEDCB(const std_msgs::Int32ConstPtr &msg);
    void positionCB(const geometry_msgs::PointConstPtr &msg);
    void batteryCB(const std_msgs::Int32ConstPtr &msg);
    void twistCB(const geometry_msgs::TwistStampedConstPtr &msg);

    // subs and pubs for all robot specific topics.
    ros::Subscriber LEDstatePub;
    ros::Publisher twistPub;
    ros::Subscriber twistSub;
    ros::Subscriber positionSub;
    ros::Subscriber batterySub;
    ros::Subscriber leftSensorSub;
    ros::Subscriber rightSensorSub;
    ros::Subscriber backSensorSub;


};

#endif
