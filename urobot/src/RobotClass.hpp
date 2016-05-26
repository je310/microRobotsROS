#ifndef ROBOT_CLASS_H
#define ROBOT_CLASS_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>
#include <string.h>
#include <sstream>


class RobotClass {
public:



    //variables
    std_msgs::Int32 leftSensor;
    std_msgs::Int32 rightSensor;
    std_msgs::Int32 backSensor;
    std_msgs::Int32 battery;
    geometry_msgs::Point position;
    geometry_msgs::TwistStamped twist;
    std_msgs::Int32 LEDState;



    //constructor
    RobotClass(ros::NodeHandle nh, int ID);

    //pub functions
    void publishTwist(geometry_msgs::TwistStamped twist);

private:
    void rightSensorCB(const std_msgs::Int32ConstPtr &msg);
    void leftSensorCB(const std_msgs::Int32ConstPtr &msg);
    void backSensorCB(const std_msgs::Int32ConstPtr &msg);
    void LEDCB(const std_msgs::Int32ConstPtr &msg);
    void positionCB(const geometry_msgs::PointConstPtr &msg);
    void batteryCB(const std_msgs::Int32ConstPtr &msg);

    // subs and pubs for all robot specific topics.
    ros::Subscriber LEDstatePub;
    ros::Publisher twistPub;
    ros::Subscriber positionSub;
    ros::Subscriber batterySub;
    ros::Subscriber leftSensorSub;
    ros::Subscriber rightSensorSub;
    ros::Subscriber backSensorSub;


};

#endif
