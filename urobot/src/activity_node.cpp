
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <iostream>
#include <ctype.h>
#include <stdlib.h>
#include<stdio.h>
#include <math.h>
#include <unistd.h>

#include "RobotClass.hpp"
#include "rosToInterface.hpp"

using namespace std;


// this struct should hold all the interfaces for a given robot, each node should initialise the ones that it needs to use.


class activity_node{

    ros::Subscriber isInitSub;
    ros::Subscriber joySub;
    ros::Publisher instructionPush;
    bool isInit= 0;
    vector<RobotClass> robotInterface;
    int number_robots;
    ros::NodeHandle nh_;

public:
    activity_node(int robotNum);
    void isInitCB(const std_msgs::BoolConstPtr& msg);
    void joyCB(const sensor_msgs::JoyConstPtr& msg);

};

activity_node::activity_node(int robotNum){
    number_robots = robotNum;
    ROS_INFO("there are %d robots in this activity",number_robots);
    //make a vector of the right size with pub/sub for all features.
    instructionPush = nh_.advertise<std_msgs::Int32>("/instructionPush",1,this);
    joySub = nh_.subscribe("/joy",1,&activity_node::joyCB,this);
    isInitSub= nh_.subscribe("urobot_init_node/isInit", 1,&activity_node::isInitCB,this);
    //do launching of stuff here, find all the robots and give them names. When complete call the 'publishIsInit' function. This is best to hapen only once, so lauch this node last and have others wait for this signal.
    for(int i= 0; i < number_robots; i ++){
        RobotClass newRobot(nh_,i,0);
        robotInterface.push_back(newRobot);
    }

}

//in here I expect there to be lots of string forming to subscribe to the right things.


void activity_node::isInitCB(const std_msgs::BoolConstPtr& msg)
{
    if (msg->data == true){
        isInit =1;
    }

    //setup and launch the rest of the activity.
}

//here should do something nice to take controll of all the buttons.
void activity_node::joyCB(const sensor_msgs::JoyConstPtr& msg){
    static float oldLin = 0;
    static float oldAng = 0;
    geometry_msgs::Vector3 linear;
    linear.x = msg->axes.at(1);
    geometry_msgs::Vector3 angular;
    angular.z = msg->axes.at(0);
    if(oldLin!=linear.x || oldAng != angular.z){
        for(int i = 0; i < robotInterface.size(); i++){
            geometry_msgs::TwistStamped thisTwist;
            ROS_INFO("%d , %f",0,linear.x);
            ROS_INFO("%d , %f",1,angular.z );
            thisTwist.twist.linear = linear;
            thisTwist.twist.angular = angular;
            thisTwist.header.stamp = msg->header.stamp;
            //robotInterface.at(i).twistOut = thisTwist;
            robotInterface.at(i).publishTwist(thisTwist);
            ros::spinOnce();
        }
        ros::spinOnce();
        std_msgs::Int32 pushType;
        pushType.data =CmdLINANG;
        oldLin = linear.x;
        oldAng = angular.z;
        instructionPush.publish(pushType);
    }
    ros::spinOnce();
}


int main(int argc, char** argv) {

    //ros setup
    ros::init(argc, argv, "activity_node");
    activity_node AN(atoi(argv[1]));
    ros::spin();
    return 0;
}
