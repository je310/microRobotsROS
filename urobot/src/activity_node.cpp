
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <iostream>
#include <ctype.h>
#include <stdlib.h>
#include<stdio.h>
#include <math.h>



using namespace std;


// this struct should hold all the interfaces for a given robot, each node should initialise the ones that it needs to use.
struct robot{
    int ID;
    ros::Publisher linVel;
    ros::Publisher angVel;
    ros::Publisher specialAction;
    ros::Subscriber battery;
};

class activity_node{

    ros::Subscriber isInitSub;
    bool isInit= 0;
    vector<robot> robotInterface;
    int number_robots;
    ros::NodeHandle nh_;

public:
    activity_node(int robotNum);
    void isInitCB(const std_msgs::BoolConstPtr& msg);
    void initialisePubSub(vector<robot> Interface);

};

activity_node::activity_node(int robotNum){
    number_robots = robotNum;
    ROS_INFO("there are %d robots in this activity",number_robots);
    //make a vector of the right size with pub/sub for all features.
    for(int i = 0; i < number_robots; i ++ ){
        robot newRobot;
        robotInterface.push_back(newRobot);
    }
    activity_node::initialisePubSub(robotInterface);
    isInitSub= nh_.subscribe("urobot_init_node/isInit", 1,&activity_node::isInitCB,this);
    //do launching of stuff here, find all the robots and give them names. When complete call the 'publishIsInit' function. This is best to hapen only once, so lauch this node last and have others wait for this signal.

}

//in here I expect there to be lots of string forming to subscribe to the right things.
void activity_node::initialisePubSub(vector<robot> Interface){

}

void activity_node::isInitCB(const std_msgs::BoolConstPtr& msg)
{
    if (msg->data == true){
        isInit =1;
    }

    //setup and launch the rest of the activity.
}


int main(int argc, char** argv) {

    //ros setup
    ros::init(argc, argv, "activity_node");
    activity_node AN(atoi(argv[1]));
    ros::spin();
    return 0;
}
