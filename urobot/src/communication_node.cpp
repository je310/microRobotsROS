
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




class communication_node{

    ros::Subscriber isInitSub;
    bool isInit= 0;
    int number_robots;
    ros::NodeHandle nh_;

public:
    communication_node(int numRobot);
    void isInitCB(const std_msgs::BoolConstPtr& msg);
};

communication_node::communication_node(int numRobot){

    number_robots = numRobot;
    ROS_INFO("I will Communicate with %d robots",number_robots);
    isInitSub= nh_.subscribe("urobot_init_node/isInit", 1,&communication_node::isInitCB,this);

    //do launching of stuff here, find all the robots and give them names. When complete call the 'publishIsInit' function. This is best to hapen only once, so lauch this node last and have others wait for this signal.
}

void communication_node::isInitCB(const std_msgs::BoolConstPtr& msg)
{
    if (msg->data == true){
        isInit =1;
    }

    //setup and launch the rest of the activity.
}

int main(int argc, char** argv) {

    //ros setup

    ros::init(argc, argv, "communication_node");
    communication_node CN(atoi(argv[1]));
    ros::spin();
    return 0;
}
