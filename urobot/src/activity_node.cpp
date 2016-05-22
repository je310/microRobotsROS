
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




class activity_node{

    ros::Subscriber isInitSub;
    bool isInit= 0;
    int number_robots;
    ros::NodeHandle nh_;

public:
    activity_node(int robotNum);
    void isInitCB(const std_msgs::BoolConstPtr& msg);

};

activity_node::activity_node(int robotNum){
    number_robots = robotNum;
    ROS_INFO("there are %d robots in this activity",number_robots);
    isInitSub= nh_.subscribe("urobot_init_node/isInit", 1,&activity_node::isInitCB,this);
    //do launching of stuff here, find all the robots and give them names. When complete call the 'publishIsInit' function. This is best to hapen only once, so lauch this node last and have others wait for this signal.

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
