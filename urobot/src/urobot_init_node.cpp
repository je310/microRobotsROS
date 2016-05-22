
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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




class init_node{

    ros::Publisher isInitPub;

    int number_robots;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
public:
    init_node(int numRobots);
    void publishIsInit();
};

init_node::init_node(int numRobots): it_(nh_){
    number_robots = numRobots;
    ROS_INFO("I will initialise %d robots",number_robots);
    isInitPub= nh_.advertise<std_msgs::Bool>("urobot_init_node/isInit", 1);

    //do launching of stuff here, find all the robots and give them names. When complete call the 'publishIsInit' function. This is best to hapen only once, so lauch this node last and have others wait for this signal.

}
void init_node::publishIsInit()
{
    std_msgs::Bool initMsg;
    initMsg.data = true;
    isInitPub.publish(initMsg);
}

int main(int argc, char** argv) {

    //ros setup

    ros::init(argc, argv, "urobot_init_node");
    init_node IN(atoi(argv[1]));
    ros::spin();
    return 0;
}