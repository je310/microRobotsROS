
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
#include <vector>



using namespace std;


// this struct should hold all the interfaces for a given robot, each node should initialise the ones that it needs to use. 
struct robot{
    int ID; 
    ros::Publisher linVel;
    ros::Publisher angVel;
    ros::Publisher specialAction;
    ros::Subscriber battery;
};


class init_node{

    ros::Publisher isInitPub;
    vector<robot> robotInterface;
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
    
    //make a vector of the right size with pub/sub for all features. 
    for(int i = 0; i < number_robots; i ++ ){
        robot newRobot;
        robotInterface.push_back(newRobot);
    }
    isInitPub= nh_.advertise<std_msgs::Bool>("urobot_init_node/isInit", 1);

    //do launching of stuff here///////////////////////////////////////////////
    
    
    
    
    //end launching stuff//////////////////////////////////////////////////////
    publishIsInit();

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
