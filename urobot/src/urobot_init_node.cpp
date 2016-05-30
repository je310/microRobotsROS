/*Copyright (c) 2016 "Joshua Elsdon,"
Micro Robots Project

This file is part of Micro Robots.

Micro Robots is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.*/
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
#include "RobotClass.hpp"



using namespace std;


// this struct should hold all the interfaces for a given robot, each node should initialise the ones that it needs to use. 



class init_node{

    ros::Publisher isInitPub;
    vector<RobotClass> robotInterface;
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
    isInitPub = nh_.advertise<std_msgs::Bool>("/isInit",1,this);

    //do launching of stuff here///////////////////////////////////////////////
    for(int i= 0; i < number_robots; i ++){
        RobotClass newRobot(nh_,i,0);
        robotInterface.push_back(newRobot);
    }
    
    
    
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
