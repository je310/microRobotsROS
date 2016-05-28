#include "RobotClass.hpp"

RobotClass::RobotClass(ros::NodeHandle nh, int ID,bool shouldListen){
    std::stringstream robotName;
    robotName << "/robot"<< ID;

    std::stringstream  LEDStatePubString;
    LEDStatePubString << robotName.str() << "/LEDState";
    LEDstatePub = nh.subscribe(LEDStatePubString.str(),1,&RobotClass::LEDCB,this);

    std::stringstream  twistPubString;
    twistPubString << robotName.str() << "/twist";
    twistPub =  nh.advertise<geometry_msgs::TwistStamped>(twistPubString.str(),1,this);
    //if(shouldListen)
        twistSub = nh.subscribe(twistPubString.str(),1,&RobotClass::twistCB,this);

    std::stringstream  positionSubString;
    positionSubString << robotName.str() << "/position";
    positionSub = nh.subscribe(positionSubString.str(),1,&RobotClass::positionCB,this);

    std::stringstream  batterySubString;
    batterySubString << robotName.str() << "/battery";
    batterySub = nh.subscribe(batterySubString.str(),1,&RobotClass::batteryCB,this);

    std::stringstream  leftSensorSubString;
    leftSensorSubString << robotName.str() << "/leftSensor";
    leftSensorSub = nh.subscribe(leftSensorSubString.str(),1,&RobotClass::leftSensorCB,this);

    std::stringstream  rightSensorSubString;
    rightSensorSubString << robotName.str() << "/rightSensor";
    rightSensorSub = nh.subscribe(rightSensorSubString.str(),1,&RobotClass::rightSensorCB,this);

    std::stringstream  backSensorSubString;
    backSensorSubString << robotName.str() << "/backSensor";
    backSensorSub = nh.subscribe(backSensorSubString.str(),1, &RobotClass::backSensorCB,this);
}
void RobotClass::rightSensorCB(const std_msgs::Int32ConstPtr &msg){
    rightSensor = *msg;
}

void RobotClass::leftSensorCB(const std_msgs::Int32ConstPtr &msg){
    leftSensor = *msg;
}
void RobotClass::backSensorCB(const std_msgs::Int32ConstPtr &msg){
    backSensor = *msg;
}
void RobotClass::LEDCB(const std_msgs::Int32ConstPtr &msg){
    LEDState = *msg;
}

void RobotClass::positionCB(const geometry_msgs::PointConstPtr &msg){
    position = *msg;
}

void RobotClass::batteryCB(const std_msgs::Int32ConstPtr &msg){
    battery = *msg;
}
void RobotClass::publishTwist(geometry_msgs::TwistStamped twist){
    RobotClass::twistPub.publish(twist);
}
void RobotClass::twistCB(const geometry_msgs::TwistStampedConstPtr &msg){
    twistIn = msg->twist;
     ROS_INFO("in class %d",this);
}

geometry_msgs::Twist RobotClass::getTwist(){
    return twistIn;
}

