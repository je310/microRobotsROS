#include "RobotClass.hpp"

RobotClass::RobotClass(ros::NodeHandle nh, int ID){
    std::stringstream robotName;
    robotName << "/robot"<< ID;

    std::stringstream  LEDStatePubString;
    LEDStatePubString << robotName.str() << "/LEDState";
    LEDstatePub = nh.subscribe(LEDStatePubString.str(),1,&RobotClass::LEDCB,this);

    std::stringstream  twistPubString;
    twistPubString << robotName.str() << "/twist";
    twistPub =  nh.advertise<geometry_msgs::TwistStamped>(twistPubString.str(),1,this);

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
    RobotClass::rightSensor = *msg;
}

void RobotClass::leftSensorCB(const std_msgs::Int32ConstPtr &msg){
    RobotClass::leftSensor = *msg;
}
void RobotClass::backSensorCB(const std_msgs::Int32ConstPtr &msg){
    RobotClass::backSensor = *msg;
}
void RobotClass::LEDCB(const std_msgs::Int32ConstPtr &msg){
    RobotClass::LEDState = *msg;
}

void RobotClass::positionCB(const geometry_msgs::PointConstPtr &msg){
    RobotClass::position = *msg;
}

void RobotClass::batteryCB(const std_msgs::Int32ConstPtr &msg){
    RobotClass::battery = *msg;
}
void RobotClass::publishTwist(geometry_msgs::TwistStamped twist){
    RobotClass::twistPub.publish(twist);
}
