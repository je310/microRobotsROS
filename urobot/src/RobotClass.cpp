#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>


class RobotClass {
public:

    // subs and pubs for all robot specific topics.
    ros::Subscriber LEDstatePub;
    ros::Publisher twistPub;
    ros::Subscriber positionSub;
    ros::Subscriber batterySub;
    ros::Subscriber leftSensorSub;
    ros::Subscriber rightSensorSub;
    ros::Subscriber backSensorSub;

    //variables
    std_msgs::Int32 leftSensor;
    std_msgs::Int32 rightSensor;
    std_msgs::Int32 backSensor;
    std_msgs::Int32 battery;
    geometry_msgs::Point position;
    geometry_msgs::TwistStamped twist;
    std_msgs::Int32 LEDState;




    RobotClass(ros::NodeHandle nh, int ID){
        std::string robotName("/robot");
        robotName << ID;

        std::string LEDStatePubString;
        LEDStatePubString << robotName << "/LEDState";
        LEDstatePub = nh.subscribe(LEDStatePubString,1,&RobotClass::LEDCB);

        std::string twistPubString;
        twistPubString << robotName << "/twist";
        twistPub =  nh.advertise(twistPubString,1);

        std::string positionSubString;
        positionSubString << robotName << "/position";
        positionSub = nh.subscribe(positionSubString,1,&RobotClass::positionCB);

        std::string batterySubString;
        batterySubString << robotName << "/battery";
        batterySub = nh.subscribe(batterySubString,1,&RobotClass::batteryCB);

        std::string leftSensorSubString;
        leftSensorSubString << robotName << "/leftSensor";
        leftSensorSub = nh.subscribe(leftSensorSubString,1,&RobotClass::leftSensorCB);

        std::string rightSensorSubString;
        rightSensorSubString << robotName << "/rightSensor";
        rightSensorSub = nh.subscribe(rightSensorSubString,1,&RobotClass::rightSensorCB);

        std::string backSensorSubString;
        backSensorSubString << robotName << "/backSensor";
        backSensorSub = nh.subscribe)(backSensorSubString,1, &RobotClass::backSensorCB);




    }

private:



};
