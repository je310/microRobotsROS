
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

#include <com_err.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "RobotClass.hpp"
#include "rosToInterface.hpp"

using namespace std;




class communication_node{
    int fd;
    ros::Subscriber isInitSub;
    ros::Subscriber instructionPush;
    vector<RobotClass*> robotInterface;
    char *portname= "/dev/ttyACM0";
    bool isInit= 0;
    int number_robots;
    ros::NodeHandle nh_;

public:
    communication_node(int numRobot);
    void isInitCB(const std_msgs::BoolConstPtr& msg);
    void instructionPushCB(const std_msgs::Int32ConstPtr& msg);
    int set_interface_attribs (int fd, int speed, int parity);
    void set_blocking (int fd, int should_block);
    void sendInstruction(instructionPack thisInstruction);
};

communication_node::communication_node(int numRobot){
    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC); //setting up serial.
    if (fd < 0)
    {
            ROS_ERROR("error %d opening %s: %s", errno, portname, strerror (errno));
            return;
    }
    set_interface_attribs (fd, B115200, 0);
    set_blocking (fd, 0);
    number_robots = numRobot;
    ROS_INFO("I will Communicate with %d robots",number_robots);
    isInitSub= nh_.subscribe("urobot_init_node/isInit", 1,&communication_node::isInitCB,this);
    instructionPush = nh_.subscribe("instructionPush",1,&communication_node::instructionPushCB,this);
    //setup and launch the rest of the activity.
    for(int i= 0; i < number_robots; i ++){
        RobotClass* newRobot = new RobotClass(nh_,i,1);
        robotInterface.push_back(newRobot);
    }
    //do launching of stuff here, find all the robots and give them names. When complete call the 'publishIsInit' function. This is best to hapen only once, so lauch this node last and have others wait for this signal.
}

void communication_node::isInitCB(const std_msgs::BoolConstPtr& msg)
{
    if (msg->data == true){
        isInit =1;
    }


}

void communication_node::instructionPushCB(const std_msgs::Int32ConstPtr& msg)
{
    ROS_INFO("pushing some data");
    int instructionType = msg->data;
    uint8_t ID = rand();
    switch (instructionType){
    case CmdLINANG:
        for(int i = 0; i < robotInterface.size(); i ++){


            instructionPack thisInstruction;
            thisInstruction.instructionType =  (uint8_t)CmdLINANG;
            thisInstruction.instructionID = ID;
            //LinAng type has a 4bit signed lin and ang put into one 8bit type; giving 8 speeds of all F/B/L/R
            geometry_msgs::Twist thisTwist = robotInterface.at(i)->getTwist();
            float lin =  robotInterface.at(i)->twistIn.linear.x;// scale a linear normalised value up to 127;
            int8_t intLin = 16*(int8_t)lin;
            float ang =  robotInterface.at(i)->twistIn.angular.z;
            int8_t intAng = 16*(int8_t)ang;
            intLin &= 0b11110000;
            intAng = intAng >> 4;
            intAng &= 0b00001111;
            int8_t result = intLin | intAng;
            thisInstruction.value1 = result;
            ///ARDUIO debug
            int8_t Alin = result & 0b11110000;
            int8_t Aang = result & 0b00001111;
            Alin = Alin >> 4;
            if(Aang & 0b00001000){ //extend the top bit to recover 2s comp.
              Aang = Aang | 0b11110000;
            }

            /////
            int8_t ardLin =result & 0b11110000;
            int8_t ardAng;
            thisInstruction.robotID = i;
            ROS_INFO("hello some data %d, %f ,%f",result,robotInterface.at(i)->twistIn.linear.x,ang);
            communication_node::sendInstruction(thisInstruction);



        }
    }

}

void communication_node::sendInstruction(instructionPack thisInstruction){
    instructionUnion myUnion;
    myUnion.pack = thisInstruction;
    write(fd,myUnion.bytes,sizeof(thisInstruction));
}

//The set_interface_attribs and set_blocking functions were liberated from user wallyk on stack overflow. http://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
int communication_node::set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                ROS_ERROR("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                ROS_ERROR("error %d from tcgetattr", errno);
                return -1;
        }
        return 0;
}

void communication_node::set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                ROS_ERROR("error %d from tcgetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                ROS_ERROR("error %d from tcgetattr", errno);
}


int main(int argc, char** argv) {

    //ros setup

    ros::init(argc, argv, "communication_node");
    communication_node CN(atoi(argv[1]));
    ros::spin();
    return 0;
}
