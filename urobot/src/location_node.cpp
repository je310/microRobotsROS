
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <iostream>
#include <ctype.h>
#include <stdlib.h>
#include<stdio.h>
#include <math.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "RobotClass.hpp"


using namespace std;
using namespace cv;


// this struct should hold all the interfaces for a given robot, each node should initialise the ones that it needs to use.




class location_node{
    struct pairs{
        int head;
        int led;
        float error;

    };
    struct kPoints{
        Point2f head;
        Point2f led;
        float error;
    };
    struct uiButton{
        Point2i botL;
        Point2i topR;
    };
    ros::Subscriber isInitSub;
    bool isInit= 0;
    static const int numberFedu = 4;
    cv::Point2f camWobble;  // stored so that we can search a min bounding box for feducials. then Min bounding boxes are more possible for the robots.
    int isSetup = 0; // are we happy with the thresholds and the wobble perameters.
    vector<RobotClass*> robotInterface;
    int number_robots;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber camera;
    cv::Mat interface;
    uiButton leftBut;
    uiButton rightBut;
    uiButton midBut;
     Point2i fiducialPoints[numberFedu];



    int interfaceState = 1;

    int FLowH = 0;
    int FHighH = 15;

    int FLowS = 158;
    int FHighS = 255;

    int FLowV = 97;
    int FHighV = 175;

    int LLowH = 4;
    int LHighH = 97;

    int LLowS = 0;
    int LHighS = 55;

    int LLowV = 225;

    int LHighV = 255;

    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    cv::Mat image;

    int radius = 36; // should be initialised to the number of pixels between the front marker and the led.

    bool debug = 1;

    int fiduSearch = 50;
    int robotSearch = 50;

public:
    location_node(int robotNum, char* argin);
    void isInitCB(const std_msgs::BoolConstPtr& msg);
    void joyCB(const sensor_msgs::JoyConstPtr& msg);
    void imageCB(const sensor_msgs::ImageConstPtr& msg);
    vector<pairs> findPairs(std::vector<cv::KeyPoint> Fkeypoints,std::vector<cv::KeyPoint>  Lkeypoints,float radius,int &best);
    float findDistance(cv::Point2f A,cv::Point2f B);
    void thesholdImage(cv::Mat &imageIn,bool needHead,std::vector<cv::KeyPoint> &Fkeypoints,bool needLED,std::vector<cv::KeyPoint> &Lkeypoints );
    static void CallBackFunc(int event, int x, int y, int flags, void* userdata);
    void doMouseCallback(int event, int x, int y, int flags);
    bool isIn(uiButton but, int x, int y);
    void loadScreen(int screenNumber);
    Rect getSafeRect (Point2i point, int size, Mat &im);
};

void location_node::CallBackFunc(int event, int x, int y, int flags, void* userdata)
{

    location_node *self = static_cast<location_node*>(userdata);
    self->doMouseCallback(event, x, y, flags);

}

bool location_node::isIn(uiButton but, int x, int y){
    if(x < but.topR.x && x > but.botL.x && y < but.botL.y && y > but.topR.y) return true;
    return false;
}

void location_node::loadScreen(int screenNumber){
    if( screenNumber ==1){
        interface = cv::Mat(480,800,CV_8UC3,cv::Scalar(100,25,200));
        rectangle( interface,
                   leftBut.botL,
                   leftBut.topR,
                   Scalar( 0, 255, 255 ),
                   -1,
                   8 );
        rectangle( interface,
                   rightBut.botL,
                   rightBut.topR,
                   Scalar( 0, 255, 255 ),
                   -1,
                   8 );
        rectangle( interface,
                   midBut.botL,
                   midBut.topR,
                   Scalar( 0, 255, 255 ),
                   -1,
                   8 );
        cv::putText(interface, "Find fiducials", cvPoint(rightBut.botL.x + 25,rightBut.botL.y - 25),
                    FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,0), 1, CV_AA);
        cv::putText(interface, "Find robots", cvPoint(leftBut.botL.x + 25,leftBut.botL.y - 25),
                    FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,0), 1, CV_AA);
        cv::putText(interface, "GO!", cvPoint(midBut.botL.x + 25,midBut.botL.y - 25),
                    FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,0), 1, CV_AA);
        interfaceState = 1;
        imshow("interface",interface);
    }

}

void location_node::doMouseCallback(int event, int x, int y, int flags){
    switch (interfaceState){
    case 1:
        if(event == EVENT_LBUTTONDOWN){
            if(isIn(leftBut,x,y)){
                interfaceState = 2;
                cv::putText(image, "Click on robots (in order)", cvPoint( 25,image.rows - 25),
                            FONT_HERSHEY_COMPLEX_SMALL, 1.3, cvScalar(0,0,0), 1, CV_AA);
                imshow("interface",image);
            }
            if(isIn(rightBut,x,y)){
                interfaceState = 3;
                cv::putText(image, "Click on fiducial (in order)", cvPoint( 25,image.rows - 25),
                            FONT_HERSHEY_COMPLEX_SMALL, 1.3, cvScalar(0,0,0), 1, CV_AA);
                imshow("interface",image);
            }
            if(isIn(midBut,x,y)){
                interfaceState = 4;
                isSetup = 1;
            }
        }
        break;
    case 2:
        if(event == EVENT_LBUTTONDOWN){
            static int robotCount = 0;
            geometry_msgs::Point here;
            here.x = x;
            here.y = y;
            if(robotCount < number_robots-1){
                robotInterface[robotCount]->position = here;
                robotCount++;
            }
            else{
                robotInterface[robotCount]->position = here;
                robotCount++;
                robotCount = 0;
                loadScreen(1);
            }
        }
        break;

    case 3:
        if(event == EVENT_LBUTTONDOWN){
            static int fiducialCount = 0;
            Point2i here;
            here.x = x;
            here.y = y;
            if(fiducialCount < numberFedu-1){
                fiducialPoints[fiducialCount] = here;
                fiducialCount++;
            }
            else{
                fiducialPoints[fiducialCount] = here;
                fiducialCount++;
                fiducialCount = 0;
                loadScreen(1);

            }
        }
        break;

    case 4:
        if(event == EVENT_LBUTTONDOWN){
           loadScreen(1);
           fiduSearch = 50;
           robotSearch = 50;
           isSetup = 0;
        }
        break;

    }
}

location_node::location_node(int robotNum, char* argin):it_(nh_){
    if (*argin != 'y'){
        debug= 0;
    }
    camera = it_.subscribe("/camera",1,&location_node::imageCB,this);
    number_robots = robotNum;
    ROS_INFO("there are %d robots in this location system",number_robots);
    //make a vector of the right size with pub/sub for all features.
    isInitSub= nh_.subscribe("urobot_init_node/isInit", 1,&location_node::isInitCB,this);
    //do launching of stuff here, find all the robots and give them names. When complete call the 'publishIsInit' function. This is best to hapen only once, so lauch this node last and have others wait for this signal.
    for(int i= 0; i < number_robots; i ++){
        RobotClass* newRobot = new RobotClass(nh_,i,1);
        robotInterface.push_back(newRobot);
    }


    leftBut.botL.x = 50;
    leftBut.botL.y = 400;
    leftBut.topR.x = 250;
    leftBut.topR.y = 50;

    rightBut.botL.x = 300;
    rightBut.botL.y = 400;
    rightBut.topR.x = 500;
    rightBut.topR.y = 50;

    midBut.botL.x = 550;
    midBut.botL.y = 400;
    midBut.topR.x = 750;
    midBut.topR.y = 50;


    cvNamedWindow("interface",CV_WINDOW_NORMAL);
    cvSetWindowProperty("interface", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
    loadScreen(1);
    setMouseCallback("interface", CallBackFunc,this);

    // Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 12;

    // Filter by Area.
    params.filterByArea =false;
    params.minArea = 8;
    params.maxArea = 150;

    params.filterByInertia = false;

    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.1;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;

    params.thresholdStep = 1;

    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    //imshow( "Display window", image );
    //Create trackbars in "Control" window
    cvCreateTrackbar("FLowH", "Control", &FLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("FHighH", "Control", &FHighH, 179);

    cvCreateTrackbar("FLowS", "Control", &FLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("FHighS", "Control", &FHighS, 255);

    cvCreateTrackbar("FLowV", "Control", &FLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("FHighV", "Control", &FHighV, 255);

    cvCreateTrackbar("LLowH", "Control", &LLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("LHighH", "Control", &LHighH, 179);

    cvCreateTrackbar("LLowS", "Control", &LLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("LHighS", "Control", &LHighS, 255);

    cvCreateTrackbar("LLowV", "Control", &LLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("LHighV", "Control", &LHighV, 255);


    cvCreateTrackbar("radius", "Control", &radius, 255);




}


void location_node::isInitCB(const std_msgs::BoolConstPtr& msg)
{
    if (msg->data == true){
        isInit =1;
    }


}

void location_node::thesholdImage(cv::Mat &imageIn,bool needHead,std::vector<cv::KeyPoint> &Fkeypoints,bool needLED,std::vector<cv::KeyPoint> &Lkeypoints ){
    cv::Mat imgHSV,FimgThresholded,LimgThresholded;
    cv::cvtColor(imageIn, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    if(needHead) cv::inRange(imgHSV, cv::Scalar(FLowH, FLowS, FLowV), cv::Scalar(FHighH,FHighS, FHighV), FimgThresholded); //Threshold the image
    if(needLED)cv::inRange(imgHSV, cv::Scalar(LLowH, LLowS, LLowV), cv::Scalar(LHighH,LHighS, LHighV), LimgThresholded); //Threshold the image

    int erodeVal = 2;
    if(erodeVal !=0){
        if(needHead){
            cv::erode(FimgThresholded, FimgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeVal,erodeVal)) );
                        cv::dilate( FimgThresholded, FimgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeVal,erodeVal)) );
                        cv::dilate( FimgThresholded, FimgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeVal,erodeVal)) );
                        cv::erode(FimgThresholded, FimgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeVal,erodeVal)) );
        }

        if(needLED){
            cv::erode(LimgThresholded, LimgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeVal,erodeVal)) );
                        cv::dilate( LimgThresholded, LimgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeVal,erodeVal)) );
                        cv::dilate( LimgThresholded, LimgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeVal,erodeVal)) );
                        cv::erode(LimgThresholded, LimgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeVal,erodeVal)) );
        }
    }
    // Set up the detector with default parameters.
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    if(needHead) {
        cv::threshold(FimgThresholded, FimgThresholded, 0.5, 255,cv::THRESH_BINARY_INV);
        detector->detect(  FimgThresholded, Fkeypoints);
    }
    if(needLED) {
        cv::threshold(LimgThresholded, LimgThresholded, 0.5, 255,cv::THRESH_BINARY_INV);
        detector->detect(  LimgThresholded, Lkeypoints);
    }
    if(debug){
        if(needHead) {
        cvNamedWindow("heads", CV_WINDOW_NORMAL);
        cv::imshow("heads", FimgThresholded );
        }
        if(needLED){
        cvNamedWindow("leds", CV_WINDOW_NORMAL);
        cv::imshow("leds", LimgThresholded );
        }
        waitKey(1);
    }

}

Rect location_node::getSafeRect (Point2i point, int size, Mat &im){
    Point2i tLeft,bRight;
    tLeft.x = ((point.x - size > 0) ? point.x - size : 0);
    tLeft.y = ((point.y - size > 0) ? point.y - size : 0);
    bRight.x = ((point.x + size < im.cols-1)) ? point.x + size : im.cols-1;
    bRight.y = ((point.y + size < im.rows-1)) ? point.y + size : im.rows-1;
    return Rect(tLeft,bRight);
}

void location_node::imageCB(const sensor_msgs::ImageConstPtr& msg){
    bool shouldUpdate = false;
        static ros::Time old = ros::Time(0);
    static ros::Time lastupdate = ros::Time(0);
    ros::Duration framePeriod(1);
   if ((old - lastupdate).toSec() > framePeriod.toSec()) {
       lastupdate = ros::Time::now();
       shouldUpdate = true;
   }
    //get image from message.
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image = cv_ptr->image;
    if(isSetup){

        //#pragma omp parallel for
        for(int i = 0; i < numberFedu; i++){
            std::vector<cv::KeyPoint> Fkeypoints,Lkeypoints;
            Rect R =getSafeRect( fiducialPoints[i],fiduSearch,image);


            Mat thisPeice = image(R);

            thesholdImage(thisPeice,1,Fkeypoints,0,Lkeypoints );
            if(Fkeypoints.size() > 0){
                Point2f offset;
                offset.x = R.x;
                offset.y = R.y;
                fiducialPoints[i] = Fkeypoints[0].pt +offset;
            }
        }
        fiduSearch = 20;
        kPoints ourPairs[number_robots];
       // #pragma omp parallel for
        for(int i = 0; i < number_robots; i++){
            std::vector<cv::KeyPoint> Fkeypoints,Lkeypoints;
            geometry_msgs::Point position = robotInterface[i]->position;
            Point2f positionCV;
            positionCV.x = position.x;
            positionCV.y = position.y;
            Rect R =getSafeRect( positionCV,robotSearch,image);
            Mat thisPeice = image(R);
            thesholdImage(thisPeice,1,Fkeypoints,1,Lkeypoints );
            if(Fkeypoints.size() > 0&& Lkeypoints.size()> 0){
                int best;
                vector<location_node::pairs> thesePairs = location_node::findPairs(Fkeypoints, Lkeypoints, radius,best);
                kPoints winner;
                Point2f offset;
                offset.x = R.x;
                offset.y = R.y;
                winner.head = Fkeypoints[thesePairs[best].head].pt+offset;
                winner.led = Lkeypoints[best].pt+offset;
                winner.error = thesePairs[best].error;
                ourPairs[i] = winner;
            }
        }
        robotSearch = 35;


        cv::Mat im_with_keypoints;
        image.copyTo(im_with_keypoints);

        for(int i = 0;  i < number_robots; i++){
            if(ourPairs[i].error <130){
                if(shouldUpdate){
                    cv::circle(im_with_keypoints, ourPairs[i].led, radius, cv::Scalar(0,0,255) );


                    line(im_with_keypoints,ourPairs[i].led, ourPairs[i].head, cv::Scalar(255,0,255));
                    cv::Point2f centre = 0.55*ourPairs[i].led + 0.45*ourPairs[i].head;
                    cv::circle(im_with_keypoints, centre, 3, cv::Scalar(0,0,255) );
                }
                //get robot IDS here.
                robotInterface.at(i)->publishTF(ourPairs[i].led,ourPairs[i].head, msg->header.stamp);

            }
        }
        if(shouldUpdate){
        for(int i = 0; i < numberFedu; i ++){
            Point2i lineLengthx, lineLengthy;
            lineLengthx.x = 10;
            lineLengthy.y = 10;
            line(im_with_keypoints,fiducialPoints[i]+lineLengthx, fiducialPoints[i]-lineLengthx, cv::Scalar(255,0,255));
            line(im_with_keypoints,fiducialPoints[i]+lineLengthy, fiducialPoints[i]-lineLengthy, cv::Scalar(255,0,255));
        }
        }
        if(debug){
            // Show blobs
            cvNamedWindow("keypoints", CV_WINDOW_NORMAL); cv::imshow("keypoints", im_with_keypoints );
            cvNamedWindow("raw", CV_WINDOW_NORMAL);
            cv::imshow("raw",image);
        }


        ros::Duration period = ros::Time::now() - old;
         old = ros::Time::now();
         float fps = 1/ period.toSec();

        if(shouldUpdate){
            std::ostringstream buff;
            buff<<fps;
            cv::putText(im_with_keypoints, buff.str().c_str(), cvPoint( 25,im_with_keypoints.rows - 25),
                        FONT_HERSHEY_COMPLEX_SMALL, 1, cvScalar(0,0,0), 1, CV_AA);
            cv::imshow("interface", im_with_keypoints );
        }
    }
    ////// end of image testing.
    cv::waitKey(1);


}

vector<location_node::pairs> location_node::findPairs(std::vector<cv::KeyPoint> Fkeypoints, std::vector<cv::KeyPoint> Lkeypoints,float radius, int &best){
    //for all leds
    vector<location_node::pairs> thePairs;
    float globalBestScore = 10000000;
    for(int i = 0; i < Lkeypoints.size(); i ++){
        //find distance to heads.
        location_node::pairs thisPair;
        thisPair.led = i;
        float bestError = 100000000;
        for (int j = 0; j < Fkeypoints.size(); j++){
            cv::Point2f  A= Lkeypoints.at(i).pt;
            cv::Point2f  B= Fkeypoints.at(j).pt;
            float distance = findDistance(A,B);
            float error  = pow(distance - radius,2);
            if(error< bestError){
                thisPair.head = j;
                thisPair.error = error;
                bestError = error;
                if(error < globalBestScore){
                    globalBestScore = error;
                    best = i;
                }
            }

        }
        if(bestError <2000)thePairs.push_back(thisPair);

    }
    return thePairs;
}

float location_node::findDistance(cv::Point2f A,cv::Point2f B){
    float xChange = A.x - B.x;
    float yChange = A.y - B.y;
    return sqrt(pow(xChange,2)+pow(yChange,2));

}


int main(int argc, char** argv) {

    //ros setup
    ros::init(argc, argv, "location_node");
    location_node LN(atoi(argv[1]),argv[2]);
    ros::spin();
    return 0;
}
