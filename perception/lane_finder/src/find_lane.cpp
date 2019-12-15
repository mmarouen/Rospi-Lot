#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <string>
#include <iomanip>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp" //fitline
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp> //rect,size

/*
Gets the raw image from the camera and returns a binary image containing lanes information
Subscribes to
    /raspicam_node/image
Publishes
    /perception/lanes 
*/

cv_bridge::CvImagePtr imageptr;
int thd, roiHeight, roiWidth, duration;

std::vector<cv::Point> flatten(const std::vector<std::vector<cv::Point> > &orig)
{   
    std::vector<cv::Point> ret;
    for(uint i=0;i<orig.size();i++){
        std::vector<cv::Point> v=orig[i];
        ret.insert(ret.end(), v.begin(), v.end());    
    }

    return ret;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        imageptr=cv_bridge::toCvCopy(msg, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    
}

void processImage(cv_bridge::CvImagePtr& imgptr)
{
    cv::Mat processedIm=imgptr->image;
    //cv::cvtColor(imgptr->image,processedIm,cv::COLOR_BGR2GRAY); //convert to grayscale
    cv::equalizeHist( processedIm, processedIm); //equalize histogram
    cv::threshold(processedIm,processedIm,thd,255,cv::THRESH_BINARY); //binary thresholding
    //erode image
    int erosion_size = 1;
    int vertical_size = processedIm.rows / 30; // Specify size on vertical axis
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(1, vertical_size));
    cv::erode( processedIm, processedIm, element);
    cv::GaussianBlur(processedIm,processedIm,cv::Size(3,3),0,0);
    //crop to ROI
    cv::Size imgSize=processedIm.size();
    cv::Point offset((int)(imgSize.width*0.5-roiWidth*0.5),(int)(imgSize.height-roiHeight));
    cv::Rect roi(offset.x,offset.y,roiWidth,roiHeight);
    //apply direction gradient on ROI
    cv::Mat roiImage = processedIm(roi);
    cv::Mat sobelx,sobely,gradDirection;
    cv::Sobel(roiImage,sobelx,CV_64F,1,0,3);
    cv::Sobel(roiImage,sobely,CV_64F,0,1,3);
    gradDirection = cv::Mat::zeros(roiImage.rows, roiImage.cols, CV_32F);
    sobelx.convertTo(sobelx,CV_32F);
    sobely.convertTo(sobely,CV_32F);
    cv::phase(sobelx, sobely, gradDirection); //between 0..pi
    cv::bitwise_and((gradDirection>0.8*CV_PI),roiImage,roiImage);
    cv::bitwise_and(((gradDirection<1.2*CV_PI)),roiImage,roiImage);
    //detect contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(roiImage,contours,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE,offset);
    //lane image
    cv::Mat res=cv::Mat::zeros(processedIm.size(), CV_8UC1);
    cv::drawContours(res,contours,-1,cv::Scalar(255),1);
    //draw angle position (if debug)
    /*
    std::vector<cv::Point> pts1;
    pts1 = flatten(contours);
    cv::Vec4f line;
    cv::fitLine(pts1,line,1,2,0.01,0.01);
    cv::Point C(imgSize.width*0.5,imgSize.height-roiHeight);
    cv::Point M(line[2]+(imgSize.height-roiHeight-line[3])*line[0]/line[1],imgSize.height-roiHeight);
    float heading=-atan2(M.x-C.x,roiHeight);
    cv::drawMarker(res,M,cv::Scalar(255),cv::MARKER_TILTED_CROSS,10);
    */
    imgptr->image = res;
}

int main (int argc, char **argv)
{
	ros::init (argc, argv, "find_lane");

	ros::NodeHandle n;
    ros::param::get("/perception/lanes/roiHeight",roiHeight);
    ros::param::get("/perception/lanes/roiWidth",roiWidth);
    ros::param::get("/perception/lanes/thd",thd);
    ros::NodeHandle n_params("~");
    n_params.param("duration", duration, (int)30);

	ros::Subscriber image_sub = n.subscribe("/raspicam_node/image", 3,image_callback);	
    //ROS_INFO("> Lane subscriber correctly initialized");
	ros::Publisher ros_pub_lanes = n.advertise<sensor_msgs::Image>("lanes",1);
    //ROS_INFO("> Lane publisher correctly initialized");
    
    ros::Rate loop_rate(10);
    time_t current=time(NULL);

    while (time(NULL)-current<duration)
    {
        if(imageptr){
            processImage(imageptr);
            imageptr->encoding ="mono8";
            ros_pub_lanes.publish(imageptr->toImageMsg());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    if(time(NULL)-current>=duration){
        ros::shutdown();
    }

  return 0;
}