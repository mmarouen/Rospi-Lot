#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>

/*
Gets binary lane information and returns a trajectory for the vehicle to follow
Subscribes to
    /perception/lanes
Publishes
    /planning/trajectory
*/

cv_bridge::CvImagePtr imageptr;
float speed;
int roiHeight;
float yaw;

template<typename T>
std::vector<T> flatten(const std::vector<std::vector<T> > &orig)
{   
    std::vector<T> ret;
    for(uint i=0;i<orig.size();i++){
        std::vector<T> v=orig[i];
        ret.insert(ret.end(), v.begin(), v.end());
    }
    return ret;
}

void lane_callback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        imageptr=cv_bridge::toCvCopy(msg, "mono8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
    }
}

void generateHeading(cv_bridge::CvImagePtr& imgptr)
{
    cv::Mat img=imgptr->image;
    cv::Size imgSize=img.size();
    //find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(img,contours,cv::RETR_LIST,cv::CHAIN_APPROX_SIMPLE);
    //fit line
    std::vector<cv::Point> pts;
    pts = flatten(contours);
    if(pts.size()>0){
        cv::Vec4f line;
        cv::fitLine(pts,line,1,2,0.01,0.01);
        //find heading points O,M,C
        cv::Point O(imgSize.width*0.5,imgSize.height);
        cv::Point C(imgSize.width*0.5,imgSize.height-roiHeight);
        cv::Point M(line[2]+(imgSize.height-roiHeight-line[3])*line[0]/line[1],imgSize.height-roiHeight);
        //generate heading
        yaw=-atan2(M.x-C.x,roiHeight);
        std::cout<<"found points, yaw:"<<yaw<<std::endl;
    }else{
        std::cout<<"no points found, yaw:"<<yaw<<std::endl;

    }
}

int main (int argc, char **argv)
{
	ros::init (argc, argv, "generate_trajectory");

	ros::NodeHandle n;
    ros::NodeHandle n_params("~");
    n_params.param("speed", speed, (float)360);
    n_params.param("roiHeight", roiHeight, (int)100);

	ros::Subscriber lanes_sub = n.subscribe("/perception/lanes/lanes", 1,lane_callback);	
    ROS_INFO("> Trajectory subscriber correctly initialized");
	ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    ROS_INFO("> Trajectory publisher correctly initialized");
    
    ros::Rate loop_rate(1);
    int duration =30;
    time_t current=time(NULL);
    geometry_msgs::Twist vel_msg;

    while (time(NULL)-current<duration)
    {
        if(imageptr){
            generateHeading(imageptr);
            vel_msg.linear.x=speed;
            vel_msg.angular.z=yaw*180/CV_PI;
            pub_vel.publish(vel_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}