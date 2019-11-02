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
#include <cv_bridge/cv_bridge.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>


/*
Gets the raw image from the camera and returns a binary image containing lanes information
Subscribes to
    /raspicam_node/image
Publishes
    /perception/lanes 
*/
//sensor_msgs::Image current_frame;
cv_bridge::CvImagePtr imageptr;
//cv_bridge::CvImage cv_image;


void image_callback(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        imageptr=cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    
}

void processImage(cv_bridge::CvImagePtr& imgptr){
    cv::cvtColor(imgptr->image,imgptr->image,cv::COLOR_BGR2GRAY);
    cv::equalizeHist( imgptr->image, imgptr->image );
    cv::threshold(imgptr->image,imgptr->image,127,255,cv::THRESH_BINARY);
    int erosion_size = 1;
    cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                        cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                        cv::Point( erosion_size, erosion_size ) );
    cv::erode( imgptr->image, imgptr->image, element );
}

template <typename T>
std::string to_string(T value)
{
	std::ostringstream os ;
	os << value ;
	return os.str() ;
}

int main (int argc, char **argv)
{
	// globals
	

	ros::init (argc, argv, "find_lane");

	ros::NodeHandle n;

	ros::Subscriber image_sub = n.subscribe("/raspicam_node/image", 1,image_callback);	
    ROS_INFO("> Subscriber correctly initialized");
	ros::Publisher ros_pub_lanes = n.advertise<sensor_msgs::Image>("/perception/lanes",1);
    //image_transport::ImageTransport it(n);
    //image_transport::Publisher ros_pub_lanes = it.advertise("/perception/lanes", 1);
    ROS_INFO("> Publisher correctly initialized");
    
    ros::Rate loop_rate(1);

    int count = 0;
    int duration =20;
    time_t current=time(NULL);

    while (time(NULL)-current<duration)
    {
        if(imageptr){
            processImage(imageptr);
            //std::string txt="/home/ubuntu/catkin_ws/src/perception/"+to_string(count)+"_before.png";
            //cv::imwrite(txt, imageptr->image );
            //ros_pub_lanes.publish(imageptr->toImageMsg());

            sensor_msgs::ImagePtr ros_image;
            ros_image = imageptr->toImageMsg();
            ros_image->encoding=sensor_msgs::image_encodings::MONO8;
            ros_pub_lanes.publish(ros_image);
            /*
            sensor_msgs::ImagePtr ros_image;
            ros_image = imageptr->toImageMsg();
            try
            {
                ros_image->encoding=sensor_msgs::image_encodings::MONO8;
                std::cout<<"image ptr encoding "<<ros_image->encoding<<std::endl;
                std::cout<<"width "<<ros_image->width<<std::endl;
                cv_bridge::CvImagePtr imageptr0=cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::MONO8);
                std::string txt0="/home/ubuntu/catkin_ws/src/perception/"+to_string(count)+"_after.png";
                cv::imwrite(txt0, imageptr0->image );
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
            */
            

        }
        //std::cout<<"count "<<count<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

  return 0;
}

