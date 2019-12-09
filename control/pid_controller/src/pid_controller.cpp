#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include "i2cpwm_board/ServoArray.h"
#include "i2cpwm_board/Servo.h"

/*
Gets the twist command and returns a servo array message
Subscribes to
    /planning/cmd_vel
Publishes
    /control/servos_absolute
*/

geometry_msgs::Twist twist;
i2cpwm_board::ServoArray servos;
float kp=0.01;
float ki=0.01;
float kd=0.01;
float mn=0.0;
float mx=0.2;
float current_alpha=0;
double last_time=0;
double integral_init=0;
double last_error =0;
float center_val=330;
float range=90;

void yaw_callback(const geometry_msgs::Twist& msg)
{
    twist.linear.x=msg.linear.x;
    twist.angular.z=msg.angular.z;
}

void pidControl(const float alpha)
{
    //init controller variables
    float error= alpha - current_alpha;
    double current_time=ros::Time::now().toSec();
    double sample_time=current_time-last_time;
    //compute pid
    double integral=integral_init+error*sample_time;
    double deriv = (error-last_error)/sample_time;
    current_alpha = kp * error + ki * integral + kd * deriv;
    //update parameters
    last_time=current_time;
    integral_init = integral;
    last_error=error;
}

int main (int argc, char **argv)
{
	ros::init (argc, argv, "pid_controller");

	ros::NodeHandle n;

	ros::Subscriber lanes_sub = n.subscribe("/planning/trajectory/cmd_vel", 1,yaw_callback);	
    ROS_INFO("> Pid subscriber correctly initialized");
	ros::Publisher pub_vel = n.advertise<i2cpwm_board::ServoArray>("servos_absolute",1);
    ROS_INFO("> Pid publisher correctly initialized");
    
    ros::Rate loop_rate(1);
    int duration =30;
    time_t current=time(NULL);
    while (time(NULL)-current<duration)
    {

        if(true){
            pidControl(twist.angular.z);

            i2cpwm_board::Servo throttle;
            throttle.servo =1;
            throttle.value = twist.linear.x;
            i2cpwm_board::Servo steering;
            steering.servo =2;
            float servo2 = current_alpha*0.5*range+center_val;
            if(servo2>378) servo2=378;
            if(servo2<288) servo2=288;
            steering.value = servo2;
            i2cpwm_board::ServoArray servoArray;
            servoArray.servos.insert(servoArray.servos.end(),throttle);
            servoArray.servos.insert(servoArray.servos.end(),steering);
            pub_vel.publish(servoArray);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
  return 0;
}