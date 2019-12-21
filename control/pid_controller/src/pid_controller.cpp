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
#include "i2cpwm_board/StopServos.h"
#include "i2cpwm_board/ServoConfig.h"

/*
Gets the twist command and returns a servo array message
Subscribes to
    /planning/cmd_vel
Publishes
    /control/servos_absolute
*/

//runtime params
time_t lastTimeReceivedCmd;
bool received;
//servo params
i2cpwm_board::Servo throttle;
i2cpwm_board::Servo steering;
i2cpwm_board::ServoArray servoArray;
float steering_center_val;
float throttle_center_val;
float steering_range;
float throttle_range;
int duration;

//pid params
float kp=0.01;
float ki=0.01;
float kd=0.001;
float mn=0.0;
float mx=0.2;
float current_alpha=0;
double last_time=0;
double integral_init=0;
double last_error =0;

void pidControl(const float alpha)
{
    //init controller variables
    float error= alpha - current_alpha;
    double current_time=ros::Time::now().toSec();
    double sample_time=current_time-last_time;
    if(last_time==0){
        sample_time=0.1;
    }
    //update time
    //compute pid
    double integral=integral_init+error*sample_time;
    double deriv = (error-last_error)/sample_time;
    current_alpha = kp * error + ki * integral + kd * deriv;
    std::cout<<"sample time "<<sample_time<<" current_alpha "<<current_alpha<<std::endl;
    //update parameters
    last_time=current_time;
    integral_init = integral;
    last_error=error;
}

void update_servo_array()
{
    servoArray.servos.clear();
    servoArray.servos.push_back(throttle);
    servoArray.servos.push_back(steering);
}

void yaw_callback(const geometry_msgs::Twist& msg)
{
    lastTimeReceivedCmd=time(NULL);
    received=true;
    pidControl(msg.angular.z);
    float servo2 = current_alpha*0.5*steering_range+steering_center_val;
    std::cout<<"msg.z "<<msg.angular.z<<" servo "<<servo2<<std::endl;;
    if(servo2>(steering_center_val+0.5*steering_range)) servo2=steering_center_val+0.5*steering_range;
    if(servo2<(steering_center_val-0.5*steering_range)) servo2=steering_center_val-0.5*steering_range;
    //std::cout<<"servo2 "<<servo2<<" ";
    steering.servo =2;
    steering.value = servo2;
    throttle.servo =1;
    float speed = msg.linear.x*throttle_range+throttle_center_val;
    if(speed>(throttle_center_val+throttle_range)) speed=360;
    if(speed<throttle_center_val) speed=throttle_center_val;
    throttle.value = speed;
}

void set_idle()
{
    throttle.servo =1;
    throttle.value = throttle_center_val;
    steering.servo =2;
    steering.value = steering_center_val;
}

int main (int argc, char **argv)
{
	ros::init (argc, argv, "pid_controller");
    ros::NodeHandle n;
    ros::Publisher pub_vel = n.advertise<i2cpwm_board::ServoArray>("servos_absolute",1);
	ros::Subscriber lanes_sub = n.subscribe("/planning/trajectory/cmd_vel", 1,yaw_callback);
	//ros::Subscriber lanes_sub = n.subscribe("/cmd_vel", 1,yaw_callback);
    //load parameters
    ros::NodeHandle n_params("~");
    n_params.param("duration", duration, (int)30);
    ros::param::get("/control/pid/centerSteering",steering_center_val);
    ros::param::get("/control/pid/centerThrottle",throttle_center_val);
    ros::param::get("/control/pid/rangeSteering",steering_range);
    ros::param::get("/control/pid/rangeThrottle",throttle_range);

    //setup i2cpwm board services
    ros::ServiceClient serv1 = n.serviceClient<i2cpwm_board::StopServos>("stop_servos");
    i2cpwm_board::StopServos stopServos;
    ros::ServiceClient serv2 = n.serviceClient<i2cpwm_board::StopServos>("config_servos");
    i2cpwm_board::ServoConfig servoConfig;
    
    //setup timers and rate
    ros::Rate loop_rate(10);
    time_t initTime=time(NULL);
    lastTimeReceivedCmd=time(NULL);
    int timeout=6;

    while (time(NULL)-initTime<duration)
    {
        if(time(NULL)-lastTimeReceivedCmd>timeout)
        {
            received=false;
            set_idle();
            update_servo_array();
            pub_vel.publish(servoArray);
        }
        if(received)
        {
            update_servo_array();
            pub_vel.publish(servoArray);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    if(time(NULL)-initTime>=duration)
    {
        //serv1.call(stopServos);
        set_idle();
        update_servo_array();
        pub_vel.publish(servoArray);
        ros::shutdown();
    }    
    
  return 0;
}