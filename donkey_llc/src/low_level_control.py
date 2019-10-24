#!/usr/bin/python

import rospy, time
from i2cpwm_board.msg import Servo,ServoArray
from geometry_msgs.msg import Twist

class ServoConvert():
    """
    class for handling the servos for the i2c control i2cpwm_board
    """
    def __init__(self, id=1, center_value=333, range=90, direction=1):
        self.value = 0.0
        self.center_value = center_value
        self.id = id

        self._center = center_value
        self._range = range
        self._half_range = 0.5*range
        self._dir = direction

        #-- convert range in [-1,1]
        self._sf = 1.0/self._half_range
    
    def get_value_out(self, value_in):
        """
        given an input value in [-1,1], it converts it in the actual servo range
        """
        self.value = value_in
        self.value_out = int(self._dir*value_in*self._half_range+self._center)
        return(self.value_out)

class DkLowLevelCtrl():
    """
    Low level control for donkey car in ROS
    """
    def __init__(self):
        rospy.loginfo("Setting up the node ...")

        rospy.init_node("dk_llc")

        #--- Create an actuator dictionary
        self.actuators = {}
        self.actuators['throttle'] = ServoConvert(id=1)
        self.actuators['steering'] = ServoConvert(id=2, direction=1)#-positive left

        #--- Create servo array publisher
        #-- Create the message
        self._servo_msg = ServoArray()
        for i in range(2):
            self._servo_msg.servos.append(Servo())
        self._ros_pub_servo_array = rospy.Publisher("servos_absolute",ServoArray,queue_size=1)
        rospy.loginfo("> Publisher correctly initialized")

        #--- Create the subscriber to the /cmd_vel topic
        self.ros_sub_twist = rospy.Subscriber("/cmd_vel",Twist,self.set_actuators_from_cmdvel)
        rospy.loginfo("> Subscriber correctly initialized")

        #--- Save the last time we got a correspondence. Stop the vehicle
        self._last_time_cmd_rcv = time.time()
        self._timeout_s = 5 #-- stop after 5 sec

        rospy.loginfo("Initialization complete")

    def set_actuators_from_cmdvel(self,message):
        """
        Gets a Twist message from /cmd_vel, assuming max input is 1
        """
        #-- save the time
        self._last_time_cmd_rcv = time.time()

        #-- convert the message into servo message
        self.actuators['throttle'].get_value_out(message.linear.x) #- positive fwd
        self.actuators['steering'].get_value_out(message.angular.z) #- positive right

        rospy.loginfo("Got a command v= %2.1f s=%2.1f"%(message.linear.x,message.angular.z))

        #-- Publish the servo message
        self.send_servo_msg()
    
    def set_actuators_idle(self):
        self.actuators['throttle'].get_value_out(0) #-positive fwd
        self.actuators['steering'].get_value_out(0) #-positive right
        rospy.loginfo("Setting actuators to idle")

        #-- Publish the message using a function
        self.send_servo_msg()

    def send_servo_msg(self):
        """
        Publish the current actuators value to the i2cpwm board
        Servos = array of servos. Each Servo has an id and an array to fill
        """
        for actuator_name,servo_obj in self.actuators.iteritems():
            self._servo_msg.servos[servo_obj.id -1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id -1].value = servo_obj.value_out
            rospy.loginfo("Sending to %s command %d"%(actuator_name,servo_obj.value_out))
        self._ros_pub_servo_array.publish(self._servo_msg)
    
    @property
    def is_controller_connected(self):
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)
    
    def run(self):

        #- set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            print(str(self._last_time_cmd_rcv)+" controller connected: "+str(self.is_controller_connected))

            if not self.is_controller_connected:
                self.set_actuators_idle()
            rate.sleep()

"""
Execute main function
"""
if __name__ == "__main__":
    dk_llc = DkLowLevelCtrl()
    dk_llc.run()