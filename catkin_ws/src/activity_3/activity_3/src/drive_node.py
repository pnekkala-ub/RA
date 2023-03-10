#! /usr/bin/env python3 

# ROS node for UB CSE568 class activity
# Author: Yashom Dighe 

import rospy 
from ackermann_msgs.msg import AckermannDrive

def shutdown_handler(): # optional
    drive_cmd = AckermannDrive()
    driver.publish(drive_cmd)
    

if __name__ == "__main__":
    rospy.init_node("driver_node") # Initilize the ros node

    rospy.on_shutdown(shutdown_handler) # Set up a shutdown handler (optional)

    driver = rospy.Publisher(name="car_1/command", data_class=AckermannDrive, queue_size=10) # declare the publisher
        
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        drive_cmd = AckermannDrive()

        drive_cmd.speed = 0.5 # forward speed of the car
        drive_cmd.steering_angle = 0.3 # steering angle in radians

        driver.publish(drive_cmd)
        rate.sleep()



        

