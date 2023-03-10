#!/usr/bin/env python3

import rospy
import signal
import random

from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class Evader():
    def __init__(self):
        self.reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset()
        self.ack_pub = rospy.Publisher('car_1/command', AckermannDrive, queue_size=1)
        self.scan_sub = rospy.Subscriber('car_1/scan', LaserScan, self.scanner_callback)
        self.ack_msg = AckermannDrive()
        self.rate = rospy.Rate(10)
        self.count = 0
        self.sign = 1

    def scanner_callback(self, data):
        turn = False
        for i,x in enumerate(data.ranges):
            if x < 2:
                turn = True
                break
        #rospy.loginfo("Turn: "+str(int(turn)))
        if turn:
            if self.count == 0:
                self.sign = random.choice([-1,1])
            #print("Turning")
            self.count+=1
            self.ack_msg.speed = 1.0
            self.ack_msg.steering_angle = self.sign*random.uniform(0.7,1.2)
            self.ack_msg.steering_angle_velocity = 0.0
            #elf.rate.sleep()
        else:
            #print("continue")
            self.count=0
            self.ack_msg.speed = 4.0
            self.ack_msg.steering_angle = 0.0
            self.ack_msg.steering_angle_velocity = 0.0
        #rospy.loginfo("steer: "+str(self.ack_msg.steering_angle))
        self.ack_pub.publish(self.ack_msg)

if __name__ == '__main__':
    rospy.init_node('evader', anonymous=True)
    Evader()
    rospy.spin()

