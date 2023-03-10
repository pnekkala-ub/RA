#! /usr/bin/env python3 

# ROS node for UB CSE568 class activity
# Author: Yashom Dighe, hunter Rozensky

import math

import rospy
import tf
import tf.transformations as trans

from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


class Driver:
    def __init__(self):
        rospy.init_node("driver_node") # Initilize the ros node
        rospy.on_shutdown(self.shutdown_handler) # Set up a shutdown handler (optional)

        self.goal = Point()

        self.driver = rospy.Publisher(name="/car_1/command", data_class=AckermannDrive, queue_size=10) # declare the publisher

        goal_setter = rospy.Subscriber(name="/clicked_point", data_class=PointStamped, callback=self.goal_setter, queue_size=1)
        odom = rospy.Subscriber(name="/car_1/base/odom", data_class=Odometry, callback=self.navigate, queue_size=1)

    def shutdown_handler(self): # optional
        # emergency stop
        drive_cmd = AckermannDrive()
        self.driver.publish(drive_cmd)
        return

    # simply saves goal to instance variable
    def goal_setter(self, waypoint: PointStamped):
        self.goal = waypoint.point
        print("point clicked")
        return

    def navigate(self, odom: Odometry):
        dx = odom.pose.pose.position.x - self.goal.x
        dy = odom.pose.pose.position.y - self.goal.y

        orientation = odom.pose.pose.orientation
        euler = trans.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
        # Must sync both angular values to be 0 to 2pi, counterclockwise starting at -y
        # car yaw angle is -pi to pi, counterclockwise starting at -y
        # adjust by pi
        heading_current = euler[2] + math.pi
        # atan2 outputs yaw angle from -pi to pi, counterclockwise starting at +y
        # take the 2pi modulo
        heading_goal = math.atan2(dy, dx) % (2*math.pi)

        # minimum distance heading change, range: -pi to pi
        # pos = left
        # neg = right
        heading_change = (heading_goal - heading_current + 3*math.pi) % (2*math.pi) - math.pi
        print(f"current: {heading_current}, goal: {heading_goal}, change: {heading_change}")
        distance = ((odom.pose.pose.position.x - self.goal.x)**2 + \
                    (odom.pose.pose.position.y - self.goal.y)**2)**0.5

        # add a debug statement
        
        # self.drive(heading_change, distance)
        return
    
    def drive(self, heading_change, distance):
        # empty drive message
        drive_cmd = AckermannDrive()
        # guard clause when it is near target
        if distance < .5:
            self.driver.publish(drive_cmd)
            return
        
        # --- write your drive controller here ---


        self.driver.publish(drive_cmd)
        return

if __name__ == "__main__":
    driver = Driver()
    rospy.spin()

