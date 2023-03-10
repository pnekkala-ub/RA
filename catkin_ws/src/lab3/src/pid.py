#! /usr/bin/env python3

import functools
import math
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_srvs.srv import Empty
from std_msgs.msg import Float32

class PID:
    def __init__(self) -> None:
        self.reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset()
        rospy.init_node("viz_node")
        self.last_time = rospy.get_time()
        self.last_error = 0.0
        self.last_sa = 0.0
        self.frequency = 0.01
        self.p, self.i, self.d = 0.0, 0.0, 0.0
        self.kp, self.ki, self.kd = 0.01, 1.0, 0.08
        self.ref_pub = rospy.Publisher(name="car_1/ref", data_class=Marker, queue_size=1)
        self.odom_sub = rospy.Subscriber('car_1/base/odom', Odometry, self.steer)
        self.ack_pub = rospy.Publisher("car_1/command", AckermannDrive, queue_size=1)
        self.error_pub = rospy.Publisher("error", Float32, queue_size=1)
        self.p_pub = rospy.Publisher("P",Float32, queue_size=1)
        self.i_pub = rospy.Publisher("I",Float32, queue_size=1)
        self.d_pub = rospy.Publisher("D",Float32, queue_size=1)
        self.error_track=[]
        self.mae=[]
        rospy.on_shutdown(self.shutdown)
        self.plot_marker()

    def shutdown(self):
        print(self.mae)

    def plot_marker(self):
        x = np.arange(201)

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.id = 0
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = 1, 1, 0
        marker.pose.orientation.w = 1.0
        for i, p in enumerate(x):
            pt = Point32()
            pt.x = p
            if p >=0 and p <= 15:
                pt.y = 0.0
            elif p > 15 and p <= 30:
                pt.y = 2.0
            elif p > 15 and p <= 30:
                pt.y = 2.0
            elif p > 30 and p <= 45:
                pt.y = 5.0
            elif p > 45 and p <= 60:
                pt.y = 10.0
            elif p > 60 and p <= 200:
                pt.y = 15.0
            else:
                pt.y = 15.0
            
            marker.points.append(pt)
        
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.ref_pub.publish(marker)
            r.sleep()

    def steer(self, odom):
        curr_time=rospy.get_time()
        #rospy.loginfo("Time: "+str(curr_time))
        dt = curr_time-self.last_time
        if dt == 0.0:
            dt = 1e-2
        rospy.loginfo("DT; "+str(dt))
        ox = odom.pose.pose.position.x    
        if ox < 0:
            drive = AckermannDrive()
            drive.speed = rospy.get_param("/Vmax")
            self.ack_pub.publish(drive)
        else:
            oy = odom.pose.pose.position.y
            if ox >=0 and ox <=15:
                error=-oy
            elif ox >15 and ox <=30:
                error=-oy+2
            elif ox >30 and ox <=45:
                error=-oy+5
            elif ox >45 and ox <=60:
                error=-oy+10
            elif ox >60 and ox <=200:
                error=-oy+15
            else:
                pass
            self.error_track.append(abs(error))
            if ox > 15 and len(self.mae) == 0:
                self.mae.append(functools.reduce(lambda a,b:a+b, self.error_track)/len(self.error_track))
            elif ox > 30 and len(self.mae) == 1:
                self.mae.append(functools.reduce(lambda a,b:a+b, self.error_track)/len(self.error_track))
            elif ox > 45 and len(self.mae) == 2:
                self.mae.append(functools.reduce(lambda a,b:a+b, self.error_track)/len(self.error_track))
            elif ox > 60 and len(self.mae) == 3:
                self.mae.append(functools.reduce(lambda a,b:a+b, self.error_track)/len(self.error_track))
            elif ox > 200 and len(self.mae) == 4:
                self.mae.append(functools.reduce(lambda a,b:a+b, self.error_track)/len(self.error_track))
            #d_err = abs(error) - abs(self.last_error)
            d_err = error - self.last_error
            self.last_error = error
            self.error_pub.publish(error)
            self.p = error
            self.i += (d_err*dt)
            self.d = d_err/dt
            self.p_pub.publish(self.p)
            self.i_pub.publish(self.i)
            self.d_pub.publish(self.d)

            output = self.kp*self.p+self.ki*self.i+self.kd*self.d
            
            sa = output
            if sa < 0:
                sa = -0.435 + output%0.435
            else:
                sa = output%0.435
            # if sa < -0.435:
            #     sa = -0.435
            # elif sa > 0.435:
            #     sa = 0.435
            self.last_sa = sa
            self.last_time = curr_time
            #rospy.loginfo("X: "+str(ox))
            #rospy.loginfo("error: "+str(error))
            #rospy.loginfo("output: "+str(output))
            #rospy.loginfo("sa: "+str(sa*180/np.pi))
            drive = AckermannDrive()
            drive.speed = rospy.get_param("/Vmax")
            drive.steering_angle = sa
            self.ack_pub.publish(drive)

    # def steer(self, odom):
    #     curr_time=rospy.get_time()
    #     rospy.loginfo("Time: "+str(curr_time))
    #     dt = curr_time-self.last_time
    #     #self.last_time = curr_time
    #     rospy.loginfo("DT; "+str(dt))
    #     ox = odom.pose.pose.position.x    
    #     if ox < 0:
    #         drive = AckermannDrive()
    #         drive.speed = 2.0
    #         self.ack_pub.publish(drive)
    #     else:
    #         if dt < self.frequency:
    #             sa = 0.0
    #         else:
    #             oy = odom.pose.pose.position.y
    #             self.last_inp = oy
                
    #             #error = -oy
    #             if ox >=0 and ox <=15:
    #                 error=-oy
    #             elif ox >15 and ox <=30:
    #                 error=-oy+2
    #             elif ox >30 and ox <=45:
    #                 error=-oy+5
    #             elif ox >45 and ox <=60:
    #                 error=-oy+10
    #             elif ox >60 and ox <=200:
    #                 error=-oy+15
    #             else:
    #                 #error=-oy+15
    #                 pass
    #             d_err = abs(error) - abs(self.last_error)
    #             self.last_error = error
    #             self.error_pub.publish(error)
    #             self.p = error
    #             self.i += (error*dt)
    #             self.d = d_err/dt
    #             self.p_pub.publish(self.p)
    #             self.i_pub.publish(self.i)
    #             self.d_pub.publish(self.d)

    #             output = self.kp*self.p+self.ki*self.i+self.kd*self.d
    #             #sa = np.arctan2(output, ox) % 2*np.pi
    #             #sa =math.atan2(output, ox)
    #             sa = output
    #             if sa < -0.435:
    #                 sa = -0.435
    #             elif sa > 0.435:
    #                 sa = 0.435
    #             self.last_sa = sa
    #             self.last_time = curr_time
    #             rospy.loginfo("X: "+str(ox))
    #             rospy.loginfo("error: "+str(error))
    #             rospy.loginfo("output: "+str(output))
    #         drive = AckermannDrive()
    #         drive.speed = 2.0
    #         drive.steering_angle = sa
    #         self.ack_pub.publish(drive)
    #         #rospy.loginfo("steering: "+str(sa))

        



if __name__ == "__main__":
    pid = PID()
    #pid.plot_marker()




