#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
from tf.transformations import *
import time
import message_filters
from std_srvs.srv import Empty

class Mapper:
    def __init__(self):
        #self.scan_sub  = rospy.Subscriber('car_1/scan', LaserScan, self.scanner_callback)
        #self.odom_sub = rospy.Subscriber('car_1/base/odom', Odometry, self.odom_callback)
        self.reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset()
        self.scan_sub = message_filters.Subscriber('car_1/scan', LaserScan)
        self.odom_sub = message_filters.Subscriber('car_1/base/odom', Odometry)
        self.ats = message_filters.ApproximateTimeSynchronizer([self.scan_sub,self.odom_sub], queue_size=10, slop=0.1)
        self.ats.registerCallback(self.sync)
        self.rate = rospy.Rate(10)
        self.obstacles_x = []
        self.obstacles_y = []
        self.count=0
        self.pose = []
        self.size = []
        
        rospy.on_shutdown(self.shutdown)

    def sync(self, scan, odom):
        self.pose.append(odom.pose.pose)

        if self.count == 0:      
            for i,x in enumerate(scan.ranges):
                if x >= scan.range_min and x <= scan.range_max:
                    self.obstacles_x.append(-x*np.cos(scan.angle_min+scan.angle_increment*i))
                    self.obstacles_y.append(-x*np.sin(scan.angle_min+scan.angle_increment*i))
            self.size.append(len(self.obstacles_x))
            
        else:
            for i,x in enumerate(scan.ranges):
                if x <= 10:
                    self.obstacles_x.append(-x*np.cos(scan.angle_min+scan.angle_increment*i))
                    self.obstacles_y.append(-x*np.sin(scan.angle_min+scan.angle_increment*i))
            self.size.append(len(self.obstacles_x))

            
        self.count+=1

    def odom_callback(self, data):  
        self.pose.append(data.pose.pose)
        self.twist.append(data.twist.twist)

    def scanner_callback(self, data):
        if self.count == 0:      
            for i,x in enumerate(data.ranges):
                if x >= data.range_min and x <= data.range_max:
                    self.obstacles_x.append(-x*np.cos(data.angle_min+data.angle_increment*i))
                    self.obstacles_y.append(-x*np.sin(data.angle_min+data.angle_increment*i))
            self.size=len(self.obstacles_x)
            
        else:
            for i,x in enumerate(data.ranges):
                if x <= 10:
                    self.obstacles_x.append(-x*np.cos(data.angle_min+data.angle_increment*i))
                    self.obstacles_y.append(-x*np.sin(data.angle_min+data.angle_increment*i))

            
        self.count+=1
    
    def shutdown(self):
        rots=self.rotations()
        rc=[]
        
        for i in range(len(rots)):
            if i == 0:
                objs = np.vstack((np.array(self.obstacles_x[:self.size[i]]),np.array(self.obstacles_y[:self.size[i]]),np.zeros((1,self.size[i]))))
            else:
                objs = np.vstack((np.array(self.obstacles_x[self.size[i-1]:self.size[i]]),np.array(self.obstacles_y[self.size[i-1]:self.size[i]]),np.zeros((1,-self.size[i-1]+self.size[i]))))
            rc.append(np.dot(rots[i],objs))


        rospy.loginfo("Plotting")
                
        f=plt.figure()
        x=[]
        y=[]
        # rospy.loginfo(len(self.pose))
        # rospy.loginfo(len(rc))
        # rospy.loginfo(len(self.obstacles_x))
        # rospy.loginfo(len(self.size))
        # rospy.loginfo(self.count)
        for i in range(0,len(rc),10):
            for j in range(rc[i].shape[1]):
                x.append(rc[i][0][j]-self.pose[i+1].position.x)
                y.append(rc[i][1][j]-self.pose[i+1].position.y)

        plt.scatter(x,y, color='b',s=1)
        plt.scatter([-x.position.x for x in self.pose],[-x.position.y for x in self.pose], color='r')
        plt.show()

    def rotations(self):
        rots=[]
        q1 = [self.pose[0].orientation.x, self.pose[0].orientation.y, self.pose[0].orientation.z, -self.pose[0].orientation.w]
        for i in range(1,len(self.pose)):
            q2 = [self.pose[i].orientation.x, self.pose[i].orientation.y, self.pose[i].orientation.z, self.pose[i].orientation.w]
            q = quaternion_multiply(q2,q1)
            r = np.zeros((3,3))
            r[0][0] = 1-2*q[1]**2-2*q[2]**2
            r[1][1] = 1-2*q[0]**2-2*q[2]**2
            r[2][2] = 1-2*q[0]**2-2*q[1]**2
            r[0][1] = 2*q[0]*q[1]-2*q[3]*q[2]
            r[0][2] = 2*q[0]*q[2]+2*q[3]*q[1]
            r[1][0] = 2*q[0]*q[1]+2*q[3]*q[2]
            r[1][2] = 2*q[1]*q[2]-2*q[3]*q[0]
            r[2][0] = 2*q[0]*q[2]-2*q[3]*q[1]
            r[2][1] = 2*q[1]*q[2]+2*q[3]*q[0]
            rots.append(r)
        return rots
    




    def rpy(self):
        rots=[]
        q1 = [self.pose[i].orientation.x, self.pose[i].orientation.y, self.pose[i].orientation.z, -self.pose[i].orientation.w]
        for i in range(len(self.pose)-1):
            q2 = [self.pose[i+1].orientation.x, self.pose[i+1].orientation.y, self.pose[i+1].orientation.z, self.pose[i+1].orientation.w]
            q = quaternion_multiply(q2,q1)
            rpy=euler_from_quaternion(q)
            rx = np.array([[1,0,0],[0,np.cos(rpy[0]),-np.sin(rpy[0])],[0, np.sin(rpy[0]), np.cos(rpy[0])]])
            ry = np.array([[np.cos(rpy[1]),0,np.sin(rpy[1])],[0,1,0],[-np.sin(rpy[1]),0, np.cos(rpy[1])]])
            rz = np.array([[np.cos(rpy[0]),-np.sin(rpy[0]),0],[np.sin(rpy[0]), np.cos(rpy[0]),0],[0,0,1]])
            r=np.dot(np.dot(rx,ry),rz)
            rots.append(r)
        return rots



if __name__ == '__main__':
    rospy.init_node('Mapper', anonymous=True)
    m=Mapper()
    m.reset()
    #time.sleep(1)
    while 1:
        time.sleep(1)
        if len(m.obstacles_x) > m.size[0]:
            break
        else:
            continue
    time.sleep(1)
    plt.scatter(m.obstacles_x[:m.size[0]],m.obstacles_y[:m.size[0]], color='b')
    plt.scatter([0],[0],color='r')
    plt.show()
            
    rospy.spin()
