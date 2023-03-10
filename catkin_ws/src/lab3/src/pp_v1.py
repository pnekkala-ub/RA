#! /usr/bin/env python3

import functools
import math
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point32, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import tf
from geometry_msgs.msg import Quaternion

class PP:
    def __init__(self) -> None:
        self.reset = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset()
        rospy.init_node("viz_node")
        rospy.on_shutdown(self.shutdown)
        self.track = np.loadtxt(rospy.get_param("/fname"), delimiter=",", dtype=str)
        self.Kdd = 0.8
        self.speed = rospy.get_param("/Vmax")
        self.ld = self.Kdd*self.speed
        self.ref_path_pub = rospy.Publisher(name="car_1/ref_path", data_class=MarkerArray, queue_size=1)
        self.close_pub = rospy.Publisher(name="car_1/close", data_class=Marker, queue_size=1)
        self.target_pub = rospy.Publisher(name="car_1/target", data_class=Marker, queue_size=1)
        self.odom_sub = rospy.Subscriber('car_1/base/odom', Odometry, self.steer)
        self.ack_pub = rospy.Publisher("car_1/command", AckermannDrive, queue_size=1)
        #self.launch_pos = rospy.wait_for_message("/car_1/base/odom", Odometry, timeout=5)
        self.last_l, self.last_c = [0,0,0], [0,0,0]
        self.listener = tf.TransformListener()
        self.listener.waitForTransform('odom','car_1_base_link',rospy.Time(0),rospy.Duration(4.0))
        self.plot_path()

    def shutdown(self):
        pass

    def find_closest(self, curr):
        min_dist = 1e5
        idx=0
        for i,x in enumerate(self.track[1:]):
            dist = (curr.pose.pose.position.x-float(x[1]))**2 + (curr.pose.pose.position.y-float(x[2]))**2
            if  dist < min_dist:
                min_dist = dist
                idx=i
                #break
            # if i == len(self.track[1:])-1:
            #     return self.last_c
        print(idx+1)
        return self.track[idx+1]
    
    def find_lookahead(self, curr):
        min_dist = 1e5
        idx=0
        ps2=PoseStamped()
        ps2.header = curr.header
        ps2.header.stamp = rospy.Time(0)
        ps2.pose.position.x = curr.pose.pose.position.x
        ps2.pose.position.y = curr.pose.pose.position.y
        ps2.pose.position.z = 0.0
        ps2.pose.orientation.x=curr.pose.pose.orientation.x
        ps2.pose.orientation.y=curr.pose.pose.orientation.y
        ps2.pose.orientation.z=curr.pose.pose.orientation.z
        ps2.pose.orientation.w=curr.pose.pose.orientation.w
        npose2 = self.listener.transformPose('car_1_base_link',ps2)
        q=[npose2.pose.orientation.x,npose2.pose.orientation.y,npose2.pose.orientation.z,npose2.pose.orientation.w]
        v=[npose2.pose.position.x,npose2.pose.position.y, 0.0, 0.0]
        dir = list(qv_mult(q,v))
        for i,x in enumerate(self.track[1:]):
            try:
                ps1=PointStamped()
                ps1.header = curr.header
                ps1.header.stamp = rospy.Time(0)
                ps1.point.x = float(x[1])
                ps1.point.y = float(x[2])
                ps1.point.z = 0.0
                npose1 = self.listener.transformPoint('car_1_base_link',ps1)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("transform exception")
            if  np.dot(dir,[-npose2.pose.position.x+npose1.point.x,-npose2.pose.position.y+npose1.point.y])>= 0:
                dist = math.sqrt((npose1.point.x-npose2.pose.position.x)**2 + (npose1.point.y-npose2.pose.position.y)**2)
                if  abs(dist-self.ld) < min_dist:
                    alpha = math.atan2((npose1.point.y-npose2.pose.position.y), (npose1.point.x-npose2.pose.position.x))
                    min_dist = abs(dist-self.ld)
                    idx=i
        
        #npose = self.listener.transformPoint('car_1_base_link',ps)
        #rospy.loginfo(npose1.point.x)
        #rospy.loginfo(npose1.point.y)
        #rospy.loginfo(npose2.pose.position.x)
        #rospy.loginfo(npose2.pose.position.y)
        #rospy.loginfo(curr.pose.pose.position.x)
        #rospy.loginfo(curr.pose.pose.position.y)
        print(idx+1)
        return self.track[idx+1], alpha


    def plot_path(self):
        ref = MarkerArray()
        for itr, pt in enumerate(self.track[1:]):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.id = itr
            marker.type=Marker.SPHERE
            marker.type=Marker.ADD
            marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1
            marker.color.a = 1.0
            marker.color.r, marker.color.g, marker.color.b = 0, 1, 1
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = float(pt[1])
            marker.pose.position.y = float(pt[2])
            ref.markers.append(marker)
        
        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.ref_path_pub.publish(ref)
            r.sleep()

    def plot_close_target(self, curr):
        close = self.find_closest(curr)
        
        closest = Marker()
        closest.header.frame_id = "odom"
        closest.id = 0
        closest.type=closest.SPHERE
        closest.type=closest.ADD
        closest.scale.x, closest.scale.y, closest.scale.z = 0.5, 0.5, 0.5
        closest.color.a = 1.0
        closest.color.r, closest.color.g, closest.color.b = 0.63, 0.13, 0.94
        closest.pose.orientation.w = 1.0
        closest.pose.position.x = float(close[1])
        closest.pose.position.y = float(close[2])

        lookahead, alpha = self.find_lookahead(curr)

        target = Marker()
        target.header.frame_id = "odom"
        target.id = 0
        target.type=target.SPHERE
        target.type=target.ADD
        target.scale.x, target.scale.y, target.scale.z = 0.5, 0.5, 0.5
        target.color.a = 1.0
        target.color.r, target.color.g, target.color.b = 1.0, 1.0, 0.0
        target.pose.orientation.w = 1.0
        target.pose.position.x = float(lookahead[1])
        target.pose.position.y = float(lookahead[2])

        self.close_pub.publish(closest)
        self.target_pub.publish(target)

        #rospy.loginfo("Close: "+str(closest.pose.position.x)+" , "+str(closest.pose.position.y))
        #rospy.loginfo("Target: "+str(target.pose.position.x)+" , "+str(target.pose.position.y))

        return close, lookahead, alpha

    def steer(self, odom):
        c,l, alpha = self.plot_close_target(odom)

        #alpha = math.atan2(-odom.pose.pose.position.y+float(l[2]), -odom.pose.pose.position.x+float(l[1]))
        #alpha = math.atan2(-float(c[2])+float(l[2]), -float(c[1])+float(l[1]))
        #d = ((-odom.pose.pose.position.y+float(l[2]))**2 + (-odom.pose.pose.position.x+float(l[1]))**2)**0.5
        theta = math.atan(0.768*math.sin(alpha)/self.ld)
        #theta = 2*abs(float(c[2])-float(l[2]))/self.ld**2
        sa = theta
        # if sa < 0:
        #     sa = -0.435 + theta%0.435
        # else:
        #     sa = theta%0.435
        sa = np.clip(sa, -0.435, 0.435)
        rospy.loginfo("sa: "+str(sa*180/np.pi))
        drive = AckermannDrive()
        drive.speed = self.speed
        drive.steering_angle = sa
        self.ack_pub.publish(drive)

def qv_mult(q1, v1):
    # comment this out if v1 doesn't need to be a unit vector
    #v1 = tf.transformations.unit_vector(v1)
    #q1 = [q1[0],q1[1],q1[2],q1[3]]
    #v1 = [v1[0],v1[1],v1[2],v1[3]]
    q2=q1
    q2[3] = -q2[3]
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, v1), 
        #tf.transformations.quaternion_conjugate(q1)
        q2
    )[:2]


if __name__ == "__main__":
    pp = PP()

