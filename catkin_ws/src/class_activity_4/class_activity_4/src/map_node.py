#! /usr/bin/env python3

# ROS node for UB CSE 568 class activity
# Author: hunter Rozensky

import rospy
from nav_msgs.msg import OccupancyGrid

if __name__ == "__main__":
    # we need to publish a map to click on.
    rospy.init_node("map_pub")

    pub = rospy.Publisher(name="/map", data_class=OccupancyGrid, queue_size=1)
    rate = rospy.Rate(1)

    map_data = [0]

    # --- create map here ---
    map = OccupancyGrid()
    map.info.height = 1
    map.info.width = 1
    map.info.resolution = 100

    map.info.origin.position.x = -50
    map.info.origin.position.y = -50

    map.data = map_data

    # publish map here
    while not rospy.is_shutdown():
        pub.publish(map)
        rate.sleep()

