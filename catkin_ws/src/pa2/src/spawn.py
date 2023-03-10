#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
from random import randint,uniform
from pathlib import Path
home = str(Path.home())


command = '0'
obj_dict = {'construction_barrel':0,'jersey_barrier':0}
keys = list(obj_dict)



def grid_spawn(command):
    obj_count = 0
    if command == '0':
        for i in range(-5,6,1):
            for j in range(-5,6,1):
                if abs(i)<1 and abs(j)<1:
                    pass
                else:
                    idx = randint(0,len(keys)-1)
                    # print(idx)
                    selected_obj = keys[idx]
                    with open(home+"/.gazebo/models/{}/model.sdf".format(selected_obj), "r") as f:
                        product_xml = f.read()
                        
                        print(obj_dict[selected_obj])

                        item_name   =   "obj_{}".format(obj_count)
                        obj_count+=1
                        # print("Spawning model:{}".format(item_name))
                        item_pose   =   Pose()
                        if selected_obj == 'dumpster':
                            item_pose_position = Point(i*8,j*8,0.5)
                            item_pose_orientation = Quaternion(uniform(0,1),0,0,uniform(0,1))
                        else:
                            item_pose_position = Point(i*8,j*8,0)
                            item_pose_orientation = Quaternion(0,0,uniform(0,1),uniform(0,1))
                        item_pose.position = item_pose_position
                        item_pose.orientation = item_pose_orientation

                        spawn_model(item_name, product_xml, "", item_pose, "world")

    else:
        for i in range(1000):
            name = "obj_{}".format(i)
            delete_model(name)



if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    obj_count = 0
    grid_spawn(command)