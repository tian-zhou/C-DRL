#!/usr/bin/env python
import sys, rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Quaternion, Point, Pose

if __name__ == '__main__':
    # init everything
    rospy.init_node("spawn_object")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    # prepare workspace pose
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    workspace_pose = Pose(Point(0.5,0,0.45), orient)

    # spawn workspace
    with open("../sdf/workspace.sdf", "r") as f:
        workspace_xml = f.read()
    delete_model("workspace")
    print spawn("workspace", workspace_xml, "", workspace_pose, "world")


    # prepare cube pose
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    cube_pose = Pose(Point(0.5,0,0.91), orient)
    
    # spawn cube
    with open("../sdf/cube.sdf", "r") as f:
        cube_xml = f.read()
    delete_model("cube")
    print spawn("cube", cube_xml, "", cube_pose, "world")

    