#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_object():
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        urdf_file = '/home/mit/workspaces/course_ws/src/moveit_robot_arm_sim/launch/box.urdf'
        x = 0
        y = -0.3
        z = 0.75
        model_name = 'my_box'

        # Define pose
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # Call the service
        spawn_model(model_name, open(urdf_file, 'r').read(), '', pose, 'world')
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('spawn_object_node')
    spawn_object()

