#!/usr/bin/env python3`

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
from geometry_msgs.msg import Pose, Twist

def check_model_exists(model_name):
    try:
        rospy.wait_for_service('/gazebo/get_model_state', timeout=5)
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state = get_model_state(model_name=model_name)
        return model_state.success
    except rospy.ServiceException as e:
        rospy.logerr("Failed to get model state: %s" % e)
        return False

def spawn_block():
    rospy.init_node('spawn_block_node', anonymous=True)
    
    # Check if the block model exists in Gazebo
    model_name = 'block'
    if not check_model_exists(model_name):
        rospy.logerr("Model '%s' does not exist in Gazebo." % model_name)
        return
    
    # Proceed with updating the model state
    rospy.wait_for_service('/gazebo/set_model_state')
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    block_state = ModelState()
    block_state.model_name = model_name
    block_state.pose.position.x = 1.0  # Adjust position as needed
    block_state.pose.position.y = 2.0
    block_state.pose.position.z = 0.5
    block_state.pose.orientation.x = 0.0
    block_state.pose.orientation.y = 0.0
    block_state.pose.orientation.z = 0.0
    block_state.pose.orientation.w = 1.0

    try:
        set_model_state(block_state)
        rospy.loginfo("Block spawned successfully!")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to spawn block: %s" % e)

if __name__ == '__main__':
    try:
        spawn_block()
    except rospy.ROSInterruptException:
        pass

