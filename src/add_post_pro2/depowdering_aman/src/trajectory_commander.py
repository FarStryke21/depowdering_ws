#!/usr/bin/env python

import rospy
import numpy as np
import moveit_commander
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tft
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import sys
import pandas as pd
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import DeleteModelRequest


def transform_pose(input_pose, from_frame, to_frame):
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        transform = tf_buffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
        pose_transformed = tf2_geometry_msgs.do_transform_pose(input_pose, transform)
        return pose_transformed
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"Could not transform {from_frame} to {to_frame}: {e}")
        return None


def attach_object(model1 = "add_post_pro_depowdering", link1 = "gripper_Link", model2 = "cylinder_0", link2 = "link"):
    rospy.wait_for_service('/link_attacher_node/attach')
    
    try:
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2
        
        resp = attach_srv.call(req)
        if resp.ok:
            rospy.loginfo("Successfully attached object to gripper")
        else:
            rospy.logerr("Failed to attach object")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def detach_object(model1 = "add_post_pro_depowdering", link1 = "gripper_Link", model2 = "cylinder_0", link2 = "link"):
    rospy.wait_for_service('/link_attacher_node/detach')
    rospy.wait_for_service('/gazebo/delete_model')
    
    try:
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2
        
        resp = detach_srv.call(req)
        if resp.ok:
            rospy.loginfo("Successfully detached object from gripper")

        else:
            rospy.logerr("Failed to detach object")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def delete_model(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        req = DeleteModelRequest()
        req.model_name = model_name
        resp = delete_model(req)
        if resp.success:
            rospy.loginfo(f"Successfully deleted {model_name}")
        else:
            rospy.logerr(f"Failed to delete {model_name}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def motion_planner(scene = None, robot = None, object_id = 0, pick_pose=None, target_pose=None, arm_group=None, gripper_group=None):
    arm_group.set_start_state_to_current_state()
    gripper_group.set_start_state_to_current_state()

    # Ready Pose
    arm_group.set_named_target("ready")
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    # Pre grasp pose
    object_at = pick_pose.pose.position.z
    pick_pose.pose.position.z = object_at + 0.3
    # Set the target pose for the end effector
    rospy.loginfo("Moving to Pre Grasp Pose")
    arm_group.set_pose_target(pick_pose)
    arm_group.go(wait=True) 
    arm_group.stop()
    arm_group.clear_pose_targets()

    # Grasp Pose
    rospy.loginfo("Moving to Grasp Pose")
    pick_pose.pose.position.z = object_at + 0.18
    arm_group.set_pose_target(pick_pose)
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    # Close the gripper
    rospy.loginfo("Closing Gripper")
    gripper_group.set_named_target("pick")
    gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    attach_object(model1="add_post_pro_depowdering", link1="gripper_Link", model2=f"cylinder_{object_id}", link2="link")
    touch_links = robot.get_link_names()
    scene.attach_cylinder("J6", f"part_{object_id}", touch_links=touch_links)

    # Set the target pose for the end effector
    rospy.loginfo("Moving to Post Grasp Pose")
    pick_pose.pose.position.z = object_at + 0.3
    arm_group.set_pose_target(pick_pose)
    arm_group.go(wait=True) 
    arm_group.stop()
    arm_group.clear_pose_targets()

    # Waypoint
    arm_group.set_named_target("rest")
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    # Set the target pose for the end effector
    rospy.loginfo("Moving to Target Pose")
    arm_group.set_pose_target(target_pose)
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    # Open the gripper
    rospy.loginfo("Opening Gripper")
    detach_object(model1="add_post_pro_depowdering", link1="gripper_Link", model2=f"cylinder_{object_id}", link2="link")
    gripper_group.set_named_target("release")
    gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()

    scene.remove_attached_object("J6", name=f"part_{object_id}")

    # Return to the ready position

    arm_group.set_named_target("rest")
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    delete_model(f"cylinder_{object_id}")
    scene.remove_world_object(f"part_{object_id}")
    

def motion_commander(cylinder_pose):
    # Initialize the moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the robot commander
    robot = moveit_commander.RobotCommander()

    # Initialize the planning scene interface
    scene = moveit_commander.PlanningSceneInterface()

    # Initialize the move group commander
    arm_group = moveit_commander.MoveGroupCommander("arm")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")

    # Set the reference frame for the arm group
    arm_group.set_pose_reference_frame("base_link")
    gripper_group.set_pose_reference_frame("base_link")

    rospy.loginfo("Enter Ready Position")
    arm_group.set_named_target("rest")
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()
    gripper_group.set_named_target("release")
    gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()
    input("Start Motion...")

    for idx, p in enumerate(cylinder_pose, 1):
        pose = PoseStamped()
        pose.header.frame_id = "powder_box"
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]

        pose = transform_pose(pose, "powder_box", "world")

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 1.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = 0.0
        target_pose.pose.position.y = 0.3
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 1.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0

        motion_planner(object_id=idx, pick_pose=pose, target_pose=target_pose, arm_group=arm_group, gripper_group=gripper_group, scene=scene, robot=robot)  

        # if idx%9 == 0:
            # decrease the height of the sandbox model in gazebo by 0.025
        

    # Stop the moveit_commander
    moveit_commander.roscpp_shutdown()
    rospy.signal_shutdown("Motion Completed")


if __name__ == "__main__":
    # setup a global variable to trigger the callback
    trigger = False
    rospy.init_node('trajectory_commander', anonymous=True)
    file_path = rospy.get_param('object_pose_file', '/home/aman/Desktop/test_config.csv')
    # Read the csv file into a dataframe and extract the px, py, pz values
    df = pd.read_csv(file_path)
    cylinder_pose = df[['px', 'py', 'pz', 'ox', 'oy', 'oz', 'ow']].values
    # Transform the pose from the powder_box to the world frame

    motion_commander(cylinder_pose)
    rospy.spin()