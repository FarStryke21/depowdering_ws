#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from sensor_msgs import point_cloud2
import std_msgs
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import sys
import pandas as pd

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


def attach_object(model1 = "ur5", link1 = "wrist_3_link", model2 = "blue_cylinder", link2 = "link_0"):
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

def detach_object(model1 = "ur5", link1 = "wrist_3_link", model2 = "blue_cylinder", link2 = "link_0"):
    rospy.wait_for_service('/link_attacher_node/detach')
    
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

def motion_planner(pick_pose=None, target_pose=None, arm_group=None, gripper_group=None):
    arm_group.set_start_state_to_current_state()
    gripper_group.set_start_state_to_current_state()

    # Pre grasp pose
    pick_pose.pose.position.z = 0.25
    # Set the target pose for the end effector
    arm_group.set_pose_target(pick_pose)
    arm_group.go(wait=True) 
    arm_group.stop()
    arm_group.clear_pose_targets()

    # Close the gripper
    gripper_group.set_named_target("pick")
    gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()

    # Set the target pose for the end effector
    arm_group.set_pose_target(target_pose)
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()

    # Open the gripper
    gripper_group.set_named_target("release")
    gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()

    # Return to the ready position
    arm_group.set_named_target("rest")
    arm_group.go(wait=True)
    arm_group.stop()
    arm_group.clear_pose_targets()



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

    # Set the reference frame for the gripper group
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

    for p in cylinder_pose:
        pose = PoseStamped()
        pose.header.frame_id = "powder_box"
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.x = 0.707
        pose.pose.orientation.y = 0.707
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0

        pose = transform_pose(pose, "powder_box", "world")

        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0

        print(pose)

        motion_planner(pick_pose=pose, target_pose=target_pose, arm_group=arm_group, gripper_group=gripper_group)



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