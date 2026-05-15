#!/usr/bin/env python3

import sys

import geometry_msgs.msg
import moveit_commander
import rospy
from control_msgs.msg import GripperCommand


PRE_PICK = [
    0.5981001853942871, -0.27141129970550537, -0.1264430582523346,
    -0.06331165879964828, 0.9979503154754639, 0.008563138544559479,
    -0.0036662763450294733
]

PICK = [
    0.5995640754699707, -0.2726421058177948, -0.15186838805675507,
    -0.0978461354970932, 0.9948733448982239, 0.015090948902070522,
    -0.020625824108719826
]


def move(group, pose_values):
    px, py, pz, ox, oy, oz, ow = pose_values

    pose_target = geometry_msgs.msg.PoseStamped()
    pose_target.header.frame_id = "base"
    pose_target.pose.position.x = px
    pose_target.pose.position.y = py
    pose_target.pose.position.z = pz
    pose_target.pose.orientation.x = ox
    pose_target.pose.orientation.y = oy
    pose_target.pose.orientation.z = oz
    pose_target.pose.orientation.w = ow

    group.set_goal_position_tolerance(0.001)
    group.set_goal_orientation_tolerance(0.001)
    group.set_pose_target(pose_target)

    success = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()
    rospy.sleep(0.5)

    return success


def close_gripper(gripper_pub):
    command = GripperCommand()
    command.position = 0.0
    command.max_effort = 0.0
    gripper_pub.publish(command)
    rospy.sleep(0.25)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("panel_pick", anonymous=True)

    group_name = rospy.get_param("~group_name", "arm")
    wait_seconds = float(rospy.get_param("~wait_seconds", 2.0))
    group = moveit_commander.MoveGroupCommander(group_name)
    gripper_pub = rospy.Publisher("/robot/gripper/command", GripperCommand, queue_size=10)

    rospy.loginfo("[panel_pick] Moving to pre-pick pose.")
    move(group, PRE_PICK)

    rospy.loginfo("[panel_pick] Moving down to pick pose.")
    move(group, PICK)

    rospy.loginfo(f"[panel_pick] Waiting {wait_seconds:.1f} seconds before closing gripper.")
    rospy.sleep(wait_seconds)

    rospy.loginfo("[panel_pick] Closing gripper.")
    close_gripper(gripper_pub)

    rospy.loginfo("[panel_pick] Moving back to pre-pick pose.")
    move(group, PRE_PICK)

    rospy.loginfo("[panel_pick] Pick motion complete.")


if __name__ == "__main__":
    main()
