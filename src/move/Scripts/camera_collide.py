#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg

def main():
    rospy.init_node('add_colliders_in_planning_scene', anonymous=False)
    psi= moveit_commander.PlanningSceneInterface()

    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "base"
    table_pose.pose.position.x = 0.3
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = -0.19
    table_pose.pose.orientation.w = 1.0
    psi.add_box("box", pose=table_pose, size=(0.25, 0.25, 0.25))

    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "base"
    table_pose.pose.position.x = 0.82
    table_pose.pose.position.y = -0.1
    table_pose.pose.position.z = -0.275
    table_pose.pose.orientation.w = 1.0
    psi.add_box("table", pose=table_pose, size=(0.9, 1.1, 0.2))

    window_collider_pose = geometry_msgs.msg.PoseStamped()
    window_collider_pose.header.frame_id = "base"
    window_collider_pose.pose.position.x = 0.3
    window_collider_pose.pose.position.y = 0.6
    window_collider_pose.pose.position.z = 0.0
    window_collider_pose.pose.orientation.w = 1.0
    psi.add_box("window_collider", pose=window_collider_pose, size=(1.6, 0.2, 1.5))

if __name__ == "__main__":
    main()
