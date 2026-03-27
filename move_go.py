import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


print ("============ Starting tutorial setup")
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)


print ("============ Generating plan 1")
pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = -0.03172256052494049
pose_target.orientation.x = 0.7253731489181519
pose_target.orientation.y = -0.6868928074836731
pose_target.orientation.z = -0.03480038419365883
pose_target.position.x = 0.7821388840675354
pose_target.position.y = -0.05997581034898758
pose_target.position.z = 0.024752629920840263
group.set_pose_target(pose_target)

#plan1 = group.go()


success = group.go(wait=True)



# group.stop()
# print (robot.get_group_names())
group.clear_pose_targets()
# added from gpt
# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

# print ("============ Visualizing plan1")
# display_trajectory = moveit_msgs.msg.DisplayTrajectory()

# display_trajectory.trajectory_start = robot.get_current_state()
# display_trajectory.trajectory.append(go1)
# display_trajectory_publisher.publish(display_trajectory)

# print ("============ Waiting while plan1 is visualized (again)...")
# rospy.sleep(5)

"""

- Translation: [0.766, -0.115, 0.169]
- Rotation: in Quaternion [0.997, 0.012, -0.035, -0.063]
            in RPY (radian) [-3.015, 0.068, 0.029]
            in RPY (degree) [-172.749, 3.910, 1.645]

"""
