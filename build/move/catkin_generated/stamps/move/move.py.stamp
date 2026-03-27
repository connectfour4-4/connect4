import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)





print ("============ Generating plan 2")
pose_target = geometry_msgs.msg.PoseStamped()
pose_target.header.frame_id = "base"
pose_target.pose.position.x = 0.749823808670044
pose_target.pose.position.y = 0.06813254207372665
pose_target.pose.position.z = 0.017986731603741646
pose_target.pose.orientation.x = 0.6317886114120483
pose_target.pose.orientation.y = 0.7731607556343079
pose_target.pose.orientation.z = -0.04132567718625069
pose_target.pose.orientation.w = 0.0368478037416935

group.set_goal_position_tolerance(0.001)
group.set_goal_orientation_tolerance(0.001)
group.set_pose_target(pose_target)

success = group.go(wait=True)

group.stop()
group.clear_pose_targets()

print ("============ Generating plan 3")
pose_target = geometry_msgs.msg.PoseStamped()
pose_target.header.frame_id = "base"
pose_target.pose.position.x = 0.7517772912979126
pose_target.pose.position.y = 0.0694994330406189
pose_target.pose.position.z = 0.003656033193692565
pose_target.pose.orientation.x = 0.6371713280677795
pose_target.pose.orientation.y = 0.7697028517723083
pose_target.pose.orientation.z = -0.022248446941375732
pose_target.pose.orientation.w = 0.03279092162847519

group.set_goal_position_tolerance(0.0001)
group.set_goal_orientation_tolerance(0.0001)
group.set_pose_target(pose_target)

success = group.go(wait=True)

group.stop()
group.clear_pose_targets()

#pose_target.pose.position.z = 0.003656033193692565
#pose_taret.pose.orientation.z = -0.022248446941375732

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

#Position 1:
# x 0.7578372955322266"
# y  -0.09371642023324966"
#  z 0.00038133063935674727"
#  orientation
# x 0.7848950028419495
#  y  0.7848950028419495
 # z -0.015037784352898598
  # w 0.035182129591703415
