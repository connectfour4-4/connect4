
import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg


def main():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("cartesian_pose_move", anonymous=True)

    move_group = moveit_commander.MoveGroupCommander("arm")

    move_group.set_goal_position_tolerance(0.001)
    move_group.set_goal_orientation_tolerance(0.001)

    rospy.sleep(1)

    waypoints = []

    start_pose = move_group.get_current_pose().pose
    waypoints.append(copy.deepcopy(start_pose))

    target_pose = geometry_msgs.msg.Pose()

    target_pose.position.x = 0.738538
    target_pose.position.y = 0.113139
    target_pose.position.z = 0.193612

    target_pose.orientation.x = -0.0212776
    target_pose.orientation.y = 0.998726
    target_pose.orientation.z = -0.0456447
    target_pose.orientation.w = 0.00324261

    waypoints.append(copy.deepcopy(target_pose))

    plan, fraction = move_group.compute_cartesian_path(
        waypoints,
        0.005,  
        True     
    )

    rospy.loginfo("Cartesian path fraction: %.3f", fraction)

    if fraction == 1.0:
        move_group.execute(plan, wait=True)
        move_group.stop()
    else:
        rospy.logwarn("not executing")

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
"""
**left most token slot
pose: 
  position: 
    x: 0.6091257978377673
    y: -0.15964220936731843
    z: 0.18910787070375706
  orientation: 
    x: -0.027730605688687248
    y: 0.9974459433053159
    z: -0.0646522032293881
    w: -0.01235703481801244


pose: 
  position: 
    x: 0.7408925685365271
    y: -0.13248088904947217
    z: 0.2289349524746532
  orientation: 
    x: -0.00583795665771432
    y: 0.9979679356962671
    z: -0.015892152996874697
    w: 0.061427657104258786

pose6: 
  position: 
    x: 0.5781650637814465
    y: -0.06098969719494808
    z: 0.18732481280633512
  orientation: 
    x: 0.9979318089704274
    y: 0.03929276913050164
    z: 0.03447114238735647
    w: -0.037415548660663524


    rosrun usb_cam usb_cam_node _pixel_format:="yuyv"


- Translation: [0.766, -0.115, 0.169]
- Rotation: in Quaternion [0.997, 0.012, -0.035, -0.063]
            in RPY (radian) [-3.015, 0.068, 0.029]
            in RPY (degree) [-172.749, 3.910, 1.645]


  Joint position: [-0.320494140625, 0.1315009765625, -0.4840390625, -0.10794140625, 1.09147265625, 0.212912109375, 1.08734375, -1.90830859375]

"""
