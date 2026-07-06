#!/usr/bin/env python3

import argparse
import sys

import geometry_msgs.msg
import moveit_commander
import rospy
from control_msgs.msg import GripperCommand


HOME = [
    0.6222648024559021, 0.10785019397735596, 0.13365104794502258,
    0.6424244046211243, 0.7651801109313965, -0.005664573982357979,
    0.04193020984530449
]

FEEDER_UP = [
    0.5981001853942871, -0.27141129970550537, -0.1264430582523346,
    -0.06331165879964828, 0.9979503154754639, 0.008563138544559479,
    -0.0036662763450294733
]

FEEDER_DOWN = [
    0.5995640754699707, -0.2726421058177948, -0.15186838805675507,
    -0.0978461354970932, 0.9948733448982239, 0.015090948902070522,
    -0.020625824108719826
]

COLUMN_TARGETS = {
    1: (
        [
            0.7659575343132019, -0.01928199827671051, 0.014277258887887001,
            0.6242639422416687, 0.779880702495575, -0.03841532766819,
            0.024594593793153763
        ],
        [
            0.7732441425323486, -0.017850466072559357, 0.0009029355715028942,
            0.6351404786109924, 0.7694917321205139, -0.05897089093923569,
            -0.031647223979234695
        ]
    ),
    4: (
        [
            0.7502648830413818, 0.0700092688202858, 0.01566367596387863,
            0.6320086121559143, 0.7744424939155579, -0.026563867926597595,
            0.009913882240653038
        ],
        [
            0.754968523979187, 0.0692480131983757, 0.003429300617426634,
            0.6444659233093262, 0.7636176943778992, -0.03401164710521698,
            -0.019871961325407028
        ]
    ),
    7: (
        [
            0.7321473956108093, 0.16258440911769867, 0.012277955189347267,
            0.6415701508522034, 0.7659030556678772, -0.01831873506307602,
            0.038008660078048706
        ],
        [
            0.7352566123008728, 0.1646074652671814, 0.0002626599743962288,
            0.6427019238471985, 0.764492392539978, -0.04835685342550278,
            0.012136378325521946
        ]
    ),
}


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

    if not success:
        rospy.logwarn("[panel_motion] MoveIt reported an unsuccessful move.")

    return success


def gripper(gripper_pub, position, max_effort=0.0):
    command = GripperCommand()
    command.position = position
    command.max_effort = max_effort
    gripper_pub.publish(command)
    rospy.sleep(0.25)


def pick_from_feeder(group, gripper_pub, wait_seconds):
    rospy.loginfo("[panel_motion] Moving to feeder up pose.")
    move(group, FEEDER_UP)

    rospy.loginfo("[panel_motion] Moving down to feeder pick pose.")
    move(group, FEEDER_DOWN)

    rospy.loginfo(f"[panel_motion] Waiting {wait_seconds:.1f} seconds before closing gripper.")
    rospy.sleep(wait_seconds)

    rospy.loginfo("[panel_motion] Closing gripper.")
    gripper(gripper_pub, 0.0)

    rospy.loginfo("[panel_motion] Moving back up from feeder.")
    move(group, FEEDER_UP)


def place_column(group, gripper_pub, column):
    target_1, target_2 = COLUMN_TARGETS[column]

    rospy.loginfo(f"[panel_motion] Moving to column {column}.")
    move(group, HOME)
    move(group, target_1)
    move(group, target_2)

    rospy.loginfo(f"[panel_motion] Releasing piece at column {column}.")
    gripper(gripper_pub, 1.0)

    rospy.loginfo("[panel_motion] Returning home.")
    move(group, HOME)

def move_home():
    move(HOME)


def parse_args():
    parser = argparse.ArgumentParser(description="Connect Four ScriptPanel robot motions.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    column_parser = subparsers.add_parser("column")
    column_parser.add_argument("column", type=int, choices=sorted(COLUMN_TARGETS))

    pick_parser = subparsers.add_parser("pick")
    pick_parser.add_argument("--wait", type=float, default=2.0)

    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    args = parse_args()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("connect_four_panel_motion", anonymous=True)

    group_name = rospy.get_param("~group_name", "arm")
    group = moveit_commander.MoveGroupCommander(group_name)
    gripper_pub = rospy.Publisher("/robot/gripper/command", GripperCommand, queue_size=10)

    if args.command == "column":
        place_column(group, gripper_pub, args.column)
    elif args.command == "pick":
        pick_from_feeder(group, gripper_pub, args.wait)
    elif args.command == "Home":
        move_home()
    rospy.loginfo("[panel_motion] Done.")


if __name__ == "__main__":
    main()
