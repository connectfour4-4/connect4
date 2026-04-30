#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import subprocess

from std_msgs.msg import Int8

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_sawyer", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "arm_local"
group = moveit_commander.MoveGroupCommander(group_name)

is_moving = False


def terminal_log(title, text=""):
    msg = (
        f"\n================ {title} ================\n"
        f"{text}\n"
        f"=========================================\n"
    )
    print(msg, flush=True)
    rospy.loginfo(msg)


home = [
    0.6222648024559021, 0.10785019397735596, 0.13365104794502258,
    0.6424244046211243, 0.7651801109313965, -0.005664573982357979,
    0.04193020984530449
]

col8_1 = [
    0.5981001853942871, -0.27141129970550537, -0.1264430582523346,
    -0.06331165879964828, 0.9979503154754639, 0.008563138544559479,
    -0.0036662763450294733
]

col8_2 = [
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

    2: (
        [
            0.7592899799346924, 0.010645180009305477, 0.018214048817753792,
            0.6324942111968994, 0.7737475633621216, -0.019390655681490898,
            0.029830126091837883
        ],
        [
            0.7665234208106995, 0.01130811870098114, -0.0005442227120511234,
            0.629223644733429, 0.7754307389259338, -0.035582102835178375,
            -0.03897073119878769
        ]
    ),

    3: (
        [
            0.7543240785598755, 0.04085736721754074, 0.02014775015413761,
            0.6356566548347473, 0.7706232070922852, -0.02713153511285782,
            0.03666521981358528
        ],
        [
            0.7603095769882202, 0.04223603755235672, 0.0007712734513916075,
            0.6543909907341003, 0.75343918800354, -0.062399979680776596,
            -0.014424381777644157
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

    5: (
        [
            0.7449400424957275, 0.10134948045015335, 0.015208045020699501,
            0.6462280750274658, 0.7617527842521667, -0.0379912294447422,
            0.02605009824037552
        ],
        [
            0.749354362487793, 0.10310576111078262, 0.0012894036481156945,
            0.6414786577224731, 0.7638185024261475, -0.0665523111820221,
            -0.02563573606312275
        ]
    ),

    6: (
        [
            0.7401407361030579, 0.13563458621501923, 0.023469742387533188,
            0.6525406241416931, 0.7573562264442444, -0.02328832447528839,
            0.007742318790405989
        ],
        [
            0.739874541759491, 0.1324497014284134, 0.00027949680224992335,
            0.6590002179145813, 0.7514855861663818, -0.019827088341116905,
            0.024392297491431236
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


def move(px, py, pz, ox, oy, oz, ow):
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

    rospy.loginfo(
        "[move_sawyer] Moving to pose: "
        f"pos=({px:.3f}, {py:.3f}, {pz:.3f}), "
        f"ori=({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f})"
    )

    success = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()

    rospy.sleep(0.5)

    if success:
        rospy.loginfo("[move_sawyer] Pose move successful.")
    else:
        rospy.logwarn("[move_sawyer] Pose move may have failed.")

    return success


def close_gripper():
    terminal_log("MOVE NODE: CLOSING GRIPPER", "Closing gripper to pick piece.")
    subprocess.run(
        "rostopic pub -1 /robot/gripper/command control_msgs/GripperCommand "
        "'{position: 0.0, max_effort: 0.0}'",
        shell=True
    )


def open_gripper():
    terminal_log("MOVE NODE: OPENING GRIPPER", "Opening gripper to release piece.")
    subprocess.run(
        "rostopic pub -1 /robot/gripper/command control_msgs/GripperCommand "
        "'{position: 1.0, max_effort: 0.0}'",
        shell=True
    )


def pnp(target_1, target_2, selected_col):
    terminal_log(
        "MOVE NODE: STARTING PICK AND PLACE",
        f"Executing robot move for column {selected_col}."
    )

    move(*col8_1)
    move(*col8_2)

    close_gripper()

    move(*col8_1)
    move(*home)

    move(*target_1)
    move(*target_2)

    open_gripper()

    move(*home)

    terminal_log(
        "MOVE NODE: MOVE SUCCESSFUL",
        f"Robot completed pick and place for column {selected_col}."
    )


def keyboard_loop():
    terminal_log(
        "MOVE NODE: READY",
        "Enter a column number from 1 to 7.\nType q to quit."
    )

    while not rospy.is_shutdown():
        user_text = input("Enter column 1-7, or q to quit: ").strip()

        if user_text.lower() in ["q", "quit", "exit"]:
            terminal_log("MOVE NODE: EXITING", "Keyboard loop stopped.")
            break

        try:
            user_input = int(user_text)
        except ValueError:
            rospy.logwarn("[move_sawyer] Invalid input. Please enter a number from 1 to 7.")
            continue

        terminal_log(
            "MOVE NODE: RECEIVED KEYBOARD COMMAND",
            f"Column entered: {user_input}"
        )

        if user_input not in COLUMN_TARGETS:
            rospy.logwarn(f"[move_sawyer] Invalid column {user_input}. Expected 1 to 7.")
            continue

        try:
            target_1, target_2 = COLUMN_TARGETS[user_input]
            pnp(target_1, target_2, user_input)

        except Exception as e:
            terminal_log(
                "MOVE NODE: MOVE FAILED",
                f"Column: {user_input}\nError: {e}"
            )
            rospy.logerr(f"[move_sawyer] Move failed: {e}")

        rospy.loginfo("[move_sawyer] Ready for next keyboard command.")


keyboard_loop()


terminal_log("MOVE NODE: READY", "Listening on /robot_next_move.")
rospy.spin()