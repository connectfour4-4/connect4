import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import control_msgs.msg
from control_msgs.msg import GripperCommand
from std_msgs.msg import Bool
import subprocess

from std_msgs.msg import Int8

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_sawyer',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)

move_done_pub = rospy.Publisher("/robot/move_done", Bool, queue_size=10)


def move (px, py, pz, ox, oy, oz, ow):
    pose_target = geometry_msgs.msg.PoseStamped()
    pose_target.header.frame_id = "base"
    pose_target.pose.position.x = px
    pose_target.pose.position.y = py
    pose_target.pose.position.z = pz
    pose_target.pose.orientation.x = ox
    pose_target.pose.orientation.y =  oy
    pose_target.pose.orientation.z = oz
    pose_target.pose.orientation.w = ow

    group.set_goal_position_tolerance(0.001)
    group.set_goal_orientation_tolerance(0.001)
    group.set_pose_target(pose_target)

    success = group.go(wait=True)

    group.stop()
    group.clear_pose_targets()
    rospy.sleep(0.5)


def pnp(target_1, target_2):
    move(*col8_1)
    move(*col8_2)

    subprocess.run(
        "rostopic pub -1 /robot/gripper/command control_msgs/GripperCommand '{position: 0.0, max_effort: 0.0}'",
        shell=True
    )

    move(*col8_1)
    move(*home)

    move(*target_1)
    move(*target_2)

    subprocess.run(
        "rostopic pub -1 /robot/gripper/command control_msgs/GripperCommand '{position: 1.0, max_effort: 0.0}'",
        shell=True
    )

    move(*home)
    rospy.sleep(0.5)
    move_done_pub.publish(True)


home = [0.6222648024559021, 0.10785019397735596, 0.13365104794502258,
        0.6424244046211243, 0.7651801109313965, -0.005664573982357979, 0.04193020984530449]

col8_1 = [0.5981001853942871, -0.27141129970550537, -0.1264430582523346,
          -0.06331165879964828, 0.9979503154754639, 0.008563138544559479, -0.0036662763450294733]

col8_2 = [0.5995640754699707, -0.2726421058177948, -0.15186838805675507,
          -0.0978461354970932, 0.9948733448982239, 0.015090948902070522, -0.020625824108719826]


def robot_next_move_callback(msg):
    user_input = msg.data
    print(user_input)

    if user_input == 7:
        col1_1 = [0.7659575343132019, -0.01928199827671051, 0.014277258887887001,
                  0.6242639422416687, 0.779880702495575, -0.03841532766819, 0.024594593793153763]

        col1_2 = [0.7732441425323486, -0.017850466072559357, 0.0009029355715028942,
                  0.6351404786109924, 0.7694917321205139, -0.05897089093923569, -0.031647223979234695]
        pnp(col1_1, col1_2)

    if user_input == 6:
        col2_1 = [0.7592899799346924, 0.010645180009305477, 0.018214048817753792,
                  0.6324942111968994, 0.7737475633621216, -0.019390655681490898, 0.029830126091837883]

        col2_2 = [0.7665234208106995, 0.01130811870098114, -0.0005442227120511234,
                  0.629223644733429, 0.7754307389259338, -0.035582102835178375, -0.03897073119878769]
        pnp(col2_1, col2_2)

    if user_input == 5:
        col3_1 = [0.7543240785598755, 0.04085736721754074, 0.02014775015413761,
                  0.6356566548347473, 0.7706232070922852, -0.02713153511285782, 0.03666521981358528]

        col3_2 = [0.7603095769882202, 0.04223603755235672, 0.0007712734513916075,
                  0.6543909907341003, 0.75343918800354, -0.062399979680776596, -0.014424381777644157]
        pnp(col3_1, col3_2)

    if user_input == 4:
        col4_1 = [0.7502648830413818, 0.0700092688202858, 0.01566367596387863,
                  0.6320086121559143, 0.7744424939155579, -0.026563867926597595, 0.009913882240653038]

        col4_2 = [0.754968523979187, 0.0692480131983757, 0.003429300617426634,
                  0.6444659233093262, 0.7636176943778992, -0.03401164710521698, -0.019871961325407028]
        pnp(col4_1, col4_2)

    if user_input == 3:
        col5_1 = [0.7449400424957275, 0.10134948045015335, 0.015208045020699501,
                  0.6462280750274658, 0.7617527842521667, -0.0379912294447422, 0.02605009824037552]

        col5_2 = [0.749354362487793, 0.10310576111078262, 0.0012894036481156945,
                  0.6414786577224731, 0.7638185024261475, -0.0665523111820221, -0.02563573606312275]
        pnp(col5_1, col5_2)

    if user_input == 2:
        col6_1 = [0.7401407361030579, 0.13563458621501923, 0.023469742387533188,
                  0.6525406241416931, 0.7573562264442444, -0.02328832447528839, 0.007742318790405989]

        col6_2 = [0.739874541759491, 0.1324497014284134, 0.00027949680224992335,
                  0.6590002179145813, 0.7514855861663818, -0.019827088341116905, 0.024392297491431236]
        pnp(col6_1, col6_2)

    if user_input == 1:
        col7_1 = [0.7321473956108093, 0.16258440911769867, 0.012277955189347267,
                  0.6415701508522034, 0.7659030556678772, -0.01831873506307602, 0.038008660078048706]

        col7_2 = [0.7352566123008728, 0.1646074652671814, 0.0002626599743962288,
                  0.6427019238471985, 0.764492392539978, -0.04835685342550278, 0.012136378325521946]
        pnp(col7_1, col7_2)


rospy.Subscriber('/robot_next_move', Int8, robot_next_move_callback, queue_size=1)

rospy.spin()