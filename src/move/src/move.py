import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import subprocess


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()


group_name = "arm"
group = moveit_commander.MoveGroupCommander(group_name)


while(1):

    
    user_input = int(input("Enter something  "))
    print(user_input)

    if user_input == 0:
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.3690747320652008
        pose_target.pose.position.y = -0.2617364823818207
        pose_target.pose.position.z = 0.2822362780570984
        pose_target.pose.orientation.x = 0.6308044791221619
        pose_target.pose.orientation.y = 0.7757319808006287
        pose_target.pose.orientation.z = -0.01561302226036787
        pose_target.pose.orientation.w = -0.009048229083418846

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()

    if user_input == 1:
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7659575343132019
        pose_target.pose.position.y = -0.01928199827671051
        pose_target.pose.position.z = 0.014277258887887001
        pose_target.pose.orientation.x = 0.6242639422416687
        pose_target.pose.orientation.y = 0.779880702495575
        pose_target.pose.orientation.z = -0.03841532766819
        pose_target.pose.orientation.w = 0.024594593793153763

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7732441425323486
        pose_target.pose.position.y = -0.017850466072559357
        pose_target.pose.position.z = 0.0009029355715028942
        pose_target.pose.orientation.x = 0.6351404786109924
        pose_target.pose.orientation.y = 0.7694917321205139
        pose_target.pose.orientation.z = -0.05897089093923569
        pose_target.pose.orientation.w = -0.031647223979234695

        group.set_goal_position_tolerance(0.0001)
        group.set_goal_orientation_tolerance(0.0001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
    
    if user_input == 2:
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7592899799346924
        pose_target.pose.position.y = 0.010645180009305477
        pose_target.pose.position.z = 0.018214048817753792
        pose_target.pose.orientation.x = 0.6324942111968994
        pose_target.pose.orientation.y = 0.7737475633621216
        pose_target.pose.orientation.z = -0.019390655681490898
        pose_target.pose.orientation.w = 0.029830126091837883

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7665234208106995
        pose_target.pose.position.y = 0.01130811870098114
        pose_target.pose.position.z = -0.0005442227120511234
        pose_target.pose.orientation.x = 0.629223644733429
        pose_target.pose.orientation.y = 0.7754307389259338
        pose_target.pose.orientation.z = -0.035582102835178375
        pose_target.pose.orientation.w = -0.03897073119878769

        group.set_goal_position_tolerance(0.0001)
        group.set_goal_orientation_tolerance(0.0001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()

    if user_input == 3:
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7543240785598755
        pose_target.pose.position.y = 0.04085736721754074
        pose_target.pose.position.z = 0.02014775015413761
        pose_target.pose.orientation.x = 0.6356566548347473
        pose_target.pose.orientation.y = 0.7706232070922852
        pose_target.pose.orientation.z = -0.02713153511285782
        pose_target.pose.orientation.w = 0.03666521981358528

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7603095769882202
        pose_target.pose.position.y = 0.04223603755235672
        pose_target.pose.position.z = 0.0007712734513916075
        pose_target.pose.orientation.x = 0.6543909907341003
        pose_target.pose.orientation.y = 0.75343918800354
        pose_target.pose.orientation.z = -0.062399979680776596
        pose_target.pose.orientation.w = -0.014424381777644157

        group.set_goal_position_tolerance(0.0001)
        group.set_goal_orientation_tolerance(0.0001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
    if user_input == 4:
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7502648830413818
        pose_target.pose.position.y = 0.0700092688202858
        pose_target.pose.position.z = 0.01566367596387863
        pose_target.pose.orientation.x = 0.6320086121559143
        pose_target.pose.orientation.y = 0.7744424939155579
        pose_target.pose.orientation.z = -0.026563867926597595
        pose_target.pose.orientation.w = 0.009913882240653038

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.754968523979187
        pose_target.pose.position.y = 0.0692480131983757
        pose_target.pose.position.z = 0.003429300617426634
        pose_target.pose.orientation.x = 0.6444659233093262
        pose_target.pose.orientation.y = 0.7636176943778992
        pose_target.pose.orientation.z = -0.03401164710521698
        pose_target.pose.orientation.w = -0.019871961325407028

        group.set_goal_position_tolerance(0.0001)
        group.set_goal_orientation_tolerance(0.0001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
    if user_input == 5:
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7449400424957275
        pose_target.pose.position.y = 0.10134948045015335
        pose_target.pose.position.z = 0.015208045020699501
        pose_target.pose.orientation.x = 0.6462280750274658
        pose_target.pose.orientation.y = 0.7617527842521667
        pose_target.pose.orientation.z = -0.0379912294447422
        pose_target.pose.orientation.w = 0.02605009824037552

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.749354362487793
        pose_target.pose.position.y = 0.10310576111078262
        pose_target.pose.position.z = 0.0012894036481156945
        pose_target.pose.orientation.x = 0.6414786577224731
        pose_target.pose.orientation.y = 0.7638185024261475
        pose_target.pose.orientation.z = -0.0665523111820221
        pose_target.pose.orientation.w = -0.02563573606312275

        group.set_goal_position_tolerance(0.0001)
        group.set_goal_orientation_tolerance(0.0001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
    if user_input == 6:
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7401407361030579
        pose_target.pose.position.y = 0.13563458621501923
        pose_target.pose.position.z = 0.023469742387533188
        pose_target.pose.orientation.x = 0.6525406241416931
        pose_target.pose.orientation.y = 0.7573562264442444
        pose_target.pose.orientation.z = -0.02328832447528839
        pose_target.pose.orientation.w = 0.007742318790405989

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.739874541759491
        pose_target.pose.position.y = 0.1324497014284134
        pose_target.pose.position.z = 0.00027949680224992335
        pose_target.pose.orientation.x = 0.6590002179145813
        pose_target.pose.orientation.y = 0.7514855861663818
        pose_target.pose.orientation.z = -0.019827088341116905
        pose_target.pose.orientation.w = 0.024392297491431236

        group.set_goal_position_tolerance(0.0001)
        group.set_goal_orientation_tolerance(0.0001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
    if user_input == 7:
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7321473956108093
        pose_target.pose.position.y = 0.16258440911769867
        pose_target.pose.position.z = 0.012277955189347267
        pose_target.pose.orientation.x = 0.6415701508522034
        pose_target.pose.orientation.y = 0.7659030556678772
        pose_target.pose.orientation.z = -0.01831873506307602
        pose_target.pose.orientation.w = 0.038008660078048706

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.7352566123008728
        pose_target.pose.position.y = 0.1646074652671814
        pose_target.pose.position.z = 0.0002626599743962288
        pose_target.pose.orientation.x = 0.6427019238471985
        pose_target.pose.orientation.y = 0.764492392539978
        pose_target.pose.orientation.z = -0.04835685342550278
        pose_target.pose.orientation.w = 0.012136378325521946

        group.set_goal_position_tolerance(0.0001)
        group.set_goal_orientation_tolerance(0.0001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
    

    if user_input == 8:
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.5981001853942871
        pose_target.pose.position.y = -0.27141129970550537
        pose_target.pose.position.z = -0.1264430582523346
        pose_target.pose.orientation.x = -0.06331165879964828
        pose_target.pose.orientation.y = 0.9979503154754639
        pose_target.pose.orientation.z = 0.008563138544559479
        pose_target.pose.orientation.w = -0.0036662763450294733

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        rospy.sleep(0.5)
        subprocess.run(
            "rostopic pub -1 /robot/gripper/command control_msgs/GripperCommand '{position: 1.0, max_effort: 0.0}'",
            shell=True
        )
        group.clear_pose_targets()
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.5995640754699707
        pose_target.pose.position.y = -0.2726421058177948
        pose_target.pose.position.z = -0.15186838805675507
        pose_target.pose.orientation.x = -0.0978461354970932
        pose_target.pose.orientation.y = 0.9948733448982239
        pose_target.pose.orientation.z = 0.015090948902070522
        pose_target.pose.orientation.w = -0.020625824108719826

        group.set_goal_position_tolerance(0.0001)
        group.set_goal_orientation_tolerance(0.0001)
        group.set_pose_target(pose_target)

        success = group.go(wait=True)

        group.stop()
        group.clear_pose_targets()
        ####################################################
        

        
        rospy.sleep(0.5)
        subprocess.run(
            "rostopic pub -1 /robot/gripper/command control_msgs/GripperCommand '{position: 0.0, max_effort: 0.0}'",
            shell=True
        )
        ###############################
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "base"
        pose_target.pose.position.x = 0.5981001853942871
        pose_target.pose.position.y = -0.27141129970550537
        pose_target.pose.position.z = -0.1264430582523346
        pose_target.pose.orientation.x = -0.06331165879964828
        pose_target.pose.orientation.y = 0.9979503154754639
        pose_target.pose.orientation.z = 0.008563138544559479
        pose_target.pose.orientation.w = -0.0036662763450294733

        group.set_goal_position_tolerance(0.001)
        group.set_goal_orientation_tolerance(0.001)
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
