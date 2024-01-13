#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Empty
import math
from control_msgs.srv import (QueryTrajectoryState, QueryTrajectoryStateRequest, QueryTrajectoryStateResponse)
import time

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from copy import deepcopy
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose
import rospkg

def attach_models(model_1, link_1, model_2, link_2):
    rospy.loginfo("Attaching {} and {}".format(model_1, model_2))
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    req = AttachRequest()
    req.model_name_1 = model_1
    req.link_name_1 = link_1
    req.model_name_2 = model_2
    req.link_name_2 = link_2
    attach_srv.call(req)

def detach_models(model_1, link_1, model_2, link_2):
    rospy.loginfo("Detaching {} and {}".format(model_1, model_2))
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
    attach_srv.wait_for_service()
    req = AttachRequest()
    req.model_name_1 = model_1
    req.link_name_1 = link_1
    req.model_name_2 = model_2
    req.link_name_2 = link_2
    attach_srv.call(req)

rospack = rospkg.RosPack()
package_path = rospack.get_path('srs_modules_description')
file_path = package_path + '/meshes/srs_module.sdf'
model_xml_path = open(file_path, 'r').read()

def spawn_srs(i, main_body):
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(
    model_name='srs_module_' + str(i),
    model_xml=model_xml_path,
    # Enter the directory name to your system's workspace directory along with the username
    # model_xml=open('/home/peraspera/mario_ws/src/srs_modules_description/meshes/srs_module.sdf', 'r').read(),
    robot_namespace='/spawn_srs_ns',
    initial_pose=Pose(),
    reference_frame= main_body #'world' # Change the world frame to the main SRS 5 singlebody
    )

def move_forward(main_body):
    #### attach the left arm to the srs    
    attach_models("mario_so2", "arm_02_link_06_1", main_body, "base_link")

    #### Move the right arm up
    group = moveit_commander.MoveGroupCommander("arm3")
    group.set_named_target("lift")
    plan1 = group.plan()
    #rospy.sleep(1)
    group.go(wait=True)
    
    time.sleep(1)

    #### Move the left arm so the whole robot goes down
    group = moveit_commander.MoveGroupCommander("arm2")
    group.set_named_target("lift")
    plan1 = group.plan()
    group.go(wait=True)
    
    time.sleep(1)

    #### Detach the left arm and attach the right arm
    detach_models("mario_so2", "arm_02_link_06_1", main_body, "base_link")
    attach_models("mario_so2", "arm_03_link_06_1", main_body, "base_link")

    #### Move the right arm to standing position
    group = moveit_commander.MoveGroupCommander("arm3")
    group.set_named_target("stand")
    plan1 = group.plan()
    group.go(wait=True)

    time.sleep(1)

    #### Move the left arm to the standing position
    group = moveit_commander.MoveGroupCommander("arm2")
    group.set_named_target("stand")
    plan1 = group.plan()
    group.go(wait=True)

    #### attach the model 
    detach_models("mario_so2", "arm_03_link_06_1", main_body, "base_link")

    time.sleep(1)

def move_reverse(main_body):
     #### attach the left arm to the srs    
    attach_models("mario_so2", "arm_03_link_06_1", main_body, "base_link")

    #### Move the left arm up
    group = moveit_commander.MoveGroupCommander("arm2")
    group.set_named_target("lift")
    plan1 = group.plan()
    #rospy.sleep(1)
    group.go(wait=True)
    
    time.sleep(1)

    #### Move the right arm so the whole robot goes down
    group = moveit_commander.MoveGroupCommander("arm3")
    group.set_named_target("lift")
    plan1 = group.plan()
    group.go(wait=True)
    
    time.sleep(1)

    #### detach the right arm and attach the left arm
    detach_models("mario_so2", "arm_03_link_06_1", main_body, "base_link")
    attach_models("mario_so2", "arm_02_link_06_1", main_body, "base_link")

    #### Move the left arm to standing position
    group = moveit_commander.MoveGroupCommander("arm2")
    group.set_named_target("stand")
    plan1 = group.plan()
    group.go(wait=True)

    time.sleep(1)

    #### Move the right arm to the standing position
    group = moveit_commander.MoveGroupCommander("arm3")
    group.set_named_target("stand")
    plan1 = group.plan()
    group.go(wait=True)

    #### detach the arm from srs 
    detach_models("mario_so2", "arm_02_link_06_1", main_body, "base_link")

    time.sleep(1)

def assemble(j, main_body):
    
    # Attach the right arm to the assembled srs structure
    if j < 1:
        attach_models("mario_so2", "arm_02_link_06_1", main_body, "base_link")
    else:
        attach_models("mario_so2", "arm_02_link_06_1", "srs_module_" + str(j-1), "base_link")

    ##### Move the left arm out of the way, to home pose
    group = moveit_commander.MoveGroupCommander("arm2")
    group.set_named_target("attach_pose_bow")
    plan1 = group.plan()
    group.go(wait=True)

    time.sleep(1)

    #### Move the right arm to initate the attaching process 
    group = moveit_commander.MoveGroupCommander("arm3")
    group.set_named_target("attach_pose")
    plan1 = group.plan()
    group.go(wait=True)

    time.sleep(1)

    # ##### Move the top arm to attaching position
    group = moveit_commander.MoveGroupCommander("arm1")
    group.set_named_target("attach_pose_place")
    plan1 = group.plan()
    group.go(wait=True)

    time.sleep(1)
    
    # ##### Move the top arm to attaching position
    group = moveit_commander.MoveGroupCommander("arm1")
    group.set_named_target("attach_pose_dock")
    plan1 = group.plan()
    group.go(wait=True)

    time.sleep(1)

    #### detach the srs from arm and attach it to the structure
    detach_models("mario_so2", "arm_01_link_06_1", "srs_module_" + str(j), "base_link")
    attach_models(main_body, "base_link", "srs_module_" + str(j), "base_link")

    #### Move the right arm to standing position 
    group = moveit_commander.MoveGroupCommander("arm2")
    group.set_named_target("stand")
    plan1 = group.plan()
    group.go(wait=True)

    ##### Move the top arm to standing position
    group = moveit_commander.MoveGroupCommander("arm1")
    group.set_named_target("stand")
    plan1 = group.plan()
    group.go(wait=True)

    detach_models("mario_so2", "arm_02_link_06_1", main_body, "base_link")


if __name__ == '__main__':
    try:
        main_body = "srs_5_singlebody"
        total_SRS_to_be_spawned = 2     # Enter the desired number of SRS tiles to be attached
        rospy.init_node('mario_assembler')

        time.sleep(1)
        
        detach_models("mario_so2", "arm_02_link_06_1", main_body, "base_link")
        detach_models("mario_so2", "arm_03_link_06_1", main_body, "base_link")

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

        for j in range(total_SRS_to_be_spawned):

            spawn_srs(j, main_body)
            
            #### Attach the top arm to the srs
            attach_models("mario_so2", "arm_01_link_06_1", "srs_module_" + str(j), "base_link")

            for i in range(3):
                move_forward(main_body)

            for i in range(j+3):
                move_reverse(main_body)             

            assemble(j,main_body)

            for i in range(j):
                move_forward(main_body)

        moveit_commander.roscpp_shutdown()

    except rospy.ROSInterruptException:
        pass
