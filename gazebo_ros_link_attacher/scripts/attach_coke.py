#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    req = AttachRequest()
    req.model_name_1 = "mario_khada"
    req.link_name_1 = "arm_03_link_06_1"
    req.model_name_2 = "coke_can"
    req.link_name_2 = "link"

    attach_srv.call(req)