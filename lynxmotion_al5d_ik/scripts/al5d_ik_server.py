#!/usr/bin/env python

import math
import rospy
import tf
from std_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from lynxmotion_ik.srv import *

tf_listener = None # TODO add look up

def handle_do_ik(req):
  L_12 = 0.06985
  L_23 = 0.14605
  L_34 = 0.18500
  L_4G = 0.08730

  X = req.target.pose.position.x
  Y = req.target.pose.position.y
  Z = req.target.pose.position.z
  gripper_angle = req.gripper_angle

  try:
    j1 = math.atan2(Y,X)
    r = math.hypot(X,Y)
    j4 = gripper_angle
    rp = r - (L_4G * math.cos(j4))
    zp = Z - (L_4G * math.sin(j4)) - L_12
    beta = math.atan2(zp,rp)
    r0 = math.hypot(zp,rp)
    alpha = math.acos( (math.pow(r0,2) + math.pow(L_23,2) - math.pow(L_34,2)) / (2 * r0 * L_23) )
    j2 = alpha + beta
    gamma = math.acos( (math.pow(L_34,2)+math.pow(L_23,2)-math.pow(r0,2)) / (2*L_23*L_34) )
    j3 = (j2 + gamma) - math.pi
    j2 = j2 - math.pi/2 # The IK solution assumes that all angles are zero the the x axis
    return DoIKResponse(j1,j2,j3,j4)
  except ValueError:
    return None

def do_ik_server():
  rospy.init_node('do_ik_server')
  tf_listener = tf.TransformListener()
  s = rospy.Service('do_ik', DoIK, handle_do_ik)
  print "Ready to do IK."
  rospy.spin()

if __name__ == "__main__":
  try:
    do_ik_server()
  except rospy.ROSInterruptException:
    pass
