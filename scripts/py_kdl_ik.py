#!/usr/bin/env python
# coding: utf-8

"""
BSD 3-Clause License

Copyright (c) 2019, Shota Hirama
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import kdl_parser_py.urdf
import PyKDL as kdl
import math
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

if __name__ == "__main__":
    rospy.init_node("py_kdl_ik")
    robot_description = rospy.get_param("robot_description")
    (ok, tree) = kdl_parser_py.urdf.treeFromString(robot_description)
    joint_names = rospy.get_param("/crane_x7/arm_controller/joints")
    chain = tree.getChain("base_link", "crane_x7_gripper_base_link")
    solver = kdl.ChainIkSolverPos_LMA(chain)
    q_init = kdl.JntArray(chain.getNrOfJoints())
    p_out = kdl.Frame(kdl.Rotation.RPY(
        0.0, math.radians(90.0), 0.0), kdl.Vector(0.0, 0.0, 0.32))
    q_sol = kdl.JntArray(chain.getNrOfJoints())

    ret = solver.CartToJnt(q_init, p_out, q_sol)
    if ret != 0:
        rospy.logerr("IK Failed")
        rospy.signal_shutdown("IK Failed")

    ac = actionlib.SimpleActionClient(
        "/crane_x7/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    if not ac.wait_for_server(timeout=rospy.Duration(3.0)):
        rospy.logerr("Cannot found action server")
        rospy.signal_shutdown("Cannot found action server")

    jt = JointTrajectory()
    jt.joint_names = joint_names
    p = JointTrajectoryPoint()
    for q in q_sol:
        p.positions.append(q)
    p.time_from_start = rospy.Duration(3.0)
    jt.points.append(p)
    jt.header.stamp = rospy.Time.now()
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = jt

    ac.send_goal(goal)
    ac.wait_for_result(timeout=rospy.Duration(10.0))
