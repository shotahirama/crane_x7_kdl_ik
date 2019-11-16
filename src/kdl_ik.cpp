/* BSD 3-Clause License
 *
 * Copyright (c) 2019, Shota Hirama
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <angles/angles.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "kdl_ik");
  ros::NodeHandle nh;
  KDL::Tree tree;
  std::string robot_description_string;
  nh.getParam("robot_description", robot_description_string);
  if (!kdl_parser::treeFromString(robot_description_string, tree))
  {
    ROS_ERROR_STREAM("Failed to construct kdl tree");
    return 1;
  }

  std::vector<std::string> joint_names;
  nh.getParam("/crane_x7/arm_controller/joints", joint_names);

  KDL::Chain chain;
  tree.getChain("world", "crane_x7_gripper_base_link", chain);

  int n = chain.getNrOfJoints();
  KDL::ChainIkSolverPos_LMA solver(chain);

  // solver.display_information = true;

  KDL::JntArray q_init(n);
  KDL::JntArray q_sol(n);
  KDL::Frame p_out;
  p_out.p.x(0.0);
  p_out.p.y(0.0);
  p_out.p.z(0.32);
  p_out.M = KDL::Rotation::RPY(0.0, angles::from_degrees(90.0), 0.0);

  int ret = solver.CartToJnt(q_init, p_out, q_sol);
  if (ret != 0)
  {
    ROS_ERROR_STREAM("IK Failed");
    return 1;
  }

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/crane_x7/arm_controller/follow_joint_trajectory", true);
  if (!ac.waitForServer(ros::Duration(3.0)))
  {
    ROS_ERROR_STREAM("Cannot found action server");
    return 1;
  }

  trajectory_msgs::JointTrajectory jt;
  jt.joint_names = joint_names;
  trajectory_msgs::JointTrajectoryPoint p;
  for (int i = 0; i < n; i++)
  {
    p.positions.emplace_back(q_sol.data(i));
  }
  p.time_from_start = ros::Duration(3.0);
  jt.points.emplace_back(p);
  jt.header.stamp = ros::Time::now();
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = jt;
  ac.sendGoal(goal);
  ac.waitForResult(ros::Duration(10.0));

  return 0;
}
