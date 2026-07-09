/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

// ROS 2 / MoveIt 2 port of the TRAC-IK MoveIt kinematics plugin.

#include <trac_ik/trac_ik_kinematics_plugin.hpp>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_model/robot_model.h>
#include <trac_ik/trac_ik.hpp>
#include <urdf/model.h>

namespace trac_ik_kinematics_plugin
{

namespace
{
rclcpp::Logger getLogger()
{
  return rclcpp::get_logger("trac_ik_kinematics_plugin");
}

KDL::Frame poseMsgToKDL(const geometry_msgs::msg::Pose& pose)
{
  return KDL::Frame(
    KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w),
    KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::msg::Pose kdlToPoseMsg(const KDL::Frame& frame)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = frame.p.x();
  pose.position.y = frame.p.y();
  pose.position.z = frame.p.z();
  frame.M.GetQuaternion(pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
  return pose;
}
}  // namespace

bool TRAC_IKKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node,
                                         const moveit::core::RobotModel& robot_model,
                                         const std::string& group_name,
                                         const std::string& base_frame,
                                         const std::vector<std::string>& tip_frames,
                                         double search_discretization)
{
  node_ = node;
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  if (tip_frames.size() != 1)
  {
    RCLCPP_ERROR(getLogger(), "TRAC-IK supports exactly one tip frame; got %zu for group '%s'",
                 tip_frames.size(), group_name.c_str());
    return false;
  }

  const urdf::ModelInterfaceSharedPtr& robot_urdf = robot_model.getURDF();
  if (!robot_urdf)
  {
    RCLCPP_ERROR(getLogger(), "Robot model has no URDF; cannot initialize TRAC-IK for group '%s'",
                 group_name.c_str());
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*robot_urdf, tree))
  {
    RCLCPP_ERROR(getLogger(), "Failed to extract KDL tree from the URDF robot description");
    return false;
  }

  if (!tree.getChain(base_frame_, tip_frames_[0], chain_))
  {
    RCLCPP_ERROR(getLogger(), "Couldn't find KDL chain %s -> %s", base_frame_.c_str(),
                 tip_frames_[0].c_str());
    return false;
  }

  num_joints_ = chain_.getNrOfJoints();

  joint_min_.resize(num_joints_);
  joint_max_.resize(num_joints_);

  unsigned int joint_num = 0;
  for (const KDL::Segment& segment : chain_.segments)
  {
    link_names_.push_back(segment.getName());

    urdf::JointConstSharedPtr joint = robot_urdf->getJoint(segment.getJoint().getName());
    if (!joint || joint->type == urdf::Joint::UNKNOWN || joint->type == urdf::Joint::FIXED)
    {
      continue;
    }

    joint_num++;
    joint_names_.push_back(joint->name);

    double lower = std::numeric_limits<float>::lowest();
    double upper = std::numeric_limits<float>::max();
    if (joint->type != urdf::Joint::CONTINUOUS && joint->limits)
    {
      if (joint->safety)
      {
        lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
        upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
      }
      else
      {
        lower = joint->limits->lower;
        upper = joint->limits->upper;
      }
    }
    joint_min_(joint_num - 1) = lower;
    joint_max_(joint_num - 1) = upper;
    RCLCPP_DEBUG(getLogger(), "IK using joint %s: [%g, %g]", joint->name.c_str(), lower, upper);
  }

  lookupParam(node_, "position_only_ik", position_ik_, false);
  lookupParam(node_, "solve_type", solve_type_, std::string("Speed"));
  lookupParam(node_, "epsilon", epsilon_, 1e-5);

  RCLCPP_INFO(getLogger(),
              "TRAC-IK ready for group '%s': chain %s -> %s (%u joints), "
              "solve_type=%s, epsilon=%g, position_only_ik=%s",
              group_name.c_str(), base_frame_.c_str(), tip_frames_[0].c_str(), num_joints_,
              solve_type_.c_str(), epsilon_, position_ik_ ? "true" : "false");

  active_ = true;
  return true;
}

int TRAC_IKKinematicsPlugin::getKDLSegmentIndex(const std::string& name) const
{
  int i = 0;
  while (i < static_cast<int>(chain_.getNrOfSegments()))
  {
    if (chain_.getSegment(i).getName() == name)
    {
      return i + 1;
    }
    i++;
  }
  return -1;
}

bool TRAC_IKKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                            const std::vector<double>& joint_angles,
                                            std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (!active_)
  {
    RCLCPP_ERROR(getLogger(), "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != num_joints_)
  {
    RCLCPP_ERROR(getLogger(), "Joint angles vector must have size: %u", num_joints_);
    return false;
  }

  KDL::JntArray jnt_pos_in(num_joints_);
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  KDL::ChainFkSolverPos_recursive fk_solver(chain_);

  bool valid = true;
  KDL::Frame p_out;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    if (fk_solver.JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >= 0)
    {
      poses[i] = kdlToPoseMsg(p_out);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "Could not compute FK for %s", link_names[i].c_str());
      valid = false;
    }
  }

  return valid;
}

bool TRAC_IKKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                            const std::vector<double>& ik_seed_state,
                                            std::vector<double>& solution,
                                            moveit_msgs::msg::MoveItErrorCodes& error_code,
                                            const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = nullptr;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, solution, solution_callback,
                          error_code, consistency_limits, options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state,
                                               double timeout,
                                               std::vector<double>& solution,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = nullptr;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state,
                                               double timeout,
                                               const std::vector<double>& consistency_limits,
                                               std::vector<double>& solution,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = nullptr;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state,
                                               double timeout,
                                               std::vector<double>& solution,
                                               const IKCallbackFn& solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state,
                                               double timeout,
                                               const std::vector<double>& consistency_limits,
                                               std::vector<double>& solution,
                                               const IKCallbackFn& solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool TRAC_IKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                               const std::vector<double>& ik_seed_state,
                                               double timeout,
                                               std::vector<double>& solution,
                                               const IKCallbackFn& solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code,
                                               const std::vector<double>& /*consistency_limits*/,
                                               const kinematics::KinematicsQueryOptions& /*options*/) const
{
  if (!active_)
  {
    RCLCPP_ERROR(getLogger(), "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != num_joints_)
  {
    RCLCPP_ERROR(getLogger(), "Seed state must have size %u instead of size %zu", num_joints_,
                 ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  const KDL::Frame frame = poseMsgToKDL(ik_pose);

  KDL::JntArray in(num_joints_), out(num_joints_);

  for (unsigned int z = 0; z < num_joints_; z++)
  {
    in(z) = ik_seed_state[z];
  }

  KDL::Twist bounds = KDL::Twist::Zero();

  if (position_ik_)
  {
    bounds.rot.x(std::numeric_limits<float>::max());
    bounds.rot.y(std::numeric_limits<float>::max());
    bounds.rot.z(std::numeric_limits<float>::max());
  }

  TRAC_IK::SolveType solvetype;

  if (solve_type_ == "Manipulation1")
  {
    solvetype = TRAC_IK::Manip1;
  }
  else if (solve_type_ == "Manipulation2")
  {
    solvetype = TRAC_IK::Manip2;
  }
  else if (solve_type_ == "Distance")
  {
    solvetype = TRAC_IK::Distance;
  }
  else
  {
    if (solve_type_ != "Speed")
    {
      RCLCPP_WARN(getLogger(), "%s is not a valid solve_type; setting to default: Speed",
                  solve_type_.c_str());
    }
    solvetype = TRAC_IK::Speed;
  }

  TRAC_IK::TRAC_IK ik_solver(chain_, joint_min_, joint_max_, timeout, epsilon_, solvetype);

  const int rc = ik_solver.CartToJnt(in, frame, out, bounds);

  solution.resize(num_joints_);

  if (rc >= 0)
  {
    for (unsigned int z = 0; z < num_joints_; z++)
    {
      solution[z] = out(z);
    }

    // check for collisions if a callback is provided
    if (solution_callback)
    {
      solution_callback(ik_pose, solution, error_code);
      if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        RCLCPP_DEBUG(getLogger(), "Solution passes callback");
        return true;
      }
      RCLCPP_DEBUG(getLogger(), "Solution has error code %d", error_code.val);
      return false;
    }

    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;  // no collision check callback provided
  }

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

}  // namespace trac_ik_kinematics_plugin

// register TRAC_IKKinematicsPlugin as a KinematicsBase implementation
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(trac_ik_kinematics_plugin::TRAC_IKKinematicsPlugin, kinematics::KinematicsBase);
