/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <boost/math/constants/constants.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/bind.hpp>
#include <limits>
#include <memory>

namespace kinematic_constraints
{
static double normalizeAngle(double angle)
{
  double v = fmod(angle, 2.0 * boost::math::constants::pi<double>());
  if (v < -boost::math::constants::pi<double>())
    v += 2.0 * boost::math::constants::pi<double>();
  else if (v > boost::math::constants::pi<double>())
    v -= 2.0 * boost::math::constants::pi<double>();
  return v;
}

KinematicConstraint::KinematicConstraint(const robot_model::RobotModelConstPtr& model)
  : type_(UNKNOWN_CONSTRAINT), robot_model_(model), constraint_weight_(std::numeric_limits<double>::epsilon())
{
}

KinematicConstraint::~KinematicConstraint() = default;

bool JointConstraint::configure(const moveit_msgs::JointConstraint& jc)
{
  // clearing before we configure to get rid of any old data
  clear();

  // testing tolerances first
  if (jc.tolerance_above < 0.0 || jc.tolerance_below < 0.0)
  {
    ROS_WARN_NAMED("kinematic_constraints", "JointConstraint tolerance values must be positive.");
    joint_model_ = nullptr;
    return false;
  }

  joint_variable_name_ = jc.joint_name;
  local_variable_name_.clear();
  if (robot_model_->hasJointModel(joint_variable_name_))
    joint_model_ = robot_model_->getJointModel(joint_variable_name_);
  else
  {
    std::size_t pos = jc.joint_name.find_last_of('/');
    if (pos != std::string::npos)
    {
      joint_model_ = robot_model_->getJointModel(jc.joint_name.substr(0, pos));
      if (pos + 1 < jc.joint_name.length())
        local_variable_name_ = jc.joint_name.substr(pos + 1);
    }
    else
      joint_model_ = robot_model_->getJointModel(jc.joint_name);
  }

  if (joint_model_)
  {
    if (local_variable_name_.empty())
    {
      // check if the joint has 1 DOF (the only kind we can handle)
      if (joint_model_->getVariableCount() == 0)
      {
        ROS_ERROR_NAMED("kinematic_constraints", "Joint '%s' has no parameters to constrain", jc.joint_name.c_str());
        joint_model_ = nullptr;
      }
      else if (joint_model_->getVariableCount() > 1)
      {
        ROS_ERROR_NAMED("kinematic_constraints", "Joint '%s' has more than one parameter to constrain. "
                                                 "This type of constraint is not supported.",
                        jc.joint_name.c_str());
        joint_model_ = nullptr;
      }
    }
    else
    {
      int found = -1;
      const std::vector<std::string>& local_var_names = joint_model_->getLocalVariableNames();
      for (std::size_t i = 0; i < local_var_names.size(); ++i)
        if (local_var_names[i] == local_variable_name_)
        {
          found = i;
          break;
        }
      if (found < 0)
      {
        ROS_ERROR_NAMED("kinematic_constraints", "Local variable name '%s' is not known to joint '%s'",
                        local_variable_name_.c_str(), joint_model_->getName().c_str());
        joint_model_ = nullptr;
      }
    }
  }

  if (joint_model_)
  {
    joint_is_continuous_ = false;
    joint_tolerance_above_ = jc.tolerance_above;
    joint_tolerance_below_ = jc.tolerance_below;
    joint_variable_index_ = robot_model_->getVariableIndex(joint_variable_name_);

    // check if we have to wrap angles when computing distances
    joint_is_continuous_ = false;
    if (joint_model_->getType() == robot_model::JointModel::REVOLUTE)
    {
      const robot_model::RevoluteJointModel* rjoint = static_cast<const robot_model::RevoluteJointModel*>(joint_model_);
      if (rjoint->isContinuous())
        joint_is_continuous_ = true;
    }
    else if (joint_model_->getType() == robot_model::JointModel::PLANAR)
    {
      if (local_variable_name_ == "theta")
        joint_is_continuous_ = true;
    }

    if (joint_is_continuous_)
    {
      joint_position_ = normalizeAngle(jc.position);
    }
    else
    {
      joint_position_ = jc.position;
      const robot_model::VariableBounds& bounds = joint_model_->getVariableBounds(joint_variable_name_);

      if (bounds.min_position_ > joint_position_ + joint_tolerance_above_)
      {
        joint_position_ = bounds.min_position_;
        joint_tolerance_above_ = std::numeric_limits<double>::epsilon();
        ROS_WARN_NAMED("kinematic_constraints", "Joint %s is constrained to be below the minimum bounds. "
                                                "Assuming minimum bounds instead.",
                       jc.joint_name.c_str());
      }
      else if (bounds.max_position_ < joint_position_ - joint_tolerance_below_)
      {
        joint_position_ = bounds.max_position_;
        joint_tolerance_below_ = std::numeric_limits<double>::epsilon();
        ROS_WARN_NAMED("kinematic_constraints", "Joint %s is constrained to be above the maximum bounds. "
                                                "Assuming maximum bounds instead.",
                       jc.joint_name.c_str());
      }
    }

    if (jc.weight <= std::numeric_limits<double>::epsilon())
    {
      ROS_WARN_NAMED("kinematic_constraints",
                     "The weight on constraint for joint '%s' is very near zero.  Setting to 1.0.",
                     jc.joint_name.c_str());
      constraint_weight_ = 1.0;
    }
    else
      constraint_weight_ = jc.weight;
  }
  return joint_model_ != nullptr;
}

bool JointConstraint::equal(const KinematicConstraint& other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const JointConstraint& o = static_cast<const JointConstraint&>(other);
  if (o.joint_model_ == joint_model_ && o.local_variable_name_ == local_variable_name_)
    return fabs(joint_position_ - o.joint_position_) <= margin &&
           fabs(joint_tolerance_above_ - o.joint_tolerance_above_) <= margin &&
           fabs(joint_tolerance_below_ - o.joint_tolerance_below_) <= margin;
  return false;
}

ConstraintEvaluationResult JointConstraint::decide(const robot_state::RobotState& state, bool verbose) const
{
  if (!joint_model_)
    return ConstraintEvaluationResult(true, 0.0);

  double current_joint_position = state.getVariablePosition(joint_variable_index_);
  double dif = 0.0;

  // compute signed shortest distance for continuous joints
  if (joint_is_continuous_)
  {
    dif = normalizeAngle(current_joint_position) - joint_position_;

    if (dif > boost::math::constants::pi<double>())
      dif = 2.0 * boost::math::constants::pi<double>() - dif;
    else if (dif < -boost::math::constants::pi<double>())
      dif += 2.0 * boost::math::constants::pi<double>();  // we include a sign change to have dif > 0
  }
  else
    dif = current_joint_position - joint_position_;

  // check bounds
  bool result = dif <= (joint_tolerance_above_ + 2.0 * std::numeric_limits<double>::epsilon()) &&
                dif >= (-joint_tolerance_below_ - 2.0 * std::numeric_limits<double>::epsilon());
  if (verbose)
    ROS_INFO_NAMED("kinematic_constraints", "Constraint %s:: Joint name: '%s', actual value: %f, desired value: %f, "
                                            "tolerance_above: %f, tolerance_below: %f",
                   result ? "satisfied" : "violated", joint_variable_name_.c_str(), current_joint_position,
                   joint_position_, joint_tolerance_above_, joint_tolerance_below_);
  return ConstraintEvaluationResult(result, constraint_weight_ * fabs(dif));
}

bool JointConstraint::enabled() const
{
  return joint_model_;
}

void JointConstraint::clear()
{
  joint_model_ = nullptr;
  joint_variable_index_ = -1;
  joint_is_continuous_ = false;
  local_variable_name_ = "";
  joint_variable_name_ = "";
  joint_position_ = joint_tolerance_below_ = joint_tolerance_above_ = 0.0;
}

void JointConstraint::print(std::ostream& out) const
{
  if (joint_model_)
  {
    out << "Joint constraint for joint " << joint_variable_name_ << ": " << std::endl;
    out << "  value = ";
    out << joint_position_ << "; ";
    out << "  tolerance below = ";
    out << joint_tolerance_below_ << "; ";
    out << "  tolerance above = ";
    out << joint_tolerance_above_ << "; ";
    out << std::endl;
  }
  else
    out << "No constraint" << std::endl;
}

bool PositionConstraint::configure(const moveit_msgs::PositionConstraint& pc, const robot_state::Transforms& tf)
{
  // clearing before we configure to get rid of any old data
  clear();

  link_model_ = robot_model_->getLinkModel(pc.link_name);
  if (link_model_ == nullptr)
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "Position constraint link model %s not found in kinematic model. Constraint invalid.",
                   pc.link_name.c_str());
    return false;
  }

  if (pc.header.frame_id.empty())
  {
    ROS_WARN_NAMED("kinematic_constraints", "No frame specified for position constraint on link '%s'!",
                   pc.link_name.c_str());
    return false;
  }

  offset_ = Eigen::Vector3d(pc.target_point_offset.x, pc.target_point_offset.y, pc.target_point_offset.z);
  has_offset_ = offset_.squaredNorm() > std::numeric_limits<double>::epsilon();

  if (tf.isFixedFrame(pc.header.frame_id))
  {
    constraint_frame_id_ = tf.getTargetFrame();
    mobile_frame_ = false;
  }
  else
  {
    constraint_frame_id_ = pc.header.frame_id;
    mobile_frame_ = true;
  }

  // load primitive shapes, first clearing any we already have
  for (std::size_t i = 0; i < pc.constraint_region.primitives.size(); ++i)
  {
    std::unique_ptr<shapes::Shape> shape(shapes::constructShapeFromMsg(pc.constraint_region.primitives[i]));
    if (shape)
    {
      if (pc.constraint_region.primitive_poses.size() <= i)
      {
        ROS_WARN_NAMED("kinematic_constraints", "Constraint region message does not contain enough primitive poses");
        continue;
      }
      constraint_region_.push_back(bodies::BodyPtr(bodies::createBodyFromShape(shape.get())));
      Eigen::Affine3d t;
      tf2::fromMsg(pc.constraint_region.primitive_poses[i], t);
      constraint_region_pose_.push_back(t);
      if (mobile_frame_)
        constraint_region_.back()->setPose(constraint_region_pose_.back());
      else
      {
        tf.transformPose(pc.header.frame_id, constraint_region_pose_.back(), constraint_region_pose_.back());
        constraint_region_.back()->setPose(constraint_region_pose_.back());
      }
    }
    else
      ROS_WARN_NAMED("kinematic_constraints", "Could not construct primitive shape %zu", i);
  }

  // load meshes
  for (std::size_t i = 0; i < pc.constraint_region.meshes.size(); ++i)
  {
    std::unique_ptr<shapes::Shape> shape(shapes::constructShapeFromMsg(pc.constraint_region.meshes[i]));
    if (shape)
    {
      if (pc.constraint_region.mesh_poses.size() <= i)
      {
        ROS_WARN_NAMED("kinematic_constraints", "Constraint region message does not contain enough primitive poses");
        continue;
      }
      constraint_region_.push_back(bodies::BodyPtr(bodies::createBodyFromShape(shape.get())));
      Eigen::Affine3d t;
      tf2::fromMsg(pc.constraint_region.mesh_poses[i], t);
      constraint_region_pose_.push_back(t);
      if (mobile_frame_)
        constraint_region_.back()->setPose(constraint_region_pose_.back());
      else
      {
        tf.transformPose(pc.header.frame_id, constraint_region_pose_.back(), constraint_region_pose_.back());
        constraint_region_.back()->setPose(constraint_region_pose_.back());
      }
    }
    else
    {
      ROS_WARN_NAMED("kinematic_constraints", "Could not construct mesh shape %zu", i);
    }
  }

  if (pc.weight <= std::numeric_limits<double>::epsilon())
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "The weight on position constraint for link '%s' is near zero.  Setting to 1.0.",
                   pc.link_name.c_str());
    constraint_weight_ = 1.0;
  }
  else
    constraint_weight_ = pc.weight;

  return !constraint_region_.empty();
}

bool PositionConstraint::equal(const KinematicConstraint& other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const PositionConstraint& o = static_cast<const PositionConstraint&>(other);

  if (link_model_ == o.link_model_ && robot_state::Transforms::sameFrame(constraint_frame_id_, o.constraint_frame_id_))
  {
    if ((offset_ - o.offset_).norm() > margin)
      return false;
    std::vector<bool> other_region_matches_this(constraint_region_.size(), false);
    for (std::size_t i = 0; i < constraint_region_.size(); ++i)
    {
      bool some_match = false;
      // need to check against all other regions
      for (std::size_t j = 0; j < o.constraint_region_.size(); ++j)
      {
        Eigen::Affine3d diff = constraint_region_pose_[i].inverse() * o.constraint_region_pose_[j];
        if (diff.translation().norm() < margin && diff.rotation().isIdentity(margin) &&
            constraint_region_[i]->getType() == o.constraint_region_[j]->getType() &&
            fabs(constraint_region_[i]->computeVolume() - o.constraint_region_[j]->computeVolume()) < margin)
        {
          some_match = true;
          // can't break, as need to do matches the other way as well
          other_region_matches_this[j] = true;
        }
      }
      if (!some_match)
        return false;
    }
    for (std::size_t i = 0; i < o.constraint_region_.size(); ++i)
      if (!other_region_matches_this[i])
        return false;
    return true;
  }
  return false;
}

// helper function to avoid code duplication
static inline ConstraintEvaluationResult finishPositionConstraintDecision(const Eigen::Vector3d& pt,
                                                                          const Eigen::Vector3d& desired,
                                                                          const std::string& name, double weight,
                                                                          bool result, bool verbose)
{
  double dx = desired.x() - pt.x();
  double dy = desired.y() - pt.y();
  double dz = desired.z() - pt.z();
  if (verbose)
  {
    ROS_INFO_NAMED(
        "kinematic_constraints", "Position constraint %s on link '%s'. Desired: %f, %f, %f, current: %f, %f, %f",
        result ? "satisfied" : "violated", name.c_str(), desired.x(), desired.y(), desired.z(), pt.x(), pt.y(), pt.z());
    ROS_INFO_NAMED("kinematic_constraints", "Differences %g %g %g", dx, dy, dz);
  }
  return ConstraintEvaluationResult(result, weight * sqrt(dx * dx + dy * dy + dz * dz));
}

ConstraintEvaluationResult PositionConstraint::decide(const robot_state::RobotState& state, bool verbose) const
{
  if (!link_model_ || constraint_region_.empty())
    return ConstraintEvaluationResult(true, 0.0);

  Eigen::Vector3d pt = state.getGlobalLinkTransform(link_model_) * offset_;
  if (mobile_frame_)
  {
    for (std::size_t i = 0; i < constraint_region_.size(); ++i)
    {
      Eigen::Affine3d tmp = state.getFrameTransform(constraint_frame_id_) * constraint_region_pose_[i];
      bool result = constraint_region_[i]->cloneAt(tmp)->containsPoint(pt, verbose);
      if (result || (i + 1 == constraint_region_pose_.size()))
        return finishPositionConstraintDecision(pt, tmp.translation(), link_model_->getName(), constraint_weight_,
                                                result, verbose);
      else
        finishPositionConstraintDecision(pt, tmp.translation(), link_model_->getName(), constraint_weight_, result,
                                         verbose);
    }
  }
  else
  {
    for (std::size_t i = 0; i < constraint_region_.size(); ++i)
    {
      bool result = constraint_region_[i]->containsPoint(pt, true);
      if (result || (i + 1 == constraint_region_.size()))
        return finishPositionConstraintDecision(pt, constraint_region_[i]->getPose().translation(),
                                                link_model_->getName(), constraint_weight_, result, verbose);
      else
        finishPositionConstraintDecision(pt, constraint_region_[i]->getPose().translation(), link_model_->getName(),
                                         constraint_weight_, result, verbose);
    }
  }
  return ConstraintEvaluationResult(false, 0.0);
}

void PositionConstraint::print(std::ostream& out) const
{
  if (enabled())
    out << "Position constraint on link '" << link_model_->getName() << "'" << std::endl;
  else
    out << "No constraint" << std::endl;
}

void PositionConstraint::clear()
{
  offset_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  has_offset_ = false;
  constraint_region_.clear();
  constraint_region_pose_.clear();
  mobile_frame_ = false;
  constraint_frame_id_ = "";
  link_model_ = nullptr;
}

bool PositionConstraint::enabled() const
{
  return link_model_ && !constraint_region_.empty();
}

bool OrientationConstraint::configure(const moveit_msgs::OrientationConstraint& oc, const robot_state::Transforms& tf)
{
  // clearing out any old data
  clear();

  link_model_ = robot_model_->getLinkModel(oc.link_name);
  if (!link_model_)
  {
    ROS_WARN_NAMED("kinematic_constraints", "Could not find link model for link name %s", oc.link_name.c_str());
    return false;
  }
  Eigen::Quaterniond q;
  tf2::fromMsg(oc.orientation, q);
  if (fabs(q.norm() - 1.0) > 1e-3)
  {
    ROS_WARN_NAMED("kinematic_constraints", "Orientation constraint for link '%s' is probably incorrect: %f, %f, %f, "
                                            "%f. Assuming identity instead.",
                   oc.link_name.c_str(), oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w);
    q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  }

  if (oc.header.frame_id.empty())
    ROS_WARN_NAMED("kinematic_constraints", "No frame specified for position constraint on link '%s'!",
                   oc.link_name.c_str());

  if (tf.isFixedFrame(oc.header.frame_id))
  {
    tf.transformQuaternion(oc.header.frame_id, q, q);
    desired_rotation_frame_id_ = tf.getTargetFrame();
    desired_rotation_matrix_ = Eigen::Matrix3d(q);
    desired_rotation_matrix_inv_ = desired_rotation_matrix_.inverse();
    mobile_frame_ = false;
  }
  else
  {
    desired_rotation_frame_id_ = oc.header.frame_id;
    desired_rotation_matrix_ = Eigen::Matrix3d(q);
    mobile_frame_ = true;
  }
  std::stringstream matrix_str;
  matrix_str << desired_rotation_matrix_;
  ROS_DEBUG_NAMED("kinematic_constraints", "The desired rotation matrix for link '%s' in frame %s is:\n%s",
                  oc.link_name.c_str(), desired_rotation_frame_id_.c_str(), matrix_str.str().c_str());

  if (oc.weight <= std::numeric_limits<double>::epsilon())
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "The weight on position constraint for link '%s' is near zero.  Setting to 1.0.",
                   oc.link_name.c_str());
    constraint_weight_ = 1.0;
  }
  else
    constraint_weight_ = oc.weight;
  absolute_x_axis_tolerance_ = fabs(oc.absolute_x_axis_tolerance);
  if (absolute_x_axis_tolerance_ < std::numeric_limits<double>::epsilon())
    ROS_WARN_NAMED("kinematic_constraints", "Near-zero value for absolute_x_axis_tolerance");
  absolute_y_axis_tolerance_ = fabs(oc.absolute_y_axis_tolerance);
  if (absolute_y_axis_tolerance_ < std::numeric_limits<double>::epsilon())
    ROS_WARN_NAMED("kinematic_constraints", "Near-zero value for absolute_y_axis_tolerance");
  absolute_z_axis_tolerance_ = fabs(oc.absolute_z_axis_tolerance);
  if (absolute_z_axis_tolerance_ < std::numeric_limits<double>::epsilon())
    ROS_WARN_NAMED("kinematic_constraints", "Near-zero value for absolute_z_axis_tolerance");

  return link_model_ != nullptr;
}

bool OrientationConstraint::equal(const KinematicConstraint& other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const OrientationConstraint& o = static_cast<const OrientationConstraint&>(other);

  if (o.link_model_ == link_model_ &&
      robot_state::Transforms::sameFrame(desired_rotation_frame_id_, o.desired_rotation_frame_id_))
  {
    Eigen::Matrix3d diff = desired_rotation_matrix_.inverse() * o.desired_rotation_matrix_;
    if (!diff.isIdentity(margin))
      return false;
    return fabs(absolute_x_axis_tolerance_ - o.absolute_x_axis_tolerance_) <= margin &&
           fabs(absolute_y_axis_tolerance_ - o.absolute_y_axis_tolerance_) <= margin &&
           fabs(absolute_z_axis_tolerance_ - o.absolute_z_axis_tolerance_) <= margin;
  }
  return false;
}

void OrientationConstraint::clear()
{
  link_model_ = nullptr;
  desired_rotation_matrix_ = Eigen::Matrix3d::Identity();
  desired_rotation_matrix_inv_ = Eigen::Matrix3d::Identity();
  desired_rotation_frame_id_ = "";
  mobile_frame_ = false;
  absolute_z_axis_tolerance_ = absolute_y_axis_tolerance_ = absolute_x_axis_tolerance_ = 0.0;
}

bool OrientationConstraint::enabled() const
{
  return link_model_;
}

ConstraintEvaluationResult OrientationConstraint::decide(const robot_state::RobotState& state, bool verbose) const
{
  if (!link_model_)
    return ConstraintEvaluationResult(true, 0.0);

  Eigen::Vector3d xyz;
  if (mobile_frame_)
  {
    Eigen::Matrix3d tmp = state.getFrameTransform(desired_rotation_frame_id_).rotation() * desired_rotation_matrix_;
    Eigen::Affine3d diff(tmp.inverse() * state.getGlobalLinkTransform(link_model_).rotation());
    xyz = diff.rotation().eulerAngles(0, 1, 2);
    // 0,1,2 corresponds to XYZ, the convention used in sampling constraints
  }
  else
  {
    Eigen::Affine3d diff(desired_rotation_matrix_inv_ * state.getGlobalLinkTransform(link_model_).rotation());
    xyz =
        diff.rotation().eulerAngles(0, 1, 2);  // 0,1,2 corresponds to XYZ, the convention used in sampling constraints
  }

  xyz(0) = std::min(fabs(xyz(0)), boost::math::constants::pi<double>() - fabs(xyz(0)));
  xyz(1) = std::min(fabs(xyz(1)), boost::math::constants::pi<double>() - fabs(xyz(1)));
  xyz(2) = std::min(fabs(xyz(2)), boost::math::constants::pi<double>() - fabs(xyz(2)));
  bool result = xyz(2) < absolute_z_axis_tolerance_ + std::numeric_limits<double>::epsilon() &&
                xyz(1) < absolute_y_axis_tolerance_ + std::numeric_limits<double>::epsilon() &&
                xyz(0) < absolute_x_axis_tolerance_ + std::numeric_limits<double>::epsilon();

  if (verbose)
  {
    Eigen::Quaterniond q_act(state.getGlobalLinkTransform(link_model_).rotation());
    Eigen::Quaterniond q_des(desired_rotation_matrix_);
    ROS_INFO_NAMED("kinematic_constraints",
                   "Orientation constraint %s for link '%s'. Quaternion desired: %f %f %f %f, quaternion "
                   "actual: %f %f %f %f, error: x=%f, y=%f, z=%f, tolerance: x=%f, y=%f, z=%f",
                   result ? "satisfied" : "violated", link_model_->getName().c_str(), q_des.x(), q_des.y(), q_des.z(),
                   q_des.w(), q_act.x(), q_act.y(), q_act.z(), q_act.w(), xyz(0), xyz(1), xyz(2),
                   absolute_x_axis_tolerance_, absolute_y_axis_tolerance_, absolute_z_axis_tolerance_);
  }

  return ConstraintEvaluationResult(result, constraint_weight_ * (xyz(0) + xyz(1) + xyz(2)));
}

void OrientationConstraint::print(std::ostream& out) const
{
  if (link_model_)
  {
    out << "Orientation constraint on link '" << link_model_->getName() << "'" << std::endl;
    Eigen::Quaterniond q_des(desired_rotation_matrix_);
    out << "Desired orientation:" << q_des.x() << "," << q_des.y() << "," << q_des.z() << "," << q_des.w() << std::endl;
  }
  else
    out << "No constraint" << std::endl;
}

VisibilityConstraint::VisibilityConstraint(const robot_model::RobotModelConstPtr& model)
  : KinematicConstraint(model), collision_robot_(new collision_detection::CollisionRobotFCL(model))
{
  type_ = VISIBILITY_CONSTRAINT;
}

void VisibilityConstraint::clear()
{
  mobile_sensor_frame_ = false;
  mobile_target_frame_ = false;
  target_frame_id_ = "";
  sensor_frame_id_ = "";
  sensor_pose_ = Eigen::Affine3d::Identity();
  sensor_view_direction_ = 0;
  target_pose_ = Eigen::Affine3d::Identity();
  cone_sides_ = 0;
  points_.clear();
  target_radius_ = -1.0;
  max_view_angle_ = 0.0;
  max_range_angle_ = 0.0;
}

bool VisibilityConstraint::configure(const moveit_msgs::VisibilityConstraint& vc, const robot_state::Transforms& tf)
{
  clear();
  target_radius_ = fabs(vc.target_radius);

  if (vc.target_radius <= std::numeric_limits<double>::epsilon())
    ROS_WARN_NAMED("kinematic_constraints", "The radius of the target disc that must be visible should be strictly "
                                            "positive");

  if (vc.cone_sides < 3)
  {
    ROS_WARN_NAMED("kinematic_constraints", "The number of sides for the visibility region must be 3 or more. "
                                            "Assuming 3 sides instead of the specified %d",
                   vc.cone_sides);
    cone_sides_ = 3;
  }
  else
    cone_sides_ = vc.cone_sides;

  // compute the points on the base circle of the cone that make up the cone sides
  points_.clear();
  double delta = 2.0 * boost::math::constants::pi<double>() / (double)cone_sides_;
  double a = 0.0;
  for (unsigned int i = 0; i < cone_sides_; ++i, a += delta)
  {
    double x = sin(a) * target_radius_;
    double y = cos(a) * target_radius_;
    points_.push_back(Eigen::Vector3d(x, y, 0.0));
  }

  tf2::fromMsg(vc.target_pose.pose, target_pose_);

  if (tf.isFixedFrame(vc.target_pose.header.frame_id))
  {
    tf.transformPose(vc.target_pose.header.frame_id, target_pose_, target_pose_);
    target_frame_id_ = tf.getTargetFrame();
    mobile_target_frame_ = false;
    // transform won't change, so apply it now
    for (std::size_t i = 0; i < points_.size(); ++i)
      points_[i] = target_pose_ * points_[i];
  }
  else
  {
    target_frame_id_ = vc.target_pose.header.frame_id;
    mobile_target_frame_ = true;
  }

  tf2::fromMsg(vc.sensor_pose.pose, sensor_pose_);

  if (tf.isFixedFrame(vc.sensor_pose.header.frame_id))
  {
    tf.transformPose(vc.sensor_pose.header.frame_id, sensor_pose_, sensor_pose_);
    sensor_frame_id_ = tf.getTargetFrame();
    mobile_sensor_frame_ = false;
  }
  else
  {
    sensor_frame_id_ = vc.sensor_pose.header.frame_id;
    mobile_sensor_frame_ = true;
  }

  if (vc.weight <= std::numeric_limits<double>::epsilon())
  {
    ROS_WARN_NAMED("kinematic_constraints", "The weight of visibility constraint is near zero.  Setting to 1.0.");
    constraint_weight_ = 1.0;
  }
  else
    constraint_weight_ = vc.weight;

  max_view_angle_ = vc.max_view_angle;
  max_range_angle_ = vc.max_range_angle;
  sensor_view_direction_ = vc.sensor_view_direction;

  return target_radius_ > std::numeric_limits<double>::epsilon();
}

bool VisibilityConstraint::equal(const KinematicConstraint& other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const VisibilityConstraint& o = static_cast<const VisibilityConstraint&>(other);

  if (robot_state::Transforms::sameFrame(target_frame_id_, o.target_frame_id_) &&
      robot_state::Transforms::sameFrame(sensor_frame_id_, o.sensor_frame_id_) && cone_sides_ == o.cone_sides_ &&
      sensor_view_direction_ == o.sensor_view_direction_)
  {
    if (fabs(max_view_angle_ - o.max_view_angle_) > margin || fabs(target_radius_ - o.target_radius_) > margin)
      return false;
    Eigen::Affine3d diff = sensor_pose_.inverse() * o.sensor_pose_;
    if (diff.translation().norm() > margin)
      return false;
    if (!diff.rotation().isIdentity(margin))
      return false;
    diff = target_pose_.inverse() * o.target_pose_;
    if (diff.translation().norm() > margin)
      return false;
    if (!diff.rotation().isIdentity(margin))
      return false;
    return true;
  }
  return false;
}

bool VisibilityConstraint::enabled() const
{
  return target_radius_ > std::numeric_limits<double>::epsilon();
}

shapes::Mesh* VisibilityConstraint::getVisibilityCone(const robot_state::RobotState& state) const
{
  // the current pose of the sensor

  const Eigen::Affine3d& sp =
      mobile_sensor_frame_ ? state.getFrameTransform(sensor_frame_id_) * sensor_pose_ : sensor_pose_;
  const Eigen::Affine3d& tp =
      mobile_target_frame_ ? state.getFrameTransform(target_frame_id_) * target_pose_ : target_pose_;

  // transform the points on the disc to the desired target frame
  const EigenSTL::vector_Vector3d* points = &points_;
  std::unique_ptr<EigenSTL::vector_Vector3d> temp_points;
  if (mobile_target_frame_)
  {
    temp_points.reset(new EigenSTL::vector_Vector3d(points_.size()));
    for (std::size_t i = 0; i < points_.size(); ++i)
      temp_points->at(i) = tp * points_[i];
    points = temp_points.get();
  }

  // allocate memory for a mesh to represent the visibility cone
  shapes::Mesh* m = new shapes::Mesh();
  m->vertex_count = cone_sides_ + 2;
  m->vertices = new double[m->vertex_count * 3];
  m->triangle_count = cone_sides_ * 2;
  m->triangles = new unsigned int[m->triangle_count * 3];
  // we do NOT allocate normals because we do not compute them

  // the sensor origin
  m->vertices[0] = sp.translation().x();
  m->vertices[1] = sp.translation().y();
  m->vertices[2] = sp.translation().z();

  // the center of the base of the cone approximation
  m->vertices[3] = tp.translation().x();
  m->vertices[4] = tp.translation().y();
  m->vertices[5] = tp.translation().z();

  // the points that approximate the base disc
  for (std::size_t i = 0; i < points->size(); ++i)
  {
    m->vertices[i * 3 + 6] = points->at(i).x();
    m->vertices[i * 3 + 7] = points->at(i).y();
    m->vertices[i * 3 + 8] = points->at(i).z();
  }

  // add the triangles
  std::size_t p3 = points->size() * 3;
  for (std::size_t i = 1; i < points->size(); ++i)
  {
    // triangle forming a side of the cone, using the sensor origin
    std::size_t i3 = (i - 1) * 3;
    m->triangles[i3] = i + 1;
    m->triangles[i3 + 1] = 0;
    m->triangles[i3 + 2] = i + 2;
    // triangle forming a part of the base of the cone, using the center of the base
    std::size_t i6 = p3 + i3;
    m->triangles[i6] = i + 1;
    m->triangles[i6 + 1] = 1;
    m->triangles[i6 + 2] = i + 2;
  }

  // last triangles
  m->triangles[p3 - 3] = points->size() + 1;
  m->triangles[p3 - 2] = 0;
  m->triangles[p3 - 1] = 2;
  p3 *= 2;
  m->triangles[p3 - 3] = points->size() + 1;
  m->triangles[p3 - 2] = 1;
  m->triangles[p3 - 1] = 2;

  return m;
}

void VisibilityConstraint::getMarkers(const robot_state::RobotState& state,
                                      visualization_msgs::MarkerArray& markers) const
{
  shapes::Mesh* m = getVisibilityCone(state);
  visualization_msgs::Marker mk;
  shapes::constructMarkerFromShape(m, mk);
  delete m;
  mk.header.frame_id = robot_model_->getModelFrame();
  mk.header.stamp = ros::Time::now();
  mk.ns = "constraints";
  mk.id = 1;
  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.position.x = 0;
  mk.pose.position.y = 0;
  mk.pose.position.z = 0;
  mk.pose.orientation.x = 0;
  mk.pose.orientation.y = 0;
  mk.pose.orientation.z = 0;
  mk.pose.orientation.w = 1;
  mk.lifetime = ros::Duration(60);
  // this scale necessary to make results look reasonable
  mk.scale.x = .01;
  mk.color.a = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  markers.markers.push_back(mk);

  const Eigen::Affine3d& sp =
      mobile_sensor_frame_ ? state.getFrameTransform(sensor_frame_id_) * sensor_pose_ : sensor_pose_;
  const Eigen::Affine3d& tp =
      mobile_target_frame_ ? state.getFrameTransform(target_frame_id_) * target_pose_ : target_pose_;

  visualization_msgs::Marker mka;
  mka.type = visualization_msgs::Marker::ARROW;
  mka.action = visualization_msgs::Marker::ADD;
  mka.color = mk.color;
  mka.pose = mk.pose;

  mka.header = mk.header;
  mka.ns = mk.ns;
  mka.id = 2;
  mka.lifetime = mk.lifetime;
  mka.scale.x = 0.05;
  mka.scale.y = .15;
  mka.scale.z = 0.0;
  mka.points.resize(2);
  Eigen::Vector3d d = tp.translation() + tp.rotation().col(2) * 0.5;
  mka.points[0].x = tp.translation().x();
  mka.points[0].y = tp.translation().y();
  mka.points[0].z = tp.translation().z();
  mka.points[1].x = d.x();
  mka.points[1].y = d.y();
  mka.points[1].z = d.z();
  markers.markers.push_back(mka);

  mka.id = 3;
  mka.color.b = 1.0;
  mka.color.r = 0.0;

  d = sp.translation() + sp.rotation().col(2 - sensor_view_direction_) * 0.5;
  mka.points[0].x = sp.translation().x();
  mka.points[0].y = sp.translation().y();
  mka.points[0].z = sp.translation().z();
  mka.points[1].x = d.x();
  mka.points[1].y = d.y();
  mka.points[1].z = d.z();

  markers.markers.push_back(mka);
}

ConstraintEvaluationResult VisibilityConstraint::decide(const robot_state::RobotState& state, bool verbose) const
{
  if (target_radius_ <= std::numeric_limits<double>::epsilon())
    return ConstraintEvaluationResult(true, 0.0);

  if (max_view_angle_ > 0.0 || max_range_angle_ > 0.0)
  {
    const Eigen::Affine3d& sp =
        mobile_sensor_frame_ ? state.getFrameTransform(sensor_frame_id_) * sensor_pose_ : sensor_pose_;
    const Eigen::Affine3d& tp =
        mobile_target_frame_ ? state.getFrameTransform(target_frame_id_) * target_pose_ : target_pose_;

    // necessary to do subtraction as SENSOR_Z is 0 and SENSOR_X is 2
    const Eigen::Vector3d& normal2 = sp.rotation().col(2 - sensor_view_direction_);

    if (max_view_angle_ > 0.0)
    {
      const Eigen::Vector3d& normal1 = tp.rotation().col(2) * -1.0;  // along Z axis and inverted
      double dp = normal2.dot(normal1);
      double ang = acos(dp);
      if (dp < 0.0)
      {
        if (verbose)
          ROS_INFO_NAMED("kinematic_constraints", "Visibility constraint is violated because the sensor is looking at "
                                                  "the wrong side");
        return ConstraintEvaluationResult(false, 0.0);
      }
      if (max_view_angle_ < ang)
      {
        if (verbose)
          ROS_INFO_NAMED("kinematic_constraints", "Visibility constraint is violated because the view angle is %lf "
                                                  "(above the maximum allowed of %lf)",
                         ang, max_view_angle_);
        return ConstraintEvaluationResult(false, 0.0);
      }
    }
    if (max_range_angle_ > 0.0)
    {
      const Eigen::Vector3d& dir = (tp.translation() - sp.translation()).normalized();
      double dp = normal2.dot(dir);
      if (dp < 0.0)
      {
        if (verbose)
          ROS_INFO_NAMED("kinematic_constraints", "Visibility constraint is violated because the sensor is looking at "
                                                  "the wrong side");
        return ConstraintEvaluationResult(false, 0.0);
      }

      double ang = acos(dp);
      if (max_range_angle_ < ang)
      {
        if (verbose)
          ROS_INFO_NAMED("kinematic_constraints", "Visibility constraint is violated because the range angle is %lf "
                                                  "(above the maximum allowed of %lf)",
                         ang, max_range_angle_);
        return ConstraintEvaluationResult(false, 0.0);
      }
    }
  }

  shapes::Mesh* m = getVisibilityCone(state);
  if (!m)
    return ConstraintEvaluationResult(false, 0.0);

  // add the visibility cone as an object
  collision_detection::CollisionWorldFCL collision_world;
  collision_world.getWorld()->addToObject("cone", shapes::ShapeConstPtr(m), Eigen::Affine3d::Identity());

  // check for collisions between the robot and the cone
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  collision_detection::AllowedCollisionMatrix acm;
  acm.setDefaultEntry("cone", boost::bind(&VisibilityConstraint::decideContact, this, _1));
  req.contacts = true;
  req.verbose = verbose;
  req.max_contacts = 1;
  collision_world.checkRobotCollision(req, res, *collision_robot_, state, acm);

  if (verbose)
  {
    std::stringstream ss;
    m->print(ss);
    ROS_INFO_NAMED("kinematic_constraints", "Visibility constraint %ssatisfied. Visibility cone approximation:\n %s",
                   res.collision ? "not " : "", ss.str().c_str());
  }

  return ConstraintEvaluationResult(!res.collision, res.collision ? res.contacts.begin()->second.front().depth : 0.0);
}

bool VisibilityConstraint::decideContact(const collision_detection::Contact& contact) const
{
  if (contact.body_type_1 == collision_detection::BodyTypes::ROBOT_ATTACHED ||
      contact.body_type_2 == collision_detection::BodyTypes::ROBOT_ATTACHED)
    return true;
  if (contact.body_type_1 == collision_detection::BodyTypes::ROBOT_LINK &&
      contact.body_type_2 == collision_detection::BodyTypes::WORLD_OBJECT &&
      (robot_state::Transforms::sameFrame(contact.body_name_1, sensor_frame_id_) ||
       robot_state::Transforms::sameFrame(contact.body_name_1, target_frame_id_)))
  {
    ROS_DEBUG_NAMED("kinematic_constraints", "Accepted collision with either sensor or target");
    return true;
  }
  if (contact.body_type_2 == collision_detection::BodyTypes::ROBOT_LINK &&
      contact.body_type_1 == collision_detection::BodyTypes::WORLD_OBJECT &&
      (robot_state::Transforms::sameFrame(contact.body_name_2, sensor_frame_id_) ||
       robot_state::Transforms::sameFrame(contact.body_name_2, target_frame_id_)))
  {
    ROS_DEBUG_NAMED("kinematic_constraints", "Accepted collision with either sensor or target");
    return true;
  }
  return false;
}

void VisibilityConstraint::print(std::ostream& out) const
{
  if (enabled())
  {
    out << "Visibility constraint for sensor in frame '" << sensor_frame_id_ << "' using target in frame '"
        << target_frame_id_ << "'" << std::endl;
    out << "Target radius: " << target_radius_ << ", using " << cone_sides_ << " sides." << std::endl;
  }
  else
    out << "No constraint" << std::endl;
}
// MY CODE ON CIRCLE STARTS HERE

// configure
bool CircleConstraint::configure(const moveit_msgs::CircleConstraint& cc, const robot_state::Transforms& tf)
{
  // clearing before we configure to get rid of any old data
  clear();

  link_model_ = robot_model_->getLinkModel(cc.link_name);
  if (link_model_ == nullptr)
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "Position constraint link model %s not found in kinematic model. Constraint invalid.",
                   cc.link_name.c_str());
    return false;
  }

  if (cc.header.frame_id.empty())
  {
    ROS_WARN_NAMED("kinematic_constraints", "No frame specified for position constraint on link '%s'!",
                   cc.link_name.c_str());
    return false;
  }


  circle_start_ = Eigen::Vector3d(cc.circle_start.x, cc.circle_start.y, cc.circle_start.z);
  circle_end_ = Eigen::Vector3d(cc.circle_end.x, cc.circle_end.y, cc.circle_end.z);
  circle_center_ = Eigen::Vector3d(cc.circle_center.x, cc.circle_center.y, cc.circle_center.z);
  circle_radius_ = cc.circle_radius;

  const Eigen::Vector3d ac = circle_start_ - circle_center_;
  const Eigen::Vector3d bc = circle_end_ - circle_center_;

  // const Eigen::Vector3d ox = Eigen::Vector3d(0, circle_radius_, circle_center_.z()) - circle_center_;
  // double ox_norm = ox.norm();

  // double some_phi1 = acos(ac.dot( ox )/ (circle_radius_*ox_norm) );
  // double some_phi2 = acos(bc.dot( ox )/ (circle_radius_*ox_norm) );
  double some_phi1 = atan2(ac.y(), ac.x());
  double some_phi2 = atan2(bc.y(), bc.x());

  //if (some_phi1 < 0)
  //  some_phi1 = 2*M_PI + some_phi1;
  //if (some_phi2 < 0)
  //  some_phi2 = 2*M_PI + some_phi2;



  if (some_phi1 > some_phi2) {
    phi_start_ = some_phi2;
    phi_end_ = some_phi1;
  }
  else {
    phi_start_ = some_phi1;
    phi_end_ = some_phi2;
  }
  
  ROS_INFO_STREAM("phi_start_: " << phi_start_); 
  ROS_INFO_STREAM("phi_end_: " << phi_end_);

  if (tf.isFixedFrame(cc.header.frame_id))
  {
    constraint_frame_id_ = tf.getTargetFrame();
    mobile_frame_ = false;
  }
  else
  {
    constraint_frame_id_ = cc.header.frame_id;
    mobile_frame_ = true;
  }

  if (cc.weight <= std::numeric_limits<double>::epsilon())
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "The weight on position constraint for link '%s' is near zero.  Setting to 1.0.",
                   cc.link_name.c_str());
    constraint_weight_ = 1.0;
  }
  else
    constraint_weight_ = cc.weight;

  tolerance_ = fabs(cc.tolerance);
  if (tolerance_ < std::numeric_limits<double>::epsilon())
    ROS_WARN_NAMED("kinematic_constraints", "Near-zero value for arc tolerance");

    return link_model_ != nullptr;
}
// equal
bool CircleConstraint::equal(const KinematicConstraint& other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const CircleConstraint& o = static_cast<const CircleConstraint&>(other);

  if (link_model_ == o.link_model_ && robot_state::Transforms::sameFrame(constraint_frame_id_, o.constraint_frame_id_))
  {
    if ((circle_start_ - o.circle_start_).norm() > margin)
      return false;
    if ((circle_end_ - o.circle_end_).norm() > margin)
      return false;
    if ((circle_center_ - o.circle_center_).norm() > margin)
      return false;
    if (fabs(circle_radius_ - o.circle_radius_) > margin)
    	return false;
    return true;
  }
  return false;
}
// decide

// helper function to avoid code duplication
static inline ConstraintEvaluationResult finishCircleConstraintDecision(const Eigen::Vector3d& pt_p,
                                                                      const Eigen::Vector3d& pt_center,
                                                                      double radius,
                                                                      double ph1, double ph2,
                                                                      double tolerance)
{
  // given arc with start and end points (and corresponding angles),
  // find the distance between arbitrary point p and arc

  // if point p is out of range of arc
  //double d1 = (pt_p - pt_start).norm();
  //double d2 = (pt_p - pt_end).norm();
  //double d_temp = 0;
  //if(d1 > d2) d_temp = d2;
  //else d_temp = d1;

  // find angle
  // const Eigen::Vector3d ox = Eigen::Vector3d(0, 1, pt_center.z()) - pt_center;
  // double ox_norm = ox.norm();

  const Eigen::Vector3d pc = pt_p - pt_center;

  // double p_angle = acos(pc.dot( ox )/ (radius*ox_norm) );
  double p_angle = atan2(pc.y(), pc.x());
  //if (p_angle < 0)
  //  p_angle = 2*M_PI + p_angle;


  double d = 0;
  //ROS_INFO_STREAM("--- point angle: " << p_angle);
  if ( fabs(pt_p[2]-pt_center[2]) >= tolerance )
  	return ConstraintEvaluationResult(false, 0.0);

  if ( (ph1 - std::numeric_limits<double>::epsilon()<= p_angle) && (p_angle <= ph2 + std::numeric_limits<double>::epsilon()) )
    d  = fabs( pc.norm() - radius );
  else
    return ConstraintEvaluationResult(false, 0.0);
    //d = d_temp;

  //ROS_INFO_STREAM("---circle distance: " << d);  
  return ConstraintEvaluationResult(d <= tolerance, d);
}

ConstraintEvaluationResult CircleConstraint::decide(const robot_state::RobotState& state, bool verbose) const
{
  //ROS_INFO_STREAM("---H2");
  if (!link_model_ || !enabled())
    return ConstraintEvaluationResult(true, 0.0);

  // globallinktransform transforms a point on the link to the global frame
  Eigen::Vector3d pt = state.getGlobalLinkTransform(link_model_) * Eigen::Vector3d(0.0, 0.0, 0.0);

  if (mobile_frame_)
  {
    // TODOZJ: Not checked
    Eigen::Vector3d ptTransformed = state.getFrameTransform(constraint_frame_id_) * pt;
    return finishCircleConstraintDecision(ptTransformed, circle_center_, circle_radius_, phi_start_, phi_end_, tolerance_);
  }
  else
  {   
    return finishCircleConstraintDecision(pt, circle_center_, circle_radius_, phi_start_, phi_end_, tolerance_);
  }
  return ConstraintEvaluationResult(false, 0.0);
}

// print
void CircleConstraint::print(std::ostream& out) const
{
  if (enabled())
    out << "Trajectory constraint on link '" << link_model_->getName() << "'" << std::endl;
  else
    out << "No constraint" << std::endl;
}

// clear
void CircleConstraint::clear()
{
  circle_start_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  circle_end_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  circle_center_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  phi_start_ = 0.0;
  phi_end_ = 0.0;
  circle_radius_ = 0.0;
  tolerance_ = 0.0;
  mobile_frame_ = false;
  constraint_frame_id_ = "";
  link_model_ = nullptr;
}
// enabled
bool CircleConstraint::enabled() const
{
  return link_model_ != nullptr;
}


// END OF MY CODE

bool LineConstraint::configure(const moveit_msgs::LineConstraint& lc, const robot_state::Transforms& tf)
{
  // clearing before we configure to get rid of any old data
  clear();

  link_model_ = robot_model_->getLinkModel(lc.link_name);
  if (link_model_ == nullptr)
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "Position constraint link model %s not found in kinematic model. Constraint invalid.",
                   lc.link_name.c_str());
    return false;
  }

  if (lc.header.frame_id.empty())
  {
    ROS_WARN_NAMED("kinematic_constraints", "No frame specified for position constraint on link '%s'!",
                   lc.link_name.c_str());
    return false;
  }

  line_start_ = Eigen::Vector3d(lc.line_start.x, lc.line_start.y, lc.line_start.z);
  line_end_ = Eigen::Vector3d(lc.line_end.x, lc.line_end.y, lc.line_end.z);
  v_start_end_ = line_end_ - line_start_;

  if (tf.isFixedFrame(lc.header.frame_id))
  {
    constraint_frame_id_ = tf.getTargetFrame();
    mobile_frame_ = false;
  }
  else
  {
    constraint_frame_id_ = lc.header.frame_id;
    mobile_frame_ = true;
  }

  if (lc.weight <= std::numeric_limits<double>::epsilon())
  {
    ROS_WARN_NAMED("kinematic_constraints",
                   "The weight on position constraint for link '%s' is near zero.  Setting to 1.0.",
                   lc.link_name.c_str());
    constraint_weight_ = 1.0;
  }
  else
    constraint_weight_ = lc.weight;

  tolerance_ = fabs(lc.tolerance);
  if (tolerance_ < std::numeric_limits<double>::epsilon())
    ROS_WARN_NAMED("kinematic_constraints", "Near-zero value for line tolerance");

  return v_start_end_.norm() > std::numeric_limits<double>::epsilon();
}

bool LineConstraint::equal(const KinematicConstraint& other, double margin) const
{
  if (other.getType() != type_)
    return false;
  const LineConstraint& o = static_cast<const LineConstraint&>(other);

  if (link_model_ == o.link_model_ && robot_state::Transforms::sameFrame(constraint_frame_id_, o.constraint_frame_id_))
  {
    if ((line_start_ - o.line_start_).norm() > margin)
      return false;
    if ((line_end_ - o.line_end_).norm() > margin)
      return false;
    return true;
  }
  return false;
}

// helper function to avoid code duplication
static inline ConstraintEvaluationResult finishLineConstraintDecision(const Eigen::Vector3d& pt_p,
                                                                      const Eigen::Vector3d& pt_a,
                                                                      const Eigen::Vector3d& dir_u,
                                                                      double tolerance)
{
  // Line starts from A, and has direction vector u
  // We want to find distance between point p and line.
  Eigen::Vector3d ap = pt_p - pt_a;

  Eigen::Vector3d u_hat = dir_u.normalized();
  double u_mag = dir_u.norm();
  double extentOnLine = ap.dot(u_hat) / u_mag;
  //ROS_INFO_STREAM("---------extentOnLine: " << extentOnLine);   //TODOZJ
  if (extentOnLine >= 0 - std::numeric_limits<double>::epsilon()
      && extentOnLine <= 1 + std::numeric_limits<double>::epsilon()) {
    double d = ap.cross(dir_u).norm() / u_mag;
    // ROS_INFO_STREAM("finishLineConstraintDecision: d=" << d << " " << (d <= tolerance));  //TODOZJ
    return ConstraintEvaluationResult(d <= tolerance, d);
  } else {
    // ROS_INFO_STREAM("finishLineConstraintDecision: FAILED");  //TODOZJ
    return ConstraintEvaluationResult(false, 0.0);
  }
}

ConstraintEvaluationResult LineConstraint::decide(const robot_state::RobotState& state, bool verbose) const
{
  if (!link_model_ || !enabled())
    return ConstraintEvaluationResult(true, 0.0);

  // globallinktransform transforms a point on the link to the global frame
  Eigen::Vector3d pt = state.getGlobalLinkTransform(link_model_) * Eigen::Vector3d(0.0, 0.0, 0.0);

  if (mobile_frame_)
  {
    // TODOZJ: Not checked
    Eigen::Vector3d ptTransformed = state.getFrameTransform(constraint_frame_id_) * pt;
    return finishLineConstraintDecision(ptTransformed, line_start_, v_start_end_, tolerance_);
  }
  else
  {   
    return finishLineConstraintDecision(pt, line_start_, v_start_end_, tolerance_);
  }
  return ConstraintEvaluationResult(false, 0.0);
}

void LineConstraint::print(std::ostream& out) const
{
  if (enabled())
    out << "Trajectory constraint on link '" << link_model_->getName() << "'" << std::endl;
  else
    out << "No constraint" << std::endl;
}

void LineConstraint::clear()
{
  line_start_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  line_end_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  v_start_end_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  tolerance_ = 0.0;
  mobile_frame_ = false;
  constraint_frame_id_ = "";
  link_model_ = nullptr;
}

bool LineConstraint::enabled() const
{
  return link_model_ && v_start_end_.norm() > std::numeric_limits<double>::epsilon();
}

void KinematicConstraintSet::clear()
{
  all_constraints_ = moveit_msgs::Constraints();
  kinematic_constraints_.clear();
  joint_constraints_.clear();
  position_constraints_.clear();
  orientation_constraints_.clear();
  visibility_constraints_.clear();
  line_constraints_.clear();
}

bool KinematicConstraintSet::add(const std::vector<moveit_msgs::JointConstraint>& jc)
{
  bool result = true;
  for (unsigned int i = 0; i < jc.size(); ++i)
  {
    JointConstraint* ev = new JointConstraint(robot_model_);
    bool u = ev->configure(jc[i]);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    joint_constraints_.push_back(jc[i]);
    all_constraints_.joint_constraints.push_back(jc[i]);
  }
  return result;
}

bool KinematicConstraintSet::add(const std::vector<moveit_msgs::PositionConstraint>& pc,
                                 const robot_state::Transforms& tf)
{
  bool result = true;
  for (unsigned int i = 0; i < pc.size(); ++i)
  {
    PositionConstraint* ev = new PositionConstraint(robot_model_);
    bool u = ev->configure(pc[i], tf);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    position_constraints_.push_back(pc[i]);
    all_constraints_.position_constraints.push_back(pc[i]);
  }
  return result;
}

bool KinematicConstraintSet::add(const std::vector<moveit_msgs::OrientationConstraint>& oc,
                                 const robot_state::Transforms& tf)
{
  bool result = true;
  for (unsigned int i = 0; i < oc.size(); ++i)
  {
    OrientationConstraint* ev = new OrientationConstraint(robot_model_);
    bool u = ev->configure(oc[i], tf);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    orientation_constraints_.push_back(oc[i]);
    all_constraints_.orientation_constraints.push_back(oc[i]);
  }
  return result;
}

bool KinematicConstraintSet::add(const std::vector<moveit_msgs::VisibilityConstraint>& vc,
                                 const robot_state::Transforms& tf)
{
  bool result = true;
  for (unsigned int i = 0; i < vc.size(); ++i)
  {
    VisibilityConstraint* ev = new VisibilityConstraint(robot_model_);
    bool u = ev->configure(vc[i], tf);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    visibility_constraints_.push_back(vc[i]);
    all_constraints_.visibility_constraints.push_back(vc[i]);
  }
  return result;
}

bool KinematicConstraintSet::add(const std::vector<moveit_msgs::LineConstraint>& lc,
                                 const robot_state::Transforms& tf)
{
  bool result = true;
  for (unsigned int i = 0; i < lc.size(); ++i)
  {
    LineConstraint* ev = new LineConstraint(robot_model_);
    bool u = ev->configure(lc[i], tf);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    line_constraints_.push_back(lc[i]);
    all_constraints_.line_constraints.push_back(lc[i]);
  }
  return result;
}

// NEW
bool KinematicConstraintSet::add(const std::vector<moveit_msgs::CircleConstraint>& cc,
                                 const robot_state::Transforms& tf)
{
  bool result = true;
  for (unsigned int i = 0; i < cc.size(); ++i)
  {
    CircleConstraint* ev = new CircleConstraint(robot_model_);
    bool u = ev->configure(cc[i], tf);
    result = result && u;
    kinematic_constraints_.push_back(KinematicConstraintPtr(ev));
    circle_constraints_.push_back(cc[i]);
    all_constraints_.circle_constraints.push_back(cc[i]);
  }
  return result;
}

bool KinematicConstraintSet::add(const moveit_msgs::Constraints& c, const robot_state::Transforms& tf)
{
  bool j = add(c.joint_constraints);
  bool p = add(c.position_constraints, tf);
  bool o = add(c.orientation_constraints, tf);
  bool v = add(c.visibility_constraints, tf);
  bool l = add(c.line_constraints, tf);
  bool c_c = add(c.circle_constraints, tf);
  return j && p && o && v && l && c_c;
}

ConstraintEvaluationResult KinematicConstraintSet::decide(const robot_state::RobotState& state, bool verbose) const
{
  ConstraintEvaluationResult res(true, 0.0);
  for (unsigned int i = 0; i < kinematic_constraints_.size(); ++i)
  {
    ConstraintEvaluationResult r = kinematic_constraints_[i]->decide(state, verbose);
    if (!r.satisfied)
      res.satisfied = false;
    res.distance += r.distance;
  }
  return res;
}

ConstraintEvaluationResult KinematicConstraintSet::decide(const robot_state::RobotState& state,
                                                          std::vector<ConstraintEvaluationResult>& results,
                                                          bool verbose) const
{
  ConstraintEvaluationResult result(true, 0.0);
  results.resize(kinematic_constraints_.size());
  for (std::size_t i = 0; i < kinematic_constraints_.size(); ++i)
  {
    results[i] = kinematic_constraints_[i]->decide(state, verbose);
    result.satisfied = result.satisfied && results[i].satisfied;
    result.distance += results[i].distance;
  }

  return result;
}

void KinematicConstraintSet::print(std::ostream& out) const
{
  out << kinematic_constraints_.size() << " kinematic constraints" << std::endl;
  for (unsigned int i = 0; i < kinematic_constraints_.size(); ++i)
    kinematic_constraints_[i]->print(out);
}

bool KinematicConstraintSet::equal(const KinematicConstraintSet& other, double margin) const
{
  // each constraint in this matches some in the other
  for (unsigned int i = 0; i < kinematic_constraints_.size(); ++i)
  {
    bool found = false;
    for (unsigned int j = 0; !found && j < other.kinematic_constraints_.size(); ++j)
      found = kinematic_constraints_[i]->equal(*other.kinematic_constraints_[j], margin);
    if (!found)
      return false;
  }
  // each constraint in the other matches some constraint in this
  for (unsigned int i = 0; i < other.kinematic_constraints_.size(); ++i)
  {
    bool found = false;
    for (unsigned int j = 0; !found && j < kinematic_constraints_.size(); ++j)
      found = other.kinematic_constraints_[i]->equal(*kinematic_constraints_[j], margin);
    if (!found)
      return false;
  }
  return true;
}

}  // end of namespace kinematic_constraints
