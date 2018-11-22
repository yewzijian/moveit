/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Zi Jian Yew.
/* Edited from MoveIt code from Ioan Sucan */

#include <boost/algorithm/string/trim.hpp>

#include <moveit/ompl_interface/constrained_planning_context.h>
#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/detail/constrained_sampler.h>
#include <moveit/ompl_interface/detail/constrained_goal_sampler.h>
#include <moveit/ompl_interface/detail/goal_union.h>
#include <moveit/ompl_interface/detail/projection_evaluators.h>
#include <moveit/ompl_interface/constraints_library.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/profiler/profiler.h>
#include <moveit/utils/lexical_casts.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h>

bool isStateValid(const ompl::base::State *state)
{
    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return true;
}

ompl_interface::ConstrainedPlanningContext::ConstrainedPlanningContext(const std::string& name,
                                                                     const ModelBasedPlanningContextSpecification& spec)
  : ModelBasedPlanningContext(name, spec)
{
}

void ompl_interface::ConstrainedPlanningContext::configure()
{


  ROS_INFO_STREAM("Start ConstrainedPlanningContext::configure()");

  // convert the input state to the corresponding OMPL state
  ompl::base::ScopedState<> ompl_start_state(spec_.constrained_state_space_);

  // spec_.state_space_->copyToOMPLState(ompl_start_state.get(), getCompleteInitialRobotState());
  const robot_state::RobotState& rState = getCompleteInitialRobotState();
  std::vector<double> tempVec;
  rState.copyJointGroupPositions(getJointModelGroup(), tempVec);
  spec_.constrained_state_space_->copyFromReals(ompl_start_state.get(), tempVec);
  ROS_INFO_STREAM("Start state:");
  ROS_INFO_STREAM(ompl_start_state);
  
  ompl_simple_setup_->setStartState(ompl_start_state);
  
  ompl_simple_setup_->setGoalState(ompl_start_state);
  ROS_INFO_STREAM("setStateValidityChecker()");
  ompl_simple_setup_->setStateValidityChecker(isStateValid);  // TODOZJ

  if (path_constraints_ && spec_.constraints_library_)
  {
    ROS_INFO_STREAM("(path_constraints_ && spec_.constraints_library_)");
    const ConstraintApproximationPtr& ca =
        spec_.constraints_library_->getConstraintApproximation(path_constraints_msg_);
    if (ca)
    {
      getOMPLStateSpace()->setInterpolationFunction(ca->getInterpolationFunction());
      ROS_INFO_NAMED("constrained_planning_context", "Using precomputed interpolation states");
    }
  }

  // // Try setting goal
  ROS_INFO_STREAM("Try solving...");
  auto pp = std::make_shared<og::RRTConnect>(spec_.constrained_space_info_);
  ROS_INFO_STREAM("setPlanner");
  ompl_simple_setup_->setPlanner(pp);
  ROS_INFO_STREAM("setup");
  ompl_simple_setup_->setup();
  ROS_INFO_STREAM("solve");
  ompl_simple_setup_->solve(5.0);
  ROS_INFO_STREAM("Solved");


  ROS_INFO_STREAM("before useconfig");
  useConfig();
  if (ompl_simple_setup_->getGoal()) {
    ROS_INFO_STREAM("ompl_simple_setup_->getGoal() true");
    ompl_simple_setup_->setup();
  }

  ROS_INFO_STREAM("End ConstrainedPlanningContext::configure()");
}


ompl::base::GoalPtr ompl_interface::ConstrainedPlanningContext::constructGoal()
{
  // // TOCHECK!!!!!!!
  // // ******************* set up the goal representation, based on goal constraints
  ROS_INFO_STREAM("Construct GOAL!");
  std::vector<ob::GoalPtr> goals;
  for (kinematic_constraints::KinematicConstraintSetPtr& goal_constraint : goal_constraints_)
  {
    ROS_INFO_STREAM("- kinematic_constraint: " << goal_constraint);
    constraint_samplers::ConstraintSamplerPtr cs;
    if (spec_.constraint_sampler_manager_) {
      cs = spec_.constraint_sampler_manager_->selectSampler(getPlanningScene(), getGroupName(),
                                                            goal_constraint->getAllConstraints());
      ROS_INFO_STREAM("  - select sampler");
    }
    if (cs)
    {
      ob::GoalPtr g = ob::GoalPtr(new ConstrainedGoalSampler(this, goal_constraint, cs));
      goals.push_back(g);
    }
  }

  if (!goals.empty())
    return goals.size() == 1 ? goals[0] : ompl::base::GoalPtr(new GoalSampleableRegionMux(goals));
  else
    ROS_ERROR_NAMED("model_based_planning_context", "Unable to construct goal representation");

  ROS_INFO_STREAM("Huh?");
  return ob::GoalPtr();
}
