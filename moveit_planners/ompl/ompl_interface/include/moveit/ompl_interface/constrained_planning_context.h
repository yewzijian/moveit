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

#ifndef MOVEIT_OMPL_INTERFACE_CONSTRAINED_PLANNING_CONTEXT_
#define MOVEIT_OMPL_INTERFACE_CONSTRAINED_PLANNING_CONTEXT_

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/ompl_interface/detail/constrained_valid_state_sampler.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/ompl_interface/model_based_planning_context.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/StateStorage.h>

namespace ompl_interface
{
namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

MOVEIT_CLASS_FORWARD(ConstrainedPlanningContext);
MOVEIT_CLASS_FORWARD(ConstraintsLibrary);

class ConstrainedPlanningContext : public ModelBasedPlanningContext
{
public:
  ConstrainedPlanningContext(const std::string& name, const ModelBasedPlanningContextSpecification& spec);

  virtual void configure() override;

protected:
	virtual ob::GoalPtr constructGoal();
};
}  // namespace ompl_interface

#endif
