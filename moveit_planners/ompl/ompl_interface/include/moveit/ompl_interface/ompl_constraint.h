#ifndef MOVEIT_OMPL_INTERFACE_OMPL_CONSTRAINT
#define MOVEIT_OMPL_INTERFACE_OMPL_CONSTRAINT

#include <moveit/macros/class_forward.h>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

// Constraint
namespace ompl_interface{

namespace ob = ompl::base;

MOVEIT_CLASS_FORWARD(MoveitConstraint);

class MoveitConstraint : public ob::Constraint
{
public:
  MoveitConstraint(const ob::StateSpacePtr & stateSpace);

  void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

};
}

#endif