#include <moveit/ompl_interface/ompl_constraint.h>

// Constraint
namespace ompl_interface{

  MoveitConstraint::MoveitConstraint(const ob::StateSpacePtr & stateSpace) : ob::Constraint(7, 1)
  {

  }

  void MoveitConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
  {
  	out[0] = 0.0;
  }

}