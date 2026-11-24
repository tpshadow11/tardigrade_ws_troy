// Behavior tree nodes for Tardigrade. See docs/behavior_tree_cpp.md for the
// list of nodes and their ports (the attributes you write in the XML).
#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "tardigrade_bt/vehicle.hpp"

namespace tardigrade_bt
{

// Shortest signed angle from `from` to `to`, in radians.
double angleDiff(double to, double from);
double clamp(double value, double limit);

// Registers every Tardigrade node with the factory. Call this once, before
// loading any tree XML.
void registerNodes(BT::BehaviorTreeFactory & factory, Vehicle * vehicle);

}  // namespace tardigrade_bt
