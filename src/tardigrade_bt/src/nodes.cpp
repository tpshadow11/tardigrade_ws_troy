// Behavior tree nodes for Tardigrade.
//
// Every node returns one of three answers each tick:
//   SUCCESS  - finished
//   FAILURE  - could not do it
//   RUNNING  - still working, tick me again
//
// Move nodes are StatefulActionNodes:
//   onStart()   runs once when the node begins
//   onRunning() runs on every later tick
//   onHalted()  runs if a parent stops the node early (timeout, safety check)

#include "tardigrade_bt/nodes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace tardigrade_bt
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double degToRad(double degrees) {return degrees * kPi / 180.0;}

// Read an input port, or fall back to a default when the XML leaves it out.
template<typename T>
T portOr(const BT::TreeNode & node, const std::string & name, T fallback)
{
  auto value = node.getInput<T>(name);
  return value ? value.value() : fallback;
}

}  // namespace

double angleDiff(double to, double from)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

double clamp(double value, double limit)
{
  limit = std::abs(limit);
  return std::max(-limit, std::min(limit, value));
}

// ---------------------------------------------------------------- conditions

// SUCCESS while odometry keeps arriving.
class OdometryFresh : public BT::ConditionNode
{
public:
  OdometryFresh(const std::string & name, const BT::NodeConfig & config, Vehicle * vehicle)
  : BT::ConditionNode(name, config), vehicle_(vehicle) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_age_sec", 0.5, "Oldest odometry allowed")};
  }

  BT::NodeStatus tick() override
  {
    const double max_age = portOr(*this, "max_age_sec", 0.5);
    return vehicle_->odometryAge() <= max_age ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  Vehicle * vehicle_;
};

// SUCCESS while the robot reports it is armed. Trees never arm the robot:
// a person arms it on purpose (MIS-01 safety rule).
class IsArmed : public BT::ConditionNode
{
public:
  IsArmed(const std::string & name, const BT::NodeConfig & config, Vehicle * vehicle)
  : BT::ConditionNode(name, config), vehicle_(vehicle) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    return vehicle_->armed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  Vehicle * vehicle_;
};

// ------------------------------------------------------------ simple actions

// Remember the current pose as the start. Every move is measured from it.
class RecordStart : public BT::SyncActionNode
{
public:
  RecordStart(const std::string & name, const BT::NodeConfig & config, Vehicle * vehicle)
  : BT::SyncActionNode(name, config), vehicle_(vehicle) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    if (vehicle_->odometryAge() == std::numeric_limits<double>::infinity()) {
      return BT::NodeStatus::FAILURE;
    }
    vehicle_->recordStart();
    return BT::NodeStatus::SUCCESS;
  }

private:
  Vehicle * vehicle_;
};

// RUNNING until odometry arrives, FAILURE after timeout_sec.
class WaitForOdometry : public BT::StatefulActionNode
{
public:
  WaitForOdometry(const std::string & name, const BT::NodeConfig & config, Vehicle * vehicle)
  : BT::StatefulActionNode(name, config), vehicle_(vehicle) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_sec", 30.0, "Give up after this long"),
      BT::InputPort<double>("max_age_sec", 0.5, "Oldest odometry allowed"),
    };
  }

  BT::NodeStatus onStart() override
  {
    end_time_ = vehicle_->now() + portOr(*this, "timeout_sec", 30.0);
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    // Also wait for the first robot status, so IsArmed right after this
    // node sees the real arming state instead of the "not armed" default.
    if (vehicle_->odometryAge() <= portOr(*this, "max_age_sec", 0.5) &&
      vehicle_->hasStatus())
    {
      return BT::NodeStatus::SUCCESS;
    }
    return vehicle_->now() >= end_time_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  Vehicle * vehicle_;
  double end_time_{0.0};
};

// -------------------------------------------------------------- move actions

// Shared pattern for every move: send a velocity each tick until done().
// Whenever the move ends for ANY reason it sends zero, so a timeout or a
// failed safety check always leaves the robot commanded to stop.
class MoveAction : public BT::StatefulActionNode
{
public:
  MoveAction(const std::string & name, const BT::NodeConfig & config, Vehicle * vehicle)
  : BT::StatefulActionNode(name, config), vehicle_(vehicle) {}

  static BT::PortsList headingPorts()
  {
    return {
      BT::InputPort<double>(
        "heading_deg", 0.0, "Heading to keep, degrees left of the start heading"),
    };
  }

  BT::NodeStatus onStart() override
  {
    if (!vehicle_->hasStart()) {
      // RecordStart must run before any move.
      return BT::NodeStatus::FAILURE;
    }
    target_heading_ = vehicle_->start().yaw + degToRad(portOr(*this, "heading_deg", 0.0));
    begin();
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    if (done()) {
      vehicle_->stop();
      return BT::NodeStatus::SUCCESS;
    }
    step();
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {vehicle_->stop();}

protected:
  virtual void begin() {}
  virtual bool done() = 0;
  virtual void step() = 0;

  // Small PD loop that turns toward target_heading_ (rad/s output).
  double holdHeading() const
  {
    const double error = angleDiff(target_heading_, vehicle_->pose().yaw);
    return clamp(0.4 * error - 0.1 * vehicle_->yawRate(), 0.2);
  }

  Vehicle * vehicle_;
  double target_heading_{0.0};
};

// Go to depth_m below the start depth (0 = back to the start depth).
class GoToDepth : public MoveAction
{
public:
  using MoveAction::MoveAction;

  static BT::PortsList providedPorts()
  {
    auto ports = headingPorts();
    ports.insert(BT::InputPort<double>("depth_m", "Metres below the start depth"));
    ports.insert(BT::InputPort<double>("tolerance_m", 0.05, "Close enough"));
    ports.insert(BT::InputPort<double>("max_speed", 0.2, "Fastest up/down speed, m/s"));
    return ports;
  }

protected:
  void begin() override
  {
    target_z_ = vehicle_->start().z - std::abs(portOr(*this, "depth_m", 0.0));
    tolerance_ = portOr(*this, "tolerance_m", 0.05);
    max_speed_ = portOr(*this, "max_speed", 0.2);
  }

  bool done() override {return std::abs(target_z_ - vehicle_->pose().z) <= tolerance_;}

  void step() override
  {
    // Slow down as the target gets close (ENU: up is +z).
    const double heave = clamp(0.8 * (target_z_ - vehicle_->pose().z), max_speed_);
    vehicle_->command(0.0, 0.0, heave, holdHeading());
  }

private:
  double target_z_{0.0};
  double tolerance_{0.05};
  double max_speed_{0.2};
};

// Drive distance_m in a straight line. direction: +1 forward, -1 backward
// (facing heading_deg either way), or sideways for Strafe.
class StraightMove : public MoveAction
{
public:
  StraightMove(
    const std::string & name, const BT::NodeConfig & config, Vehicle * vehicle,
    double forward, double left)
  : MoveAction(name, config, vehicle), forward_(forward), left_(left) {}

  static BT::PortsList providedPorts()
  {
    auto ports = headingPorts();
    ports.insert(BT::InputPort<double>("distance_m", "How far to go, metres"));
    ports.insert(BT::InputPort<double>("speed", 0.2, "m/s"));
    return ports;
  }

protected:
  void begin() override
  {
    const Pose here = vehicle_->pose();
    begin_x_ = here.x;
    begin_y_ = here.y;
    distance_ = std::abs(portOr(*this, "distance_m", 0.0));
    speed_ = std::abs(portOr(*this, "speed", 0.2));
  }

  bool done() override
  {
    // Distance travelled along the direction of motion (world frame).
    const Pose here = vehicle_->pose();
    const double dx = here.x - begin_x_;
    const double dy = here.y - begin_y_;
    const double c = std::cos(target_heading_);
    const double s = std::sin(target_heading_);
    const double along_forward = dx * c + dy * s;
    const double along_left = -dx * s + dy * c;
    return forward_ * along_forward + left_ * along_left >= distance_;
  }

  void step() override
  {
    vehicle_->command(forward_ * speed_, left_ * speed_, 0.0, holdHeading());
  }

private:
  double forward_;
  double left_;
  double begin_x_{0.0};
  double begin_y_{0.0};
  double distance_{0.0};
  double speed_{0.2};
};

// Turn in place until facing heading_deg (degrees left of the start heading).
class Turn : public MoveAction
{
public:
  using MoveAction::MoveAction;

  static BT::PortsList providedPorts()
  {
    auto ports = headingPorts();
    ports.insert(BT::InputPort<double>("tolerance_deg", 4.0, "Close enough"));
    return ports;
  }

protected:
  bool done() override
  {
    const double tolerance = degToRad(portOr(*this, "tolerance_deg", 4.0));
    return std::abs(angleDiff(target_heading_, vehicle_->pose().yaw)) <= tolerance;
  }

  void step() override {vehicle_->command(0.0, 0.0, 0.0, holdHeading());}
};

// Stay still for `seconds` while keeping heading_deg.
class Hold : public MoveAction
{
public:
  using MoveAction::MoveAction;

  static BT::PortsList providedPorts()
  {
    auto ports = headingPorts();
    ports.insert(BT::InputPort<double>("seconds", 2.0, "How long to hold"));
    return ports;
  }

protected:
  void begin() override {end_time_ = vehicle_->now() + portOr(*this, "seconds", 2.0);}
  bool done() override {return vehicle_->now() >= end_time_;}
  void step() override {vehicle_->command(0.0, 0.0, 0.0, holdHeading());}

private:
  double end_time_{0.0};
};

// Push straight up for `seconds`. Uses only the clock, never odometry,
// because lost odometry is one of the reasons to surface.
class Surface : public BT::StatefulActionNode
{
public:
  Surface(const std::string & name, const BT::NodeConfig & config, Vehicle * vehicle)
  : BT::StatefulActionNode(name, config), vehicle_(vehicle) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("seconds", 10.0, "How long to push up"),
      BT::InputPort<double>("speed", 0.2, "Upward speed, m/s"),
    };
  }

  BT::NodeStatus onStart() override
  {
    end_time_ = vehicle_->now() + portOr(*this, "seconds", 10.0);
    speed_ = std::abs(portOr(*this, "speed", 0.2));
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    if (vehicle_->now() >= end_time_) {
      vehicle_->stop();
      return BT::NodeStatus::SUCCESS;
    }
    vehicle_->command(0.0, 0.0, speed_, 0.0);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {vehicle_->stop();}

private:
  Vehicle * vehicle_;
  double end_time_{0.0};
  double speed_{0.2};
};

// ------------------------------------------------------------- registration

void registerNodes(BT::BehaviorTreeFactory & factory, Vehicle * vehicle)
{
  factory.registerNodeType<OdometryFresh>("OdometryFresh", vehicle);
  factory.registerNodeType<IsArmed>("IsArmed", vehicle);
  factory.registerNodeType<RecordStart>("RecordStart", vehicle);
  factory.registerNodeType<WaitForOdometry>("WaitForOdometry", vehicle);
  factory.registerNodeType<GoToDepth>("GoToDepth", vehicle);
  factory.registerNodeType<StraightMove>("Forward", vehicle, 1.0, 0.0);
  factory.registerNodeType<StraightMove>("Backward", vehicle, -1.0, 0.0);
  factory.registerNodeType<StraightMove>("StrafeLeft", vehicle, 0.0, 1.0);
  factory.registerNodeType<StraightMove>("StrafeRight", vehicle, 0.0, -1.0);
  factory.registerNodeType<Turn>("Turn", vehicle);
  factory.registerNodeType<Hold>("Hold", vehicle);
  factory.registerNodeType<Surface>("Surface", vehicle);
}

}  // namespace tardigrade_bt
