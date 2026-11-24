// The only thing behavior tree nodes know about the robot.
//
// Nodes never touch ROS directly. They call these functions instead. The ROS
// version (RosVehicle in mission_runner.cpp) reads odometry and publishes
// velocity setpoints; the test version (FakeVehicle in test/) is a pretend
// robot, so whole trees can be tested in milliseconds without Unity.
#pragma once

#include <cmath>
#include <limits>

namespace tardigrade_bt
{

// Position in the ENU "odom" frame: x forward at start, y left, z UP
// (so diving makes z more negative). yaw is the heading in radians.
struct Pose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

class Vehicle
{
public:
  virtual ~Vehicle() = default;

  // Current time in seconds (simulation time when use_sim_time is true).
  virtual double now() const = 0;
  // Seconds since the last odometry message, or infinity if none yet.
  virtual double odometryAge() const = 0;
  // True when the robot reports that it is armed. Trees never arm the robot.
  virtual bool armed() const = 0;
  virtual Pose pose() const = 0;
  // Body yaw rate in rad/s (positive = turning left).
  virtual double yawRate() const = 0;
  // Send one body-frame velocity setpoint: m/s and rad/s.
  // surge = forward, sway = left, heave = up, yaw_rate = turn left.
  virtual void command(double surge, double sway, double heave, double yaw_rate) = 0;

  void stop() {command(0.0, 0.0, 0.0, 0.0);}

  // The start pose is the reference for every move ("heading_deg 90" means
  // 90 degrees left of the heading the robot had at RecordStart).
  void recordStart() {start_ = pose(); has_start_ = true;}
  bool hasStart() const {return has_start_;}
  const Pose & start() const {return start_;}

private:
  Pose start_;
  bool has_start_{false};
};

}  // namespace tardigrade_bt
