// mission_runner: load a behavior tree XML file and tick it at a fixed rate.
//
//   ros2 run tardigrade_bt mission_runner --ros-args
//       -p tree:=playground -p dry_run:=false -p use_sim_time:=true
//
// dry_run (the default) only loads and prints the tree, then exits. Nothing
// moves until you pass dry_run:=false, and the robot must already be armed by
// a person: trees check IsArmed but never arm the robot.

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/abstract_logger.h>
#ifdef TARDIGRADE_BT_GROOT2
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#endif

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tardigrade_interfaces/msg/robot_status.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "tardigrade_bt/nodes.hpp"
#include "tardigrade_bt/vehicle.hpp"

namespace tardigrade_bt
{

// The Vehicle interface backed by ROS topics.
class RosVehicle : public Vehicle
{
public:
  RosVehicle(
    rclcpp::Node & node, const std::string & odometry_topic,
    const std::string & status_topic, const std::string & command_topic)
  : node_(node)
  {
    command_pub_ = node.create_publisher<geometry_msgs::msg::TwistStamped>(command_topic, 10);
    odometry_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        const auto & p = message->pose.pose;
        const auto & q = p.orientation;
        pose_.x = p.position.x;
        pose_.y = p.position.y;
        pose_.z = p.position.z;
        pose_.yaw = std::atan2(
          2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        yaw_rate_ = message->twist.twist.angular.z;
        odometry_time_ = now();
        has_odometry_ = true;
      });
    status_sub_ = node.create_subscription<tardigrade_interfaces::msg::RobotStatus>(
      status_topic, 10, [this](tardigrade_interfaces::msg::RobotStatus::ConstSharedPtr message) {
        armed_ = message->armed;
      });
  }

  double now() const override {return node_.get_clock()->now().seconds();}

  double odometryAge() const override
  {
    return has_odometry_ ? now() - odometry_time_ : std::numeric_limits<double>::infinity();
  }

  bool armed() const override {return armed_;}
  Pose pose() const override {return pose_;}
  double yawRate() const override {return yaw_rate_;}

  void command(double surge, double sway, double heave, double yaw_rate) override
  {
    geometry_msgs::msg::TwistStamped message;
    message.header.stamp = node_.get_clock()->now();
    message.header.frame_id = "base_link";
    message.twist.linear.x = surge;
    message.twist.linear.y = sway;
    message.twist.linear.z = heave;
    message.twist.angular.z = yaw_rate;
    command_pub_->publish(message);
  }

private:
  rclcpp::Node & node_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<tardigrade_interfaces::msg::RobotStatus>::SharedPtr status_sub_;
  Pose pose_;
  double yaw_rate_{0.0};
  double odometry_time_{0.0};
  bool has_odometry_{false};
  bool armed_{false};
};

// Prints and publishes "<node name>: <STATUS>" for actions and conditions,
// skipping the noise: resets to IDLE, and conditions that are re-checked every
// tick with the same answer. Foxglove and rosbag can show the status topic.
class MissionLogger : public BT::StatusChangeLogger
{
public:
  MissionLogger(
    const BT::Tree & tree, rclcpp::Logger logger,
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
  : BT::StatusChangeLogger(tree.rootNode()), logger_(logger),
    publisher_(std::move(publisher)) {}

  void callback(
    BT::Duration, const BT::TreeNode & node, BT::NodeStatus,
    BT::NodeStatus status) override
  {
    if (status == BT::NodeStatus::IDLE) {
      return;
    }
    if (node.type() != BT::NodeType::ACTION && node.type() != BT::NodeType::CONDITION) {
      return;
    }
    auto & last = last_status_[&node];
    if (last == status) {
      return;
    }
    last = status;
    const std::string text = node.name() + ": " + BT::toStr(status);
    RCLCPP_INFO(logger_, "%s", text.c_str());
    std_msgs::msg::String message;
    message.data = text;
    publisher_->publish(message);
  }

  void flush() override {}

private:
  rclcpp::Logger logger_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::map<const BT::TreeNode *, BT::NodeStatus> last_status_;
};

class MissionRunner : public rclcpp::Node
{
public:
  MissionRunner()
  : rclcpp::Node("mission_runner")
  {
    const auto tree_name = declare_parameter<std::string>("tree", "playground");
    dry_run_ = declare_parameter<bool>("dry_run", true);
    const auto tick_hz = declare_parameter<double>("tick_hz", 10.0);
    const auto odometry_topic = declare_parameter<std::string>(
      "odometry_topic", "/tardigrade/state/odometry/filtered");
    const auto status_topic = declare_parameter<std::string>(
      "robot_status_topic", "/tardigrade/status");
    const auto command_topic = declare_parameter<std::string>(
      "command_topic", "/tardigrade/control/velocity_setpoint/mission");
    const auto groot_port = declare_parameter<int>("groot_port", 1667);

    vehicle_ = std::make_unique<RosVehicle>(*this, odometry_topic, status_topic, command_topic);
    registerNodes(factory_, vehicle_.get());

    const auto path = resolveTreePath(tree_name);
    RCLCPP_INFO(get_logger(), "Loading tree file %s", path.c_str());
    tree_ = factory_.createTreeFromFile(path);
    RCLCPP_INFO(get_logger(), "Tree structure:");
    BT::printTreeRecursively(tree_.rootNode());

    if (dry_run_) {
      RCLCPP_WARN(
        get_logger(),
        "dry_run is true: the tree loaded fine but will NOT run. "
        "Pass -p dry_run:=false to run it (arm the robot yourself first).");
      return;
    }

    status_pub_ = create_publisher<std_msgs::msg::String>("/tardigrade/mission/status", 10);
    mission_logger_ = std::make_unique<MissionLogger>(tree_, get_logger(), status_pub_);
#ifdef TARDIGRADE_BT_GROOT2
    groot_ = std::make_unique<BT::Groot2Publisher>(tree_, static_cast<unsigned>(groot_port));
    RCLCPP_INFO(get_logger(), "Groot2 can connect on port %ld", groot_port);
#else
    (void)groot_port;
#endif

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, tick_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this]() {tick();});
  }

  bool finished() const {return dry_run_ || finished_;}
  bool failed() const {return result_ == BT::NodeStatus::FAILURE;}

  void stopRobot()
  {
    if (!dry_run_) {
      tree_.haltTree();
      vehicle_->stop();
    }
  }

private:
  std::string resolveTreePath(const std::string & tree) const
  {
    // A plain name like "playground" means trees/playground.xml in this package.
    if (tree.find('/') == std::string::npos && tree.find(".xml") == std::string::npos) {
      return ament_index_cpp::get_package_share_directory("tardigrade_bt") +
             "/trees/" + tree + ".xml";
    }
    return tree;
  }

  void tick()
  {
    if (finished_) {
      return;
    }
    const auto status = tree_.tickOnce();
    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      vehicle_->stop();
      finished_ = true;
      result_ = status;
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "Mission finished: %s", BT::toStr(status).c_str());
    }
  }

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<RosVehicle> vehicle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::unique_ptr<MissionLogger> mission_logger_;
#ifdef TARDIGRADE_BT_GROOT2
  std::unique_ptr<BT::Groot2Publisher> groot_;
#endif
  rclcpp::TimerBase::SharedPtr timer_;
  bool dry_run_{true};
  bool finished_{false};
  BT::NodeStatus result_{BT::NodeStatus::IDLE};
};

}  // namespace tardigrade_bt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<tardigrade_bt::MissionRunner>();
    while (rclcpp::ok() && !node->finished()) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (rclcpp::ok()) {
      node->stopRobot();
      exit_code = node->failed() ? 1 : 0;
    }
    // After Ctrl+C ROS is already shut down and cannot publish a final zero;
    // the velocity mux zeroes stale mission commands within 0.5 s.
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("mission_runner"), "%s", error.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
