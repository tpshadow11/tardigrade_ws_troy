// Tests the real tree XML files against a pretend robot. No ROS or Unity
// needed: each test runs a whole mission in a few milliseconds.

#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <thread>

#include "tardigrade_bt/nodes.hpp"
#include "tardigrade_bt/vehicle.hpp"

namespace
{

using tardigrade_bt::Pose;

constexpr double kDt = 0.1;  // seconds of pretend time per tick

// A point robot that follows velocity commands exactly.
class FakeVehicle : public tardigrade_bt::Vehicle
{
public:
  double now() const override {return time;}
  double odometryAge() const override
  {
    return odometry_ok ? 0.0 : std::numeric_limits<double>::infinity();
  }
  bool armed() const override {return is_armed;}
  Pose pose() const override {return p;}
  double yawRate() const override {return rate;}

  void command(double surge, double sway, double heave, double yaw_rate) override
  {
    last = {surge, sway, heave, yaw_rate};
    if (heave > 0.0 && surge == 0.0 && sway == 0.0 && yaw_rate == 0.0) {
      saw_surface_command = true;
    }
  }

  void step()
  {
    if (!stuck) {
      const double c = std::cos(p.yaw);
      const double s = std::sin(p.yaw);
      p.x += (last[0] * c - last[1] * s) * kDt;
      p.y += (last[0] * s + last[1] * c) * kDt;
      p.z += last[2] * kDt;
      p.yaw += last[3] * kDt;
      rate = last[3];
    }
    time += kDt;
  }

  bool lastCommandIsZero() const
  {
    return last[0] == 0.0 && last[1] == 0.0 && last[2] == 0.0 && last[3] == 0.0;
  }

  Pose p;
  double time{0.0};
  double rate{0.0};
  std::array<double, 4> last{0.0, 0.0, 0.0, 0.0};
  bool odometry_ok{true};
  bool is_armed{true};
  bool stuck{false};
  bool saw_surface_command{false};
};

BT::NodeStatus run(
  BT::Tree & tree, FakeVehicle & vehicle, int max_ticks,
  const std::function<void(int)> & before_tick = nullptr)
{
  for (int tick = 0; tick < max_ticks; ++tick) {
    if (before_tick) {
      before_tick(tick);
    }
    const auto status = tree.tickOnce();
    if (status != BT::NodeStatus::RUNNING) {
      return status;
    }
    vehicle.step();
  }
  return BT::NodeStatus::RUNNING;
}

std::string treeFile(const std::string & name)
{
  return std::string(TREES_DIR) + "/" + name + ".xml";
}

class TreeTest : public ::testing::Test
{
protected:
  void SetUp() override {tardigrade_bt::registerNodes(factory, &vehicle);}

  FakeVehicle vehicle;
  BT::BehaviorTreeFactory factory;
};

TEST_F(TreeTest, PlaygroundFinishesWhereExpected)
{
  auto tree = factory.createTreeFromFile(treeFile("playground"));
  EXPECT_EQ(run(tree, vehicle, 5000), BT::NodeStatus::SUCCESS);
  // Forward 2, back 2, turn left, forward 2: ends 2 m to the left, 0.5 m down.
  EXPECT_NEAR(vehicle.p.x, 0.0, 0.2);
  EXPECT_NEAR(vehicle.p.y, 2.0, 0.2);
  EXPECT_NEAR(vehicle.p.z, -0.5, 0.1);
  EXPECT_TRUE(vehicle.lastCommandIsZero());
}

TEST_F(TreeTest, SquareComesBackToStart)
{
  vehicle.p = {1.0, -3.0, -0.2, 0.7};
  auto tree = factory.createTreeFromFile(treeFile("square"));
  EXPECT_EQ(run(tree, vehicle, 5000), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(vehicle.p.x, 1.0, 0.3);
  EXPECT_NEAR(vehicle.p.y, -3.0, 0.3);
  EXPECT_NEAR(tardigrade_bt::angleDiff(vehicle.p.yaw, 0.7), 0.0, 0.1);
}

TEST_F(TreeTest, DiveTestReturnsToStartDepth)
{
  vehicle.p.z = -0.4;
  auto tree = factory.createTreeFromFile(treeFile("dive_test"));
  EXPECT_EQ(run(tree, vehicle, 2000), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(vehicle.p.z, -0.4, 0.06);
}

TEST_F(TreeTest, LostOdometrySurfacesAndFails)
{
  auto tree = factory.createTreeFromFile(treeFile("playground"));
  const auto status = run(
    tree, vehicle, 5000, [this](int tick) {
      if (tick == 100) {vehicle.odometry_ok = false;}
    });
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_TRUE(vehicle.saw_surface_command);
  EXPECT_TRUE(vehicle.lastCommandIsZero());
}

TEST_F(TreeTest, DisarmingStopsTheMission)
{
  auto tree = factory.createTreeFromFile(treeFile("square"));
  const auto status = run(
    tree, vehicle, 5000, [this](int tick) {
      if (tick == 50) {vehicle.is_armed = false;}
    });
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_TRUE(vehicle.lastCommandIsZero());
}

TEST_F(TreeTest, NeverArmedMeansNothingMoves)
{
  vehicle.is_armed = false;
  auto tree = factory.createTreeFromFile(treeFile("playground"));
  EXPECT_EQ(run(tree, vehicle, 5000), BT::NodeStatus::FAILURE);
  EXPECT_NEAR(vehicle.p.x, 0.0, 1e-9);
  EXPECT_NEAR(vehicle.p.y, 0.0, 1e-9);
}

TEST_F(TreeTest, StuckLegTimesOutAndStops)
{
  // Timeout uses the real clock, so this test uses a 200 ms timeout and
  // sleeps between ticks.
  vehicle.stuck = true;
  const char * xml = R"(
    <root BTCPP_format="4" main_tree_to_execute="T">
      <BehaviorTree ID="T">
        <Sequence>
          <RecordStart/>
          <Timeout msec="200"><Forward distance_m="5.0"/></Timeout>
        </Sequence>
      </BehaviorTree>
    </root>)";
  auto tree = factory.createTreeFromText(xml);
  const auto status = run(
    tree, vehicle, 200, [](int) {std::this_thread::sleep_for(std::chrono::milliseconds(5));});
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_TRUE(vehicle.lastCommandIsZero());
}

TEST_F(TreeTest, MoveWithoutRecordStartFails)
{
  const char * xml = R"(
    <root BTCPP_format="4" main_tree_to_execute="T">
      <BehaviorTree ID="T"><Forward distance_m="1.0"/></BehaviorTree>
    </root>)";
  auto tree = factory.createTreeFromText(xml);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(TreeTest, AngleDiffTakesTheShortWayRound)
{
  const double pi = 3.14159265358979323846;
  EXPECT_NEAR(tardigrade_bt::angleDiff(0.1, 2 * pi - 0.1), 0.2, 1e-9);
  EXPECT_NEAR(std::abs(tardigrade_bt::angleDiff(-pi / 2, pi / 2)), pi, 1e-9);
}

}  // namespace
