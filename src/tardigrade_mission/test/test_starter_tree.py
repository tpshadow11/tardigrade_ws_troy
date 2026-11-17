import math

import py_trees
import pytest

from tardigrade_mission.bt_behaviours import create_starter_tree

DT = 0.1


class FakeVehicle:
    """A point robot that follows mission velocities exactly."""

    def __init__(self, x=1.0, y=2.0, z=0.7, yaw=0.5):
        self.x, self.y, self.z, self.yaw = x, y, z, yaw
        self.time = 0.0
        self.last_command = (0.0, 0.0, 0.0)
        self.rate = 0.0
        self.odometry_fresh = True
        self.stops = 0

    def odometry_age(self):
        return 0.0 if self.odometry_fresh else math.inf

    def pose(self):
        return self.x, self.y, self.z, self.yaw

    def yaw_rate(self):
        return self.rate

    def now(self):
        return self.time

    def command(self, surge, heave, yaw_rate):
        self.last_command = (surge, heave, yaw_rate)

    def stop(self):
        self.stops += 1
        self.command(0.0, 0.0, 0.0)

    def step(self):
        surge, heave, yaw_rate = self.last_command
        self.x += surge * math.cos(self.yaw) * DT
        self.y += surge * math.sin(self.yaw) * DT
        self.z += heave * DT
        self.yaw += yaw_rate * DT
        self.rate = yaw_rate
        self.time += DT


def run(tree, vehicle, max_ticks, before_tick=None):
    for tick in range(max_ticks):
        if before_tick:
            before_tick(tick)
        tree.tick()
        if tree.root.status != py_trees.common.Status.RUNNING:
            return tree.root.status
        vehicle.step()
    return tree.root.status


@pytest.fixture(autouse=True)
def clear_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield


def test_starter_mission_goes_out_and_comes_back():
    vehicle = FakeVehicle()
    tree = py_trees.trees.BehaviourTree(
        create_starter_tree(vehicle, depth_m=1.0, distance_m=3.0))
    tree.setup(timeout=1.0)

    status = run(tree, vehicle, max_ticks=3000)

    assert status == py_trees.common.Status.SUCCESS
    assert vehicle.z == pytest.approx(-0.3, abs=0.1)
    assert math.hypot(vehicle.x - 1.0, vehicle.y - 2.0) < 0.5
    assert vehicle.last_command == (0.0, 0.0, 0.0)


def test_stale_odometry_stops_the_mission():
    vehicle = FakeVehicle()
    tree = py_trees.trees.BehaviourTree(
        create_starter_tree(vehicle, depth_m=1.0, distance_m=3.0))
    tree.setup(timeout=1.0)

    def lose_odometry(tick):
        if tick == 100:
            vehicle.odometry_fresh = False

    status = run(tree, vehicle, max_ticks=3000, before_tick=lose_odometry)

    assert status == py_trees.common.Status.FAILURE
    assert vehicle.last_command == (0.0, 0.0, 0.0)
    assert vehicle.stops >= 1
