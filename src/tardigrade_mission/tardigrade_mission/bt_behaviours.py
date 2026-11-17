"""Behaviour tree leaves for Tardigrade missions.

Every leaf talks to the robot only through a ``vehicle`` object, so the tree
can be tested without ROS. ``starter_tree.RosVehicle`` is the ROS version.

A vehicle provides:

- ``odometry_age()``: seconds since the last filtered odometry (inf if none)
- ``pose()``: ``(x, y, z, yaw)`` in the ENU ``odom`` frame
- ``yaw_rate()``: body yaw rate in rad/s
- ``now()``: current time in seconds (simulation time in Unity)
- ``command(surge, heave, yaw_rate)``: publish one mission velocity
- ``stop()``: publish a zero mission velocity
"""

import math

import py_trees

Status = py_trees.common.Status
START_KEYS = ('start_x', 'start_y', 'start_z', 'start_yaw')


def heading_error(target, current):
    """Return the shortest signed angle from current to target."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


def clamp(value, limit):
    return max(-abs(limit), min(abs(limit), value))


class OdometryFresh(py_trees.behaviour.Behaviour):
    """SUCCESS while filtered odometry is arriving, FAILURE otherwise."""

    def __init__(self, vehicle, max_age_sec=0.5, name='Odometry fresh?'):
        super().__init__(name)
        self.vehicle = vehicle
        self.max_age_sec = max_age_sec

    def update(self):
        if self.vehicle.odometry_age() <= self.max_age_sec:
            return Status.SUCCESS
        return Status.FAILURE


class RecordStart(py_trees.behaviour.Behaviour):
    """Save the current pose so later legs can be measured from it."""

    def __init__(self, vehicle, name='Record start pose'):
        super().__init__(name)
        self.vehicle = vehicle
        self.blackboard = self.attach_blackboard_client(name)
        for key in START_KEYS:
            self.blackboard.register_key(
                key, access=py_trees.common.Access.WRITE)

    def update(self):
        x, y, z, yaw = self.vehicle.pose()
        self.blackboard.start_x = x
        self.blackboard.start_y = y
        self.blackboard.start_z = z
        self.blackboard.start_yaw = yaw
        return Status.SUCCESS


class VelocityAction(py_trees.behaviour.Behaviour):
    """Publish a velocity every tick until done() is true.

    Subclasses fill in done() and command(). Whenever the leaf stops, for
    any reason (finished, failed, timed out, or interrupted by a guard), it
    publishes zero.
    """

    def __init__(self, vehicle, name, yaw_kp=0.4, yaw_kd=0.1, max_yaw=0.2):
        super().__init__(name)
        self.vehicle = vehicle
        self.yaw_kp = yaw_kp
        self.yaw_kd = yaw_kd
        self.max_yaw = max_yaw
        self.blackboard = self.attach_blackboard_client(name)
        for key in START_KEYS:
            self.blackboard.register_key(
                key, access=py_trees.common.Access.READ)

    def update(self):
        if self.done():
            self.vehicle.stop()
            return Status.SUCCESS
        surge, heave, yaw_rate = self.command()
        self.vehicle.command(surge, heave, yaw_rate)
        return Status.RUNNING

    def terminate(self, new_status):
        if new_status != Status.SUCCESS:
            self.vehicle.stop()

    def target_yaw(self, offset):
        return self.blackboard.start_yaw + offset

    def hold_heading(self, target):
        """Small PD loop that turns toward a target heading."""
        _, _, _, yaw = self.vehicle.pose()
        error = heading_error(target, yaw)
        rate = self.yaw_kp * error - self.yaw_kd * self.vehicle.yaw_rate()
        return clamp(rate, self.max_yaw)

    def done(self):
        raise NotImplementedError

    def command(self):
        raise NotImplementedError


class Descend(VelocityAction):
    """Go down depth_m below the start pose, keeping the start heading."""

    def __init__(self, vehicle, depth_m, tolerance_m=0.05, max_heave=0.2,
                 name=None):
        super().__init__(vehicle, name or f'Descend {depth_m:g} m')
        self.depth_m = abs(depth_m)
        self.tolerance_m = tolerance_m
        self.max_heave = max_heave

    def target_z(self):
        # ENU: up is +z, so deeper is more negative.
        return self.blackboard.start_z - self.depth_m

    def done(self):
        _, _, z, _ = self.vehicle.pose()
        return z <= self.target_z() + self.tolerance_m

    def command(self):
        _, _, z, _ = self.vehicle.pose()
        heave = clamp(0.8 * (self.target_z() - z), self.max_heave)
        return 0.0, heave, self.hold_heading(self.target_yaw(0.0))


class DriveLeg(VelocityAction):
    """Drive forward distance_m on (start heading + heading_offset)."""

    def __init__(self, vehicle, distance_m, heading_offset=0.0, speed=0.2,
                 name=None):
        super().__init__(vehicle, name or f'Drive {distance_m:g} m')
        self.distance_m = distance_m
        self.heading_offset = heading_offset
        self.speed = speed
        self.leg_x = None
        self.leg_y = None

    def initialise(self):
        # Runs each time this leaf starts, so the leg is measured from here.
        self.leg_x, self.leg_y, _, _ = self.vehicle.pose()

    def done(self):
        x, y, _, _ = self.vehicle.pose()
        heading = self.target_yaw(self.heading_offset)
        along = ((x - self.leg_x) * math.cos(heading)
                 + (y - self.leg_y) * math.sin(heading))
        return along >= self.distance_m

    def command(self):
        heading = self.target_yaw(self.heading_offset)
        return self.speed, 0.0, self.hold_heading(heading)


class TurnTo(VelocityAction):
    """Turn in place to (start heading + heading_offset)."""

    def __init__(self, vehicle, heading_offset, tolerance_rad=0.07,
                 name=None):
        super().__init__(
            vehicle,
            name or f'Turn to {math.degrees(heading_offset):.0f} deg')
        self.heading_offset = heading_offset
        self.tolerance_rad = tolerance_rad

    def done(self):
        _, _, _, yaw = self.vehicle.pose()
        target = self.target_yaw(self.heading_offset)
        return abs(heading_error(target, yaw)) <= self.tolerance_rad

    def command(self):
        return 0.0, 0.0, self.hold_heading(
            self.target_yaw(self.heading_offset))


class HoldStill(VelocityAction):
    """Command zero speed while holding a heading for a fixed time."""

    def __init__(self, vehicle, seconds, heading_offset=0.0, name=None):
        super().__init__(vehicle, name or f'Hold {seconds:g} s')
        self.seconds = seconds
        self.heading_offset = heading_offset
        self.end_time = None

    def initialise(self):
        self.end_time = self.vehicle.now() + self.seconds

    def done(self):
        return self.vehicle.now() >= self.end_time

    def command(self):
        return 0.0, 0.0, self.hold_heading(
            self.target_yaw(self.heading_offset))


def leg(child, seconds):
    """Fail the leg if it runs longer than seconds."""
    return py_trees.decorators.Timeout(
        child=child, name=f'{child.name} (max {seconds:g} s)',
        duration=seconds)


def create_starter_tree(vehicle, depth_m=1.0, distance_m=3.0, speed=0.2,
                        leg_timeout_sec=60.0, odometry_timeout_sec=0.5):
    """Build the starter mission tree.

    Qual uses the same shape with a 1.5 m dive and 20 m legs.
    """
    wait_for_odometry = py_trees.decorators.Timeout(
        name='Wait for odometry (max 30 s)',
        duration=30.0,
        child=py_trees.decorators.FailureIsRunning(
            OdometryFresh(vehicle, odometry_timeout_sec)),
    )
    legs = py_trees.composites.Sequence('Legs', memory=True, children=[
        leg(Descend(vehicle, depth_m), leg_timeout_sec),
        HoldStill(vehicle, 2.0),
        leg(DriveLeg(vehicle, distance_m, 0.0, speed, name='Drive out'),
            leg_timeout_sec),
        HoldStill(vehicle, 2.0),
        leg(TurnTo(vehicle, math.pi, name='Turn around'),
            leg_timeout_sec),
        HoldStill(vehicle, 2.0, math.pi),
        leg(DriveLeg(vehicle, distance_m, math.pi, speed,
                     name='Drive back'),
            leg_timeout_sec),
        HoldStill(vehicle, 2.0, math.pi),
    ])
    safety = py_trees.decorators.EternalGuard(
        name='Safety: odometry still fresh?',
        condition=lambda: vehicle.odometry_age() <= odometry_timeout_sec,
        child=legs,
    )
    return py_trees.composites.Sequence(
        'Starter mission', memory=True,
        children=[wait_for_odometry, RecordStart(vehicle), safety])
