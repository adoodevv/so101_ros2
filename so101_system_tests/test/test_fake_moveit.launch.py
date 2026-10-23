#!/usr/bin/env python3
"""
End-to-end system tests for the SO-101 stack on fake hardware.

Launches so101_bringup/so101_fake_moveit.launch.py headless (no Gazebo, no
RViz) and checks, through the public ROS interfaces only, that:

1. joint_state_broadcaster, arm_controller and gripper_controller are active;
2. the arm plans and executes through MoveIt to the SRDF named states
   `home` then `ready`;
3. the gripper goes to `open`, `closed`, then `open` through MoveIt;
4. an arm goal outside the joint limits never moves a joint past its limit
   (it is either rejected with no motion, or clamped to the limit by MoveIt);
5. a trajectory sent straight to arm_controller, bypassing MoveIt, is clamped
   to the URDF limits by the controller manager (enforce_command_limits).

Target joint values come from the installed SRDF and limits from
/robot_description, so the tests follow the robot config rather than
duplicating it. The tests are numbered because they share one launched stack
and run in order.
"""

import os
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


ROBOT_NAME = 'so101'
EXPECTED_CONTROLLERS = ('joint_state_broadcaster', 'arm_controller', 'gripper_controller')

# Per-test time budget in seconds; every blocking wait inside a test is
# bounded by what is left of it. The CTest TIMEOUT bounds the whole run.
STARTUP_TIMEOUT = 90.0
MOTION_TEST_TIMEOUT = 60.0
REJECTION_TEST_TIMEOUT = 30.0

# Fake hardware tracks commands exactly; this allows for controller goal
# tolerances and floating point noise.
POSITION_TOLERANCE = 0.01
# Overshoot past the URDF upper limit used for the out-of-limits goal.
OUT_OF_LIMITS_MARGIN = 0.5
# move_group planning attempts per goal. OMPL is randomized; a couple of
# retries is ordinary client practice and keeps a rare bad sample from
# failing the run.
PLANNING_ATTEMPTS = 3
# How long to watch for further motion after the out-of-limits goal returns.
NO_MOTION_WINDOW = 2.0


@pytest.mark.launch_test
def generate_test_description():
    """Launch the fake-hardware MoveIt stack without RViz."""
    fake_moveit_launch = os.path.join(
        get_package_share_directory('so101_bringup'), 'launch', 'so101_fake_moveit.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fake_moveit_launch),
            launch_arguments={'use_rviz': 'false'}.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


def load_srdf_group_states():
    """Return {(group, state): {joint: value}} from the installed SRDF."""
    srdf_path = os.path.join(
        get_package_share_directory('so101_moveit_config'),
        'config', ROBOT_NAME, f'{ROBOT_NAME}.srdf')
    states = {}
    for state in ET.parse(srdf_path).getroot().iter('group_state'):
        states[(state.get('group'), state.get('name'))] = {
            joint.get('name'): float(joint.get('value')) for joint in state.iter('joint')}
    return states


class TestFakeMoveIt(unittest.TestCase):
    """Drive the launched stack through its ROS interfaces."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('so101_system_test')
        cls.group_states = load_srdf_group_states()
        cls.joint_state = None
        cls.joint_limits = None

        cls.node.create_subscription(JointState, '/joint_states', cls._on_joint_state, 10)
        cls.node.create_subscription(
            String, '/robot_description', cls._on_robot_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.list_controllers = cls.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        cls.move_group = ActionClient(cls.node, MoveGroup, '/move_action')
        cls.arm_controller = ActionClient(
            cls.node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    @classmethod
    def tearDownClass(cls):
        cls.move_group.destroy()
        cls.arm_controller.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_joint_state(cls, msg):
        cls.joint_state = msg

    @classmethod
    def _on_robot_description(cls, msg):
        limits = {}
        for joint in ET.fromstring(msg.data).iter('joint'):
            limit = joint.find('limit')
            if joint.get('type') == 'revolute' and limit is not None:
                limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
        cls.joint_limits = limits

    # --- helpers -----------------------------------------------------------

    def start_deadline(self, seconds):
        self.deadline = time.monotonic() + seconds

    def remaining(self):
        left = self.deadline - time.monotonic()
        if left <= 0.0:
            self.fail(f'{self.id()} exceeded its time budget')
        return left

    def spin_until(self, predicate, what):
        """Spin until predicate() is truthy or the test budget runs out."""
        while not predicate():
            if time.monotonic() >= self.deadline:
                self.fail(f'Timed out waiting for {what}')
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def wait_future(self, future, what):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.remaining())
        if not future.done():
            self.fail(f'Timed out waiting for {what}')
        return future.result()

    def fresh_joint_positions(self):
        """Return {joint: position} from a /joint_states message received from now on."""
        self.__class__.joint_state = None
        self.spin_until(lambda: self.joint_state is not None, '/joint_states')
        return dict(zip(self.joint_state.name, self.joint_state.position))

    def controller_states(self):
        future = self.list_controllers.call_async(ListControllers.Request())
        response = self.wait_future(future, 'list_controllers response')
        return {c.name: c.state for c in response.controller}

    def move_to_joints(self, group, targets):
        """Plan and execute a joint-space goal through move_group; return the error code."""
        self.assertTrue(
            self.move_group.wait_for_server(timeout_sec=self.remaining()),
            'move_group action server /move_action not available')

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = group
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 1.0
        request.max_acceleration_scaling_factor = 1.0
        request.start_state.is_diff = True
        request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(
                joint_name=name, position=position,
                tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0)
            for name, position in targets.items()
        ])]
        goal.planning_options.plan_only = False
        # Re-plan if a plan is rejected; see PLANNING_ATTEMPTS.
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = PLANNING_ATTEMPTS

        goal_handle = self.wait_future(
            self.move_group.send_goal_async(goal), f'{group} goal acceptance')
        if not goal_handle.accepted:
            return None
        result = self.wait_future(goal_handle.get_result_async(), f'{group} goal result')
        return result.result.error_code.val

    def assert_named_state_reached(self, group, state):
        targets = self.group_states[(group, state)]
        error_code = self.move_to_joints(group, targets)
        self.assertEqual(
            error_code, MoveItErrorCodes.SUCCESS,
            f'MoveIt failed to reach {group}/{state}: error code {error_code}')
        positions = self.fresh_joint_positions()
        for joint, target in targets.items():
            self.assertAlmostEqual(
                positions[joint], target, delta=POSITION_TOLERANCE,
                msg=f'{group}/{state}: {joint} at {positions[joint]:.4f}, expected {target:.4f}')

    # --- tests -------------------------------------------------------------

    def test_01_controllers_active(self):
        """All three controllers reach the active state."""
        self.start_deadline(STARTUP_TIMEOUT)
        self.assertTrue(
            self.list_controllers.wait_for_service(timeout_sec=self.remaining()),
            '/controller_manager/list_controllers not available')
        states = {}

        def all_active():
            states.update(self.controller_states())
            if all(states.get(name) == 'active' for name in EXPECTED_CONTROLLERS):
                return True
            time.sleep(0.5)
            return False

        try:
            self.spin_until(all_active, 'controllers to become active')
        except AssertionError as error:
            raise AssertionError(f'{error}; last controller states: {states}') from None

    def test_02_arm_home_then_ready(self):
        """The arm reaches SRDF named states home then ready through MoveIt."""
        self.start_deadline(MOTION_TEST_TIMEOUT)
        self.assert_named_state_reached('arm', 'home')
        self.assert_named_state_reached('arm', 'ready')

    def test_03_gripper_open_closed_open(self):
        """The gripper reaches SRDF named states open, closed, open through MoveIt."""
        self.start_deadline(MOTION_TEST_TIMEOUT)
        for state in ('open', 'closed', 'open'):
            self.assert_named_state_reached('gripper', state)

    def test_04_out_of_limits_goal_never_exceeds_limits(self):
        """
        An arm goal beyond the URDF joint limits never drives a joint past them.

        Two outcomes are acceptable:
        - move_group rejects the goal or fails, and nothing moves;
        - move_group clamps the out-of-bounds target to the joint limit and
          stops there. MoveIt 2 JointConstraint does this, logging "Joint
          <name> is constrained to be above the maximum bounds. Assuming
          maximum bounds instead."
        Any position past a limit fails the test.
        """
        self.start_deadline(REJECTION_TEST_TIMEOUT)
        self.spin_until(lambda: self.joint_limits is not None, '/robot_description')

        before = self.fresh_joint_positions()
        arm_joints = self.group_states[('arm', 'home')].keys()
        targets = {joint: before[joint] for joint in arm_joints}
        _, upper = self.joint_limits['shoulder_pan']
        targets['shoulder_pan'] = upper + OUT_OF_LIMITS_MARGIN

        error_code = self.move_to_joints('arm', targets)

        settle_until = time.monotonic() + NO_MOTION_WINDOW
        self.spin_until(lambda: time.monotonic() >= settle_until, 'settle window')
        after = self.fresh_joint_positions()

        for joint, (lower, upper_limit) in self.joint_limits.items():
            self.assertGreaterEqual(
                after[joint], lower - POSITION_TOLERANCE, f'{joint} below its lower limit')
            self.assertLessEqual(
                after[joint], upper_limit + POSITION_TOLERANCE, f'{joint} above its upper limit')

        if error_code == MoveItErrorCodes.SUCCESS:
            # Clamped: shoulder_pan stopped at the limit; the other joints
            # were goal-constrained to their current values, so they may only
            # drift within the planner's goal tolerance.
            self.assertAlmostEqual(
                after['shoulder_pan'], upper, delta=POSITION_TOLERANCE,
                msg='shoulder_pan was not clamped to its upper limit')
            held = [joint for joint in before if joint != 'shoulder_pan']
            delta = POSITION_TOLERANCE
        else:
            # Rejected or failed: nothing may move.
            held = list(before)
            delta = 1e-4
        for joint in held:
            self.assertAlmostEqual(
                after[joint], before[joint], delta=delta,
                msg=f'{joint} moved from {before[joint]:.4f} to {after[joint]:.4f}')

    def test_05_direct_controller_command_clamped_to_limits(self):
        """A trajectory sent straight to arm_controller cannot drive a joint past its limit."""
        self.start_deadline(REJECTION_TEST_TIMEOUT)
        self.spin_until(lambda: self.joint_limits is not None, '/robot_description')
        self.assertTrue(
            self.arm_controller.wait_for_server(timeout_sec=self.remaining()),
            'arm_controller follow_joint_trajectory action not available')

        before = self.fresh_joint_positions()
        arm_joints = list(self.group_states[('arm', 'home')])
        _, upper = self.joint_limits['shoulder_pan']
        targets = {joint: before[joint] for joint in arm_joints}
        targets['shoulder_pan'] = upper + OUT_OF_LIMITS_MARGIN

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = arm_joints
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[targets[joint] for joint in arm_joints],
            time_from_start=Duration(sec=1))]
        goal_handle = self.wait_future(
            self.arm_controller.send_goal_async(goal), 'arm_controller goal acceptance')
        if goal_handle.accepted:
            self.wait_future(goal_handle.get_result_async(), 'arm_controller goal result')

        settle_until = time.monotonic() + NO_MOTION_WINDOW
        self.spin_until(lambda: time.monotonic() >= settle_until, 'settle window')
        after = self.fresh_joint_positions()
        for joint, (lower, upper_limit) in self.joint_limits.items():
            self.assertGreaterEqual(
                after[joint], lower - POSITION_TOLERANCE, f'{joint} below its lower limit')
            self.assertLessEqual(
                after[joint], upper_limit + POSITION_TOLERANCE, f'{joint} above its upper limit')
