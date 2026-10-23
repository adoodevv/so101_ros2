#!/usr/bin/env python3
"""
End-to-end test of leader-arm teleoperation on simulated servos.

Two so101_servo_sim buses stand in for the arms: the leader starts at a
fixed pose (near the SRDF `ready` pose, gripper part open), the follower
starts folded as the real arm rests. so101_bringup/so101_teleop.launch.py
runs the follower on so101_hardware and so101_leader in teleop mode.
Through the public ROS interfaces the tests check that:

1. so101_leader publishes the leader's pose on /leader/joint_states, mapped
   to the follower's joint convention;
2. the follower moves to the leader's pose no faster than sync_velocity and
   then matches it;
3. when the leader stops answering, so101_leader reports it and stops
   commanding (the follower holds still), and when it answers again the
   follower re-syncs.
"""

import json
import math
import os
import signal
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import pytest
from rcl_interfaces.msg import Log
import rclpy
from sensor_msgs.msg import JointState

ARM = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
GRIPPER_CLOSED, GRIPPER_OPEN = -0.174533, 1.74533
LEADER_POSE = {'shoulder_pan': 0.3, 'shoulder_lift': -0.7, 'elbow_flex': 0.6,
               'wrist_flex': 1.4, 'wrist_roll': 0.0, 'gripper': 1.0}
TICKS_PER_RAD = 4095 / (2 * math.pi)
# so101_leader's default sync_velocity (rad/s).
SYNC_VELOCITY = 0.5
# Allows for servo resolution (~0.0015 rad per tick) and rounding.
POSITION_TOLERANCE = 0.01
TIMEOUT = 60.0


def leader_start_ticks(calibration):
    """Return so101_servo_sim --start arguments placing the leader at LEADER_POSE."""
    args = []
    for joint in ARM:
        c = calibration[joint]
        middle = (c['range_min'] + c['range_max']) / 2
        args.append(f'{joint}={round(middle + LEADER_POSE[joint] * TICKS_PER_RAD)}')
    g = calibration['gripper']
    fraction = (LEADER_POSE['gripper'] - GRIPPER_CLOSED) / (GRIPPER_OPEN - GRIPPER_CLOSED)
    args.append(f"gripper={round(g['range_min'] + fraction * (g['range_max'] - g['range_min']))}")
    return args


def servo_sim(calibration_file, link, start):
    return ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('so101_hardware'), 'lib', 'so101_hardware',
                          'so101_servo_sim'),
             '--calibration', calibration_file, '--link', link, '--start', *start],
        output='screen')


@pytest.mark.launch_test
def generate_test_description():
    """Start both simulated arms, then the teleop launch once both are up."""
    config = os.path.join(get_package_share_directory('so101_hardware'), 'config')
    leader_calibration = os.path.join(config, 'example_leader_calibration.json')
    follower_calibration = os.path.join(config, 'example_follower_calibration.json')
    with open(leader_calibration, encoding='utf-8') as f:
        leader_start = leader_start_ticks(json.load(f))
    with open(follower_calibration, encoding='utf-8') as f:
        ranges = json.load(f)
    folded = [f"shoulder_lift={ranges['shoulder_lift']['range_min'] + 30}",
              f"elbow_flex={ranges['elbow_flex']['range_max'] - 15}"]

    tmp = tempfile.gettempdir()
    leader_port = os.path.join(tmp, f'so101_leader_port_{os.getpid()}')
    follower_port = os.path.join(tmp, f'so101_follower_port_{os.getpid()}')
    leader_sim = servo_sim(leader_calibration, leader_port, leader_start)
    follower_sim = servo_sim(follower_calibration, follower_port, folded)
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('so101_bringup'), 'launch', 'so101_teleop.launch.py')),
        launch_arguments={
            'port': follower_port, 'calibration_file': follower_calibration,
            'leader_port': leader_port, 'leader_calibration_file': leader_calibration,
            'use_rviz': 'false',
        }.items())

    up = set()

    def start_when_both_up(name):
        def handler(event):
            if b'servos on' in event.text:
                up.add(name)
                if up == {'leader', 'follower'}:
                    up.add('started')
                    return [teleop]
            return None
        return handler

    return LaunchDescription([
        leader_sim, follower_sim,
        RegisterEventHandler(OnProcessIO(
            target_action=leader_sim, on_stdout=start_when_both_up('leader'))),
        RegisterEventHandler(OnProcessIO(
            target_action=follower_sim, on_stdout=start_when_both_up('follower'))),
        launch_testing.actions.ReadyToTest(),
    ]), {'leader_sim': leader_sim}


class TestTeleop(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('so101_teleop_test')
        cls.leader = None
        cls.follower = []  # (stamp seconds, {joint: position}), from the start
        cls.logs = []  # (stamp seconds, text)
        cls.node.create_subscription(
            JointState, '/leader/joint_states', cls._on_leader, 10)
        cls.node.create_subscription(JointState, '/joint_states', cls._on_follower, 100)
        cls.node.create_subscription(Log, '/rosout', cls._on_log, 100)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_leader(cls, msg):
        cls.leader = dict(zip(msg.name, msg.position))

    @classmethod
    def _on_follower(cls, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        cls.follower.append((stamp, dict(zip(msg.name, msg.position))))

    @classmethod
    def _on_log(cls, msg):
        if msg.name == 'so101_leader':
            cls.logs.append((msg.stamp.sec + msg.stamp.nanosec * 1e-9, msg.msg))

    def spin_until(self, predicate, what, timeout=TIMEOUT):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                self.fail(f'Timed out waiting for {what}')
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def logged(self, text, since=0):
        return any(text in line for _, line in self.logs[since:])

    def test_01_leader_pose_published(self):
        """/leader/joint_states carries the leader's pose in the follower's convention."""
        self.spin_until(lambda: self.leader is not None, '/leader/joint_states')
        for joint, expected in LEADER_POSE.items():
            self.assertAlmostEqual(
                self.leader[joint], expected, delta=POSITION_TOLERANCE,
                msg=f'leader {joint} at {self.leader[joint]:.4f}, expected {expected:.4f}')

    def test_02_follower_syncs_slowly_then_matches(self):
        """The follower reaches the leader's pose, never faster than sync_velocity."""
        self.spin_until(
            lambda: self.logged('Follower matches the leader'), 'the follower to sync')
        self.spin_for(1.0)
        _, final = self.follower[-1]
        for joint, expected in LEADER_POSE.items():
            self.assertAlmostEqual(
                final[joint], expected, delta=POSITION_TOLERANCE,
                msg=f'follower {joint} at {final[joint]:.4f}, leader at {expected:.4f}')

        # Speed over >= 0.2 s windows of joint_states stamps that end before
        # so101_leader reported the sync (after it, the last few hundredths of
        # a radian close at max_velocity). shoulder_lift starts 1.1 rad away.
        synced_at = next(stamp for stamp, line in self.logs if 'Follower matches' in line)
        fastest = 0.0
        start = 0
        for end in range(len(self.follower)):
            while self.follower[end][0] - self.follower[start][0] > 0.2:
                start += 1
            t0, a = self.follower[max(start - 1, 0)]
            t1, b = self.follower[end]
            if t1 - t0 >= 0.2 and t1 < synced_at:
                fastest = max(fastest, max(abs(b[j] - a[j]) / (t1 - t0) for j in ARM))
        self.assertGreater(fastest, 0.5 * SYNC_VELOCITY, 'the follower never moved')
        self.assertLessEqual(
            fastest, SYNC_VELOCITY * 1.1,
            f'the follower moved at {fastest:.2f} rad/s while syncing')

    def test_03_leader_dropout_holds_then_resyncs(self, leader_sim):
        """A silent leader stops the commands; when it answers again the follower re-syncs."""
        pid = leader_sim.process_details['pid']
        since = len(self.logs)
        os.kill(pid, signal.SIGSTOP)
        try:
            self.spin_until(
                lambda: self.logged('Leader not answering', since), 'the dropout report', 10.0)
            _, held = self.follower[-1]
            self.spin_for(1.0)
            _, after = self.follower[-1]
            for joint in held:
                self.assertAlmostEqual(after[joint], held[joint], delta=1e-3,
                                       msg=f'{joint} moved while the leader was silent')
        finally:
            os.kill(pid, signal.SIGCONT)
        self.spin_until(
            lambda: self.logged('Moving the follower', since), 're-sync after the dropout', 10.0)
        self.spin_until(
            lambda: self.logged('Follower matches the leader', since), 'the re-sync', 20.0)
