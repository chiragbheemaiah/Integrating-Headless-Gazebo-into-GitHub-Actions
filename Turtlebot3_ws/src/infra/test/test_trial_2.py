import os
import sys
import unittest
import rclpy
import ament_index_python
import signal
import launch
import launch.actions
import time

import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertSequentialStdout
from launch_testing.tools import TimerAction

import pytest


@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    dut_process = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'infra', 'initialization_failure.launch.py'],
        env=proc_env, output='screen'
    )

    # Set up a timer to send Ctrl+C to the process after 150 seconds
    ctrl_c_timer = TimerAction(
        actions=[
            launch.actions.EmitEvent(event=launch.events.Shutdown()),
            launch.actions.LogInfo(msg="Sent Ctrl+C signal to the process after 150 seconds.")
        ],
        period=150.0
    )

    return launch.LaunchDescription([
        dut_process,

        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest(),
        ctrl_c_timer
    ]), {'dut_process': dut_process}


# These tests will run concurrently with the dut process.  After all these tests are done,
# the launch system will shut down the processes that it started up
class TestGoodProcess(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()
    def test_count_to_four(self, proc_output):
        msg = 'Goal failed'
        proc_output.assertWaitFor(msg, timeout=150, stream='stdout')
