# Copyright 2019 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import unittest

import ament_index_python

import launch
import launch.actions

import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertSequentialStdout

import pytest


@pytest.mark.rostest
def generate_test_description():
    file_path = os.path.dirname(__file__)
    # TEST_PROC_PATH = 'ros2 launch infra initialization.launch.py'

    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'

    dut_process = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'infra', 'initialization.launch.py'],
        env=proc_env, output='screen'
    )

    return launch.LaunchDescription([
        dut_process,

        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest(),
    ]), {'dut_process': dut_process}


# These tests will run concurrently with the dut process.  After all these tests are done,
# the launch system will shut down the processes that it started up
class TestGoodProcess(unittest.TestCase):
    # TODO: add setup and teardown methods
    def test_count_to_four(self, proc_output):
        msg = 'Goal succeeded'
        proc_output.assertWaitFor(msg, timeout=300, stream='stdout')

class TestBadProcess(unittest.TestCase):
    def test(self, proc_output):
        msg = 'Goal failed'
        proc_output.assertWaitFor(msg, timeout=50, stream='stdout')

# @launch_testing.post_shutdown_test()
# class TestProcessOutput(unittest.TestCase):

#     def test_exit_code(self, proc_info):
#         # Check that all processes in the launch (in this case, there's just one) exit
#         # with code 0
#         launch_testing.asserts.assertExitCodes(proc_info)

    # def test_full_output(self, proc_output, dut_process):
    #     # Using the SequentialStdout context manager asserts that the following stdout
    #     # happened in the same order that it's checked
    #     with assertSequentialStdout(proc_output, dut_process) as cm:
    #         cm.assertInStdout('Starting Up')
    #         for n in range(4):
    #             cm.assertInStdout('Loop {}'.format(n))
    #         if os.name != 'nt':
    #             # On Windows, process termination is always forced
    #             # and thus the last print in good_proc never makes it.
    #             cm.assertInStdout('Shutting Down')

    # def test_out_of_order(self, proc_output, dut_process):
    #     # This demonstrates that we notice out-of-order IO
    #     with self.assertRaisesRegex(AssertionError, "'Loop 2' not found"):
    #         with assertSequentialStdout(proc_output, dut_process) as cm:
    #             cm.assertInStdout('Loop 1')
    #             cm.assertInStdout('Loop 3')
    #             cm.assertInStdout('Loop 2')  # This should raise
