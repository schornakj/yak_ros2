import os
import unittest

from ament_index_python import get_package_share_directory, get_package_prefix
import shutil
import tempfile

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

import launch
import launch.actions
import launch_ros.actions


import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import tf2_ros
from std_srvs.srv import Trigger

import pytest
import time

@pytest.mark.launch_test
# @launch_testing.markers.keep_alive
def generate_test_description():
    yak_node = launch_ros.actions.Node(
        node_name='yak_ros2_node', package='yak_ros2', node_executable='yak_ros2_node', output='screen',
        remappings=[('input_depth_image', '/image')],
        parameters=[{
                     'tsdf_frame_id': 'tsdf_origin',
                     'camera_intrinsic_params':
                       {
                         'fx': 550.0,
                         'fy': 550.0,
                         'cx': 320.0,
                         'cy': 240.0,
                       },
                     'cols': 640,
                     'rows': 480,
                     'volume_resolution': 0.001,
                     'volume_x': 640,
                     'volume_y': 640,
                     'volume_z': 192,
                     }]
        )

    image_sim_node = launch_ros.actions.Node(
        node_name='yak_ros2_image_simulator', package='yak_ros2', node_executable='yak_ros2_image_simulator', output='screen',
        parameters=[{
                     'base_frame': 'world',
                     'orbit_speed': 6.0,
                     'framerate': 120.0,
                     'mesh': os.path.join(get_package_share_directory('yak_ros2'), 'demo', 'bun_on_table.ply'),
                     }]
        )

    static_tf_pub = launch_ros.actions.Node(
        node_name='static_tf_publisher', package='tf2_ros', node_executable='static_transform_publisher', output='screen',
        arguments=['-0.3', '-0.3', '-0.01', '0', '0', '0', '1', 'world', 'tsdf_origin'],
        )

    return launch.LaunchDescription([
        image_sim_node,
        static_tf_pub,
        yak_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSetTransforms(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_manager_node')
        self.mesh_srv_client = self.node.create_client(Trigger, '/generate_mesh_service')

    def tearDown(self):
        self.node.destroy_node()

    # def testStartup(self, proc_info, proc_output):
    #     proc_info.assertWaitForStartup()

    def testGenerateMesh(self, proc_output):
        # wait a short time for the surface to be reconstructed
        time.sleep(2.0)
        future = self.mesh_srv_client.call_async(Trigger.Request())
        rclpy.spin_once(self.node)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        assert future.result().success is True
        assert os.path.isfile('cubes.ply')
        # TODO: check similarity between original and reconstructed mesh (metric for this?)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
