# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os  
from ament_index_python.packages import get_package_share_directory

from clearpath_config.common.utils.yaml import read_yaml
from clearpath_config.clearpath_config import ClearpathConfig

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import PushRosNamespace, SetRemap
from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('setup_path',
                          default_value='/etc/clearpath/',
                          description='Clearpath setup path'),
    DeclareLaunchArgument('scan_topic',
                           default_value='sensors/lidar3d_0/scan',
                           description='Relative scan topic (without namespace)' ),
    DeclareLaunchArgument('odom_topic',
                           default_value='platform/odom',
                           description='Relative odom topic (without namespace)'),
]


def launch_setup(context, *args, **kwargs):
    # Packages
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    scan_topic = LaunchConfiguration('scan_topic')
    odom_topic = LaunchConfiguration('odom_topic') 

    # Read robot YAML
    config = read_yaml(setup_path.perform(context) + 'robot.yaml')
    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    file_parameters = PathJoinSubstitution([
        pkg_clearpath_nav2_demos,
        'config',
        platform_model,
        'nav2.yaml'])
    rewritten_parameters = RewrittenYaml(
        source_file=file_parameters,
        root_key='',
        param_rewrites = {
            "bt_navigator.ros__parameters.odom_topic": odom_topic,
            "velocity_smoother.ros__parameters.odom_topic": odom_topic,

            "local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic": scan_topic,
            "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic": scan_topic,
            "collision_monitor.ros__parameters.scan.topic": scan_topic
        },
        convert_types=True)
    launch_nav2 = PathJoinSubstitution(
      [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    nav2 = GroupAction([
        PushRosNamespace(namespace),     
        SetRemap(
            PathJoinSubstitution(['/', namespace, 'global_costmap', scan_topic]),
            PathJoinSubstitution(['/', namespace, scan_topic])
        ),
        SetRemap(
            PathJoinSubstitution(['/', namespace, 'local_costmap', scan_topic]),
            PathJoinSubstitution(['/', namespace, scan_topic])
        ),
        SetRemap(
            PathJoinSubstitution(['/', namespace, 'odom']),
            PathJoinSubstitution(['/', namespace, odom_topic])
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('params_file', rewritten_parameters),
                ('use_composition', 'False'),
                ('namespace', namespace)
              ]
        ),
    ])

    return [nav2]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
