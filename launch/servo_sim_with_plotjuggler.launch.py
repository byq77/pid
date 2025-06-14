##############################################################################
#      Title     : servo_sim_with_plotjuggler.launch.py
#      Project   : pid
#      Created   : 6/14/2025
#      Author    : byq77
#
# BSD 3-Clause License
#
# Copyright (c) 2025, byq77
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
##############################################################################

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    LocalSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare the launch argument
    declare_use_plotjuggler_arg = DeclareLaunchArgument(
        'use_plotjuggler',
        default_value='true',
        description='Whether to launch PlotJuggler',
    )
    declare_use_setpoint_node_arg = DeclareLaunchArgument(
        'use_setpoint_node',
        default_value='true',
        description='Whether to launch the setpoint node',
    )
    launch_arguments = [declare_use_plotjuggler_arg, declare_use_setpoint_node_arg]

    use_plotjuggler = LaunchConfiguration('use_plotjuggler')
    use_setpoint_node = LaunchConfiguration('use_setpoint_node')

    layout_file = PathJoinSubstitution(
        [FindPackageShare('pid'), 'config', 'pid_controller_layout.xml']
    )

    pid_controller_node = Node(
        package='pid', executable='pid_controller', name='pid_controller', output='screen'
    )

    servo_sim_node = Node(package='pid', executable='servo_sim', name='servo_sim', output='screen')

    setpoint_node = Node(package='pid', executable='setpoint', name='setpoint', output='screen')

    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        arguments=[
            '--layout',
            layout_file,
            # '--start_streamer',
            # 'ROS2 Topic Subscriber',
            '--window_title',
            'PID Controller Visualization',
        ],
        output='screen',
    )

    launch_set_point_node_after_servo_sim_node = RegisterEventHandler(
        condition=IfCondition(use_setpoint_node),
        event_handler=OnProcessStart(
            target_action=servo_sim_node,
            on_start=[TimerAction(period=1.0, actions=[setpoint_node])],
        ),
    )

    launch_plotjuggler_after_pid_controller_node = RegisterEventHandler(
        condition=IfCondition(use_plotjuggler),
        event_handler=OnProcessStart(
            target_action=pid_controller_node,
            on_start=[TimerAction(period=1.0, actions=[plotjuggler_node])],
        ),
    )

    terminate_launch_on_plotjuggler_window_exit = RegisterEventHandler(
        condition=IfCondition(use_plotjuggler),
        event_handler=OnProcessExit(
            target_action=plotjuggler_node,
            on_exit=[
                LogInfo(
                    msg=(
                        EnvironmentVariable(name='USERNAME', default_value='USER'),
                        ' closed the turtlesim window',
                    )
                ),
                EmitEvent(event=Shutdown(reason='Window closed')),
            ],
        ),
    )

    display_message_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(
                    msg=[
                        'Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason'),
                    ]
                )
            ]
        )
    )

    return LaunchDescription(
        launch_arguments
        + [
            pid_controller_node,
            servo_sim_node,
            launch_set_point_node_after_servo_sim_node,
            launch_plotjuggler_after_pid_controller_node,
            terminate_launch_on_plotjuggler_window_exit,
            display_message_on_shutdown,
        ]
    )
