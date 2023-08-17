#!/usr/bin/env python3
#
# Copyright 2023 REMAKE.AI
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

import rclpy
from rclpy.node import Node
# https://roboticsbackend.com/ros2-yaml-params/

class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_lin_vel', 0.22),
                ('max_ang_vel', 2.84),
                ('lin_vel_step_size', 0.01),
                ('ang_vel_step_size', 0.1)
            ])
        print('hello')

def main(args=None):
    print('hello2')
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
