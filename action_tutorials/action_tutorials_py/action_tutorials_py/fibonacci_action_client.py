#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from action_tutorials_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from objgraph import show_growth


class FibonacciActionClient(Node):

    def __init__(self, name):
        super().__init__(name)
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci', callback_group=self.callback_group)
        self.rate = self.create_rate(5)

    async def call_async(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # self._action_client.wait_for_server()

        goal_future = self._action_client.send_goal_async(goal_msg)

        await goal_future

        if goal_future is None:
            self.get_logger().info('send goal fail')
            return
        
        goal_handle = goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        result_future = goal_handle.get_result_async()
        await result_future

        if result_future is None:
            self.get_logger().info('get result fail')
            return
    
        result = result_future.result().result
        # self.get_logger().info('Result: {0}'.format(result.sequence))

    async def timer_callback(self):
        start = self.get_clock().now()
        await self.call_async(3)
        end = self.get_clock().now()
        self.get_logger().info('timer: time: {0} s'.format((end - start).nanoseconds / 1e9))
        show_growth()

    async def run(self):
        while rclpy.ok():
            start = self.get_clock().now()
            await self.call_async(3)
            end = self.get_clock().now()
            self.get_logger().info('task: time: {0} s'.format((end - start).nanoseconds / 1e9))

            show_growth()
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)

    action_client_num = 1
    action_clients = [FibonacciActionClient('fibonacci_{}'.format(i)) for i in range(action_client_num)]

    for action_client in action_clients:
        action_client.create_timer(0.3, action_client.timer_callback, callback_group=action_client.callback_group)

    executor = rclpy.executors.MultiThreadedExecutor()
    for action_client in action_clients:
        executor.add_node(action_client)
        executor.create_task(action_client.run)
    executor.spin()


if __name__ == '__main__':
    main()
