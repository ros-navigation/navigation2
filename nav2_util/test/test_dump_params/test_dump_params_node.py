#! /usr/bin/env python3
# Copyright (c) 2020 Sarthak Mittal
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


class TestDumpParamsNode(Node):

    def __init__(self):
        Node.__init__(self, 'test_dump_params')
        self.declare_parameter('param_not_set')
        self.declare_parameter('param_bool', True)
        self.declare_parameter('param_int', 123456)
        self.declare_parameter('param_double', 3.14)
        self.declare_parameter('param_string', 'foobar')
        self.declare_parameter('param_bool_array', [True, False])
        self.declare_parameter('param_int_array',  [1, 2, 3])
        self.declare_parameter('param_double_array', [1.50000, 23.50120, 123.0010])
        self.declare_parameter('param_string_array', ['foo', 'bar'])
        self.declare_parameter('param_byte_array', [0x52, 0x4f, 0x53, 0x32])


def main(args=None):
    rclpy.init(args=args)

    node = TestDumpParamsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
