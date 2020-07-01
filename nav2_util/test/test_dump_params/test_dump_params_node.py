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

import sys

from rcl_interfaces.msg import FloatingPointRange, IntegerRange
import rclpy
from rclpy.node import Node, ParameterDescriptor


class TestDumpParamsNode(Node):

    def __init__(self, name):
        Node.__init__(self, name)
        self.declare_parameter('param_not_set',
                               descriptor=ParameterDescriptor(description='not set'))
        self.declare_parameter('param_bool',
                               True,
                               ParameterDescriptor(description='boolean', read_only=True))
        self.declare_parameter('param_int',
                               1234,
                               ParameterDescriptor(integer_range=[IntegerRange(
                                   from_value=-1000, to_value=10000, step=2)]))
        self.declare_parameter('param_double', 3.14)
        self.declare_parameter('param_string', 'foobar')
        self.declare_parameter('param_bool_array', [True, False])
        self.declare_parameter('param_int_array',  [1, 2, 3])
        self.declare_parameter('param_double_array',
                               [1.50000, 23.50120, 123.0010],
                               ParameterDescriptor(floating_point_range=[FloatingPointRange(
                                   from_value=-1000.5, to_value=1000.5, step=0.0001)])
                               )
        self.declare_parameter('param_string_array', ['foo', 'bar'])
        self.declare_parameter('param_byte_array', [b'\x01', b'\x02', b'\x03', b'\x04'])


def main(args=None):
    rclpy.init(args=args)

    name = 'test_dump_params'
    if len(sys.argv) > 1:
        name = sys.argv[1]

    node = TestDumpParamsNode(name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
