# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Tests for the Node Action."""

import os
import pathlib
import unittest

from launch import LaunchDescription
from launch import LaunchService
from launch.substitutions import EnvironmentVariable
import launch_ros.actions.node
import yaml


class TestNode(unittest.TestCase):

    def _assert_launch_errors(self, actions):
        ld = LaunchDescription(actions)
        ls = LaunchService()
        ls.include_launch_description(ld)
        assert 0 != ls.run()

    def _assert_launch_no_errors(self, actions):
        ld = LaunchDescription(actions)
        ls = LaunchService()
        ls.include_launch_description(ld)
        assert 0 == ls.run()

    def _create_node(self, *, parameters=None, remappings=None):
        return launch_ros.actions.Node(
            package='demo_nodes_py', node_executable='talker_qos', output='screen',
            # The node name is required for parameter dicts.
            # See https://github.com/ros2/launch/issues/139.
            node_name='my_node', node_namespace='my_ns',
            arguments=['--number_of_cycles', '1'],
            parameters=parameters,
            remappings=remappings,
        )

    def _assert_type_error_creating_node(self, *, parameters=None, remappings=None):
        with self.assertRaises(TypeError):
            self._create_node(parameters=parameters, remappings=remappings)

    def test_launch_invalid_node(self):
        """Test launching an invalid node."""
        node_action = launch_ros.actions.Node(
            package='nonexistent_package', node_executable='node', output='screen'),
        self._assert_launch_errors([node_action])

    def test_launch_node(self):
        """Test launching a node."""
        self._assert_launch_no_errors([self._create_node()])

    def test_launch_node_with_remappings(self):
        """Test launching a node with remappings."""
        # Pass remapping rules to node in a variety of forms.
        # It is redundant to pass the same rule, but the goal is to test different parameter types.
        os.environ['TOPIC_NAME'] = 'chatter'
        topic_prefix = 'new_'
        node_action = self._create_node(
            remappings=[
                ('chatter', 'new_chatter'),
                (EnvironmentVariable(name='TOPIC_NAME'), [
                    topic_prefix, EnvironmentVariable(name='TOPIC_NAME')])
            ],
        )
        self._assert_launch_no_errors([node_action])

        # Check the expanded parameters.
        expanded_remappings = node_action._Node__expanded_remappings
        assert len(expanded_remappings) == 2
        for i in range(2):
            assert expanded_remappings[i] == ('chatter', 'new_chatter')

    def test_create_node_with_invalid_remappings(self):
        """Test creating a node with invalid remappings."""
        self._assert_type_error_creating_node(
            remappings={'chatter': 'new_chatter'},  # Not a list.
        )

        self._assert_type_error_creating_node(
            remappings=[{'chatter': 'new_chatter'}],  # List with elements not tuple.
        )

    def test_launch_node_with_parameter_files(self):
        """Test launching a node with parameters specified in yaml files."""
        parameters_file_dir = pathlib.Path(__file__).resolve().parent
        parameters_file_path = parameters_file_dir / 'example_parameters.yaml'
        # Pass parameter files to node in a variety of forms.
        # It is redundant to pass the same file, but the goal is to test different parameter types.
        os.environ['FILE_PATH'] = str(parameters_file_dir)
        node_action = self._create_node(
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH'), os.sep, 'example_parameters.yaml'],
            ],
        )
        self._assert_launch_no_errors([node_action])

        # Check the expanded parameters.
        expanded_parameter_files = node_action._Node__expanded_parameter_files
        assert len(expanded_parameter_files) == 3
        for i in range(3):
            assert expanded_parameter_files[i] == str(parameters_file_path)

    def test_launch_node_with_parameter_dict(self):
        """Test launching a node with parameters specified in a dictionary."""
        os.environ['PARAM1_VALUE'] = 'param1_value'
        os.environ['PARAM2'] = 'param2'
        node_action = self._create_node(
            parameters=[{
                'param1': EnvironmentVariable(name='PARAM1_VALUE'),
                EnvironmentVariable(name='PARAM2'): (EnvironmentVariable(name='PARAM2'), '_value'),
                'param_group1': {
                    'list_params': [1.2, 3.4],
                    'param_group2': {
                        (EnvironmentVariable('PARAM2'), '_values'): ['param2_value'],
                    }
                }
            }],
        )
        self._assert_launch_no_errors([node_action])

        # Check the expanded parameters (will be written to a file).
        expanded_parameter_files = node_action._Node__expanded_parameter_files
        assert len(expanded_parameter_files) == 1
        with open(expanded_parameter_files[0], 'r') as h:
            expanded_parameters_dict = yaml.load(h)
            assert expanded_parameters_dict == {
                '/my_ns': {
                    'my_node': {
                        'ros__parameters': {
                            'param1': 'param1_value',
                            'param2': 'param2_value',
                            'param_group1': {
                                'list_params': [1.2, 3.4],
                                'param_group2': {
                                    'param2_values': ['param2_value'],
                                }
                            }
                        }
                    }
                }
            }

    def test_create_node_with_invalid_parameters(self):
        """Test launching a node with invalid parameters."""
        self._assert_type_error_creating_node(parameters=[5.0])  # Invalid list values.
        self._assert_type_error_creating_node(parameters={'a': 5})  # Valid dict, not in a list.

        parameter_file_path = pathlib.Path(__file__).resolve().parent / 'example_parameters.yaml'
        self._assert_type_error_creating_node(
            parameters=str(parameter_file_path))  # Valid path, but not in a list.

        # If a parameter dictionary is specified, the node name must be also.
        with self.assertRaisesRegex(RuntimeError, 'node name must also be specified'):
            launch_ros.actions.Node(
                package='demo_nodes_py', node_executable='talker_qos', output='screen',
                arguments=['--number_of_cycles', '1'],
                parameters=[{'my_param': 'value'}],
            )

    def test_launch_node_with_invalid_parameter_dicts(self):
        """Test launching a node with invalid parameter dicts."""
        # Substitutions aren't expanded until the node action is executed, at which time a type
        # error should be raised and cause the launch to fail.
        # For each type of invalid parameter, check that they are detected at both the top-level
        # and at a nested level in the dictionary.

        # Key must be a string/Substitution evaluating to a string.
        self._assert_launch_errors(actions=[
            self._create_node(parameters=[{5: 'asdf'}])
        ])
        self._assert_launch_errors(actions=[
            self._create_node(parameters=[{
                'param_group': {
                    'param_subgroup': {
                        5: 'asdf',
                    },
                },
            }])
        ])

        # Nested lists are not supported.
        self._assert_launch_errors(actions=[
            self._create_node(parameters=[{'param': [1, 2, [3, 4]]}])
        ])
        self._assert_launch_errors(actions=[
            self._create_node(parameters=[{
                'param_group': {
                    'param_subgroup': {
                        'param': [1, 2, [3, 4]],
                    },
                },
            }])
        ])

        # Tuples are only supported for Substitutions.
        self._assert_launch_errors(actions=[
            self._create_node(parameters=[{'param': (1, 2)}])
        ])
        self._assert_launch_errors(actions=[
            self._create_node(parameters=[{
                'param_group': {
                    'param_subgroup': {
                        'param': (1, 2),
                    },
                },
            }])
        ])

        # Other types are not supported.
        self._assert_launch_errors(actions=[
            self._create_node(parameters=[{'param': {1, 2}}])
        ])
        self._assert_launch_errors(actions=[
            self._create_node(parameters=[{
                'param_group': {
                    'param_subgroup': {
                        'param': {1, 2},
                    },
                },
            }])
        ])
        self._assert_launch_errors(actions=[
            self._create_node(parameters=[{'param': self}])
        ])
        self._assert_launch_errors(actions=[
            self._create_node(parameters=[{
                'param_group': {
                    'param_subgroup': {
                        'param': self,
                    },
                },
            }])
        ])
