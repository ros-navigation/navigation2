# Copyright (c) 2025 Leander Stephen Desouza
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
import tempfile
from typing import Generator

import launch
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
import pytest
import yaml


class TestRewrittenYamlValueRewrites:
    """Test that value rewrites work correctly in RewrittenYaml."""

    @pytest.fixture(autouse=True)
    def setup_teardown(self) -> Generator[None, None, None]:
        # Create a temporary YAML file for testing
        self.test_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
        self.test_yaml.write("""\
            param0: placeholder_bool
            param1: placeholder_string
            param2: placeholder_float
            param3: placeholder_int
            param4: placeholder_list
            param5: placeholder_dict
            nested:
                param6: placeholder_bool
                param7: placeholder_string
                param8: placeholder_float
                param9: placeholder_int
                param10: placeholder_list
                param11: placeholder_dict
                other_value: 42
            list_values:
                - placeholder_bool
                - placeholder_string
                - placeholder_float
                - placeholder_int
                - placeholder_list
                - placeholder_dict
                - normal_value
        """)
        self.test_yaml.close()
        yield
        os.unlink(self.test_yaml.name)

    def test_value_rewrites(self) -> None:
        """Test that value rewrites work for various types."""
        # Set up launch configurations for our test values
        launch_configurations = {
            'test_bool': 'true',
            'test_string': 'replaced_string',
            'test_float': '3.14',
            'test_int': '100',
            'test_list': '["string", 42, 2.718, ["sublist_item1", "sublist_item2"]]',
            'test_dict': ('{"key1": "value1", "key2": 2, "key3": 3.14, '
                           '"key4": ["list_item1", "list_item2"]}')
        }
        context = launch.LaunchContext()
        for key, value in launch_configurations.items():
            context.launch_configurations[key] = value

        value_rewrites = {
            'placeholder_bool': LaunchConfiguration('test_bool'),
            'placeholder_string': LaunchConfiguration('test_string'),
            'placeholder_float': LaunchConfiguration('test_float'),
            'placeholder_int': LaunchConfiguration('test_int'),
            'placeholder_list': LaunchConfiguration('test_list'),
            'placeholder_dict': LaunchConfiguration('test_dict'),
        }

        rewriter = RewrittenYaml(
            source_file=self.test_yaml.name,
            param_rewrites={},
            value_rewrites=value_rewrites,
            convert_types=True,
        )
        output_file = rewriter.perform(context)

        try:
            with open(output_file) as f:
                result = yaml.safe_load(f)

            assert result['param0'] is True
            assert result['param1'] == 'replaced_string'
            assert result['param2'] == 3.14
            assert result['param3'] == 100
            assert result['param4'] == \
                '["string", 42, 2.718, ["sublist_item1", "sublist_item2"]]'
            assert result['param5'] == \
                '{"key1": "value1", "key2": 2, "key3": 3.14, "key4": ["list_item1", "list_item2"]}'

            # Check nested values
            assert result['nested']['param6'] is True
            assert result['nested']['param7'] == 'replaced_string'
            assert result['nested']['param8'] == 3.14
            assert result['nested']['param9'] == 100
            assert result['nested']['param10'] == \
                '["string", 42, 2.718, ["sublist_item1", "sublist_item2"]]'
            assert result['nested']['param11'] == \
                '{"key1": "value1", "key2": 2, "key3": 3.14, "key4": ["list_item1", "list_item2"]}'

            # Check list values
            assert result['list_values'][0] is True
            assert result['list_values'][1] == 'replaced_string'
            assert result['list_values'][2] == 3.14
            assert result['list_values'][3] == 100
            assert result['list_values'][4] == \
                '["string", 42, 2.718, ["sublist_item1", "sublist_item2"]]'
            assert result['list_values'][5] == \
                '{"key1": "value1", "key2": 2, "key3": 3.14, "key4": ["list_item1", "list_item2"]}'

            # Check other values remain unchanged
            assert result['nested']['other_value'] == 42
            assert result['list_values'][6] == 'normal_value'

        finally:
            if os.path.exists(output_file):
                os.unlink(output_file)
