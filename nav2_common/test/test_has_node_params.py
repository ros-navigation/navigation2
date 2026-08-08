# Copyright (c) 2026 aineoae86-sys
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

import launch
from nav2_common.launch import HasNodeParams


def perform_has_node_params(yaml_text):
    test_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
    try:
        test_yaml.write(yaml_text)
        test_yaml.close()

        substitution = HasNodeParams(
            source_file=test_yaml.name,
            node_name='slam_toolbox',
        )
        return substitution.perform(launch.LaunchContext())
    finally:
        if not test_yaml.closed:
            test_yaml.close()
        os.unlink(test_yaml.name)


def test_has_node_params_returns_true_for_present_node():
    assert perform_has_node_params(
        'slam_toolbox:\n  ros__parameters:\n    use_sim_time: true\n'
    ) == 'True'


def test_has_node_params_returns_false_for_missing_node():
    assert perform_has_node_params(
        'map_server:\n  ros__parameters:\n    use_sim_time: true\n'
    ) == 'False'


def test_has_node_params_returns_false_for_empty_yaml():
    assert perform_has_node_params('') == 'False'


def test_has_node_params_returns_false_for_non_mapping_yaml():
    assert perform_has_node_params('- slam_toolbox\n') == 'False'
