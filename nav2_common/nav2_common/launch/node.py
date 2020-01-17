# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright (c) 2019 Intel Corporation
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

"""Module for the Node action."""

import os
import pathlib
from tempfile import NamedTemporaryFile
from typing import cast
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text  # noqa: F401
from typing import Tuple  # noqa: F401
from typing import Union

from launch import Condition
from launch.action import Action
from launch.actions import ExecuteProcess
from launch.frontend import Entity
from launch.frontend import Parser
from launch.launch_context import LaunchContext
import launch.logging
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import LocalSubstitution
from launch.substitutions import TextSubstitution
from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

import launch_ros
from launch_ros.parameters_type import SomeParameters
from launch_ros.remap_rule_type import SomeRemapRules
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.utilities import evaluate_parameters
from launch_ros.utilities import normalize_parameters
from launch_ros.utilities import normalize_remap_rules

from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name

import yaml

class Node(launch_ros.actions.Node):
    def __init__(
        self, *,
        use_remappings: Optional[Condition] = None,
        **kwargs
    ) -> None:
        """
        Construct an Node action.

        Many arguments are passed eventually to
        :class:`launch.actions.ExecuteProcess`, so see the documentation of
        that class for additional details.
        However, the `cmd` is not meant to be used, instead use the
        `node_executable` and `arguments` keyword arguments to this function.

        This action, once executed, delegates most work to the
        :class:`launch.actions.ExecuteProcess`, but it also converts some ROS
        specific arguments into generic command line arguments.

        The launch_ros.substitutions.ExecutableInPackage substitution is used
        to find the executable at runtime, so this Action also raise the
        exceptions that substituion can raise when the package or executable
        are not found.

        If the node_name is not given (or is None) then no name is passed to
        the node on creation and instead the default name specified within the
        code of the node is used instead.

        The node_namespace can either be absolute (i.e. starts with /) or
        relative.
        If absolute, then nothing else is considered and this is passed
        directly to the node to set the namespace.
        If relative, the namespace in the 'ros_namespace' LaunchConfiguration
        will be prepended to the given relative node namespace.
        If no node_namespace is given, then the default namespace `/` is
        assumed.

        The parameters are passed as a list, with each element either a yaml
        file that contains parameter rules (string or pathlib.Path to the full
        path of the file), or a dictionary that specifies parameter rules.
        Keys of the dictionary can be strings or an iterable of Substitutions
        that will be expanded to a string.
        Values in the dictionary can be strings, integers, floats, or tuples
        of Substitutions that will be expanded to a string.
        Additionally, values in the dictionary can be lists of the
        aforementioned types, or another dictionary with the same properties.
        A yaml file with the resulting parameters from the dictionary will be
        written to a temporary file, the path to which will be passed to the
        node.
        Multiple dictionaries/files can be passed: each file path will be
        passed in in order to the node (where the last definition of a
        parameter takes effect).

        :param: package the package in which the node executable can be found
        :param: node_executable the name of the executable to find
        :param: node_name the name of the node
        :param: node_namespace the ros namespace for this Node
        :param: parameters list of names of yaml files with parameter rules,
            or dictionaries of parameters.
        :param: use_remappings whether to apply the remappings at runtime.
        :param: remappings ordered list of 'to' and 'from' string pairs to be
            passed to the node as ROS remapping rules
        :param: arguments list of extra arguments for the node
        """
        super().__init__(**kwargs)
        self.__use_remappings = use_remappings

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        if self.__use_remappings is not None and not self.__use_remappings.evaluate(context):
            self.__expanded_remappings = None
        return super().execute(context)
