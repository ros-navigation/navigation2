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

"""Module for the Node action."""

import logging
import os
import pathlib
from tempfile import NamedTemporaryFile
from typing import Dict  # noqa: F401
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text  # noqa: F401
from typing import Tuple

from launch import Substitution
from launch.action import Action
from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.some_substitutions_type import SomeSubstitutionsType_types_tuple
from launch.substitutions import LocalSubstitution
from launch.utilities import ensure_argument_type
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

from launch_ros.substitutions import ExecutableInPackage

from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name

import yaml

_logger = logging.getLogger(name='launch_ros')


class Node(ExecuteProcess):
    """Action that executes a ROS node."""

    def __init__(
        self, *,
        package: SomeSubstitutionsType,
        node_executable: SomeSubstitutionsType,
        node_name: Optional[SomeSubstitutionsType] = None,
        node_namespace: Optional[SomeSubstitutionsType] = None,
        parameters: Optional[List[SomeSubstitutionsType]] = None,
        remappings: Optional[List[Tuple[SomeSubstitutionsType, SomeSubstitutionsType]]] = None,
        arguments: Optional[Iterable[SomeSubstitutionsType]] = None,
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
        :param: remappings ordered list of 'to' and 'from' string pairs to be
            passed to the node as ROS remapping rules
        :param: arguments list of extra arguments for the node
        """
        cmd = [ExecutableInPackage(package=package, executable=node_executable)]
        cmd += [] if arguments is None else arguments
        # Reserve space for ros specific arguments.
        # The substitutions will get expanded when the action is executed.
        ros_args_index = 0
        if node_name is not None:
            cmd += [LocalSubstitution(
                'ros_specific_arguments[{}]'.format(ros_args_index), description='node name')]
            ros_args_index += 1
        if node_namespace is not None:
            cmd += [LocalSubstitution(
                'ros_specific_arguments[{}]'.format(ros_args_index), description='node namespace')]
            ros_args_index += 1
        if parameters is not None:
            ensure_argument_type(parameters, (list), 'parameters', 'Node')
            # All elements in the list are paths to files with parameters (or substitutions that
            # evaluate to paths), or dictionaries of parameters (fields can be substitutions).
            parameter_types = list(SomeSubstitutionsType_types_tuple) + [pathlib.Path, dict]
            i = 0
            for param in parameters:
                ensure_argument_type(param, parameter_types, 'parameters[{}]'.format(i), 'Node')
                if isinstance(param, dict) and node_name is None:
                    raise RuntimeError(
                        'If a dictionary of parameters is specified, the node name must also be '
                        'specified. See https://github.com/ros2/launch/issues/139')
                i += 1
                cmd += [LocalSubstitution(
                    'ros_specific_arguments[{}]'.format(ros_args_index),
                    description='parameter {}'.format(i))]
                ros_args_index += 1
        if remappings is not None:
            ensure_argument_type(remappings, (list), 'remappings', 'Node')
            i = 0
            for remapping in remappings:
                ensure_argument_type(remapping, (tuple), 'remappings[{}]'.format(i), 'Node')
                k, v = remapping
                i += 1
                cmd += [LocalSubstitution(
                    'ros_specific_arguments[{}]'.format(ros_args_index),
                    description='remapping {}'.format(i))]
                ros_args_index += 1
        super().__init__(cmd=cmd, **kwargs)
        self.__package = package
        self.__node_executable = node_executable
        self.__node_name = node_name
        self.__node_namespace = node_namespace
        self.__parameters = [] if parameters is None else parameters
        self.__remappings = [] if remappings is None else remappings
        self.__arguments = arguments

        self.__expanded_node_name = '<node_name_unspecified>'
        self.__expanded_node_namespace = '/'
        self.__final_node_name = None  # type: Optional[Text]
        self.__expanded_parameter_files = None  # type: Optional[List[Text]]
        self.__expanded_remappings = None  # type: Optional[List[Tuple[Text, Text]]]

        self.__substitutions_performed = False

    @property
    def node_name(self):
        """Getter for node_name."""
        if self.__final_node_name is None:
            raise RuntimeError("cannot access 'node_name' before executing action")
        return self.__final_node_name

    def _create_params_file_from_dict(self, context, params):
        with NamedTemporaryFile(mode='w', prefix='launch_params_', delete=False) as h:
            param_file_path = h.name
            # TODO(dhood): clean up generated parameter files.

            def perform_substitution_if_applicable(context, var):
                if isinstance(var, (int, float, str)):
                    # No substitution necessary.
                    return var
                if isinstance(var, Substitution):
                    return perform_substitutions(context, normalize_to_list_of_substitutions(var))
                if isinstance(var, tuple):
                    try:
                        return perform_substitutions(
                            context, normalize_to_list_of_substitutions(var))
                    except TypeError:
                        raise TypeError(
                            'Invalid element received in parameters dictionary '
                            '(not all tuple elements are Substitutions): {}'.format(var))
                else:
                    raise TypeError(
                        'Unsupported type received in parameters dictionary: {}'
                        .format(type(var)))

            def expand_dict(input_dict):
                expanded_dict = {}
                for k, v in input_dict.items():
                    # Key (parameter/group name) can only be a string/Substitutions that evaluates
                    # to a string.
                    expanded_key = perform_substitutions(
                        context, normalize_to_list_of_substitutions(k))
                    if isinstance(v, dict):
                        # Expand the nested dict.
                        expanded_value = expand_dict(v)
                    elif isinstance(v, list):
                        # Expand each element.
                        expanded_value = []
                        for e in v:
                            if isinstance(e, list):
                                raise TypeError(
                                    'Nested lists are not supported for parameters: {} found in {}'
                                    .format(e, v))
                            expanded_value.append(perform_substitution_if_applicable(context, e))
                    # Tuples are treated as Substitution(s) to be concatenated.
                    elif isinstance(v, tuple):
                        for e in v:
                            ensure_argument_type(
                                e, SomeSubstitutionsType_types_tuple,
                                'parameter dictionary tuple entry', 'Node')
                        expanded_value = perform_substitutions(
                            context, normalize_to_list_of_substitutions(v))
                    else:
                        expanded_value = perform_substitution_if_applicable(context, v)
                    expanded_dict[expanded_key] = expanded_value
                return expanded_dict

            expanded_dict = expand_dict(params)
            param_dict = {
                self.__expanded_node_name: {'ros__parameters': expanded_dict}}
            if self.__expanded_node_namespace:
                param_dict = {self.__expanded_node_namespace: param_dict}
            yaml.dump(param_dict, h, default_flow_style=False)
            return param_file_path

    def _perform_substitutions(self, context: LaunchContext) -> None:
        try:
            if self.__substitutions_performed:
                # This function may have already been called by a subclass' `execute`, for example.
                return
            self.__substitutions_performed = True
            if self.__node_name is not None:
                self.__expanded_node_name = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_name))
                validate_node_name(self.__expanded_node_name)
            self.__expanded_node_name.lstrip('/')
            if self.__node_namespace is not None:
                self.__expanded_node_namespace = perform_substitutions(
                    context, normalize_to_list_of_substitutions(self.__node_namespace))
            if not self.__expanded_node_namespace.startswith('/'):
                self.__expanded_node_namespace = '/' + self.__expanded_node_namespace
            validate_namespace(self.__expanded_node_namespace)
        except Exception:
            _logger.error(
                "Error while expanding or validating node name or namespace for '{}':"
                .format('package={}, node_executable={}, name={}, namespace={}'.format(
                    self.__package,
                    self.__node_executable,
                    self.__node_name,
                    self.__node_namespace,
                ))
            )
            raise
        self.__final_node_name = ''
        if self.__expanded_node_namespace not in ['', '/']:
            self.__final_node_name += self.__expanded_node_namespace
        self.__final_node_name += '/' + self.__expanded_node_name
        # expand parameters too
        if self.__parameters is not None:
            self.__expanded_parameter_files = []
            for params in self.__parameters:
                if isinstance(params, dict):
                    param_file_path = self._create_params_file_from_dict(context, params)
                else:
                    if isinstance(params, pathlib.Path):
                        param_file_path = str(params)
                    else:
                        param_file_path = perform_substitutions(
                            context, normalize_to_list_of_substitutions(params))
                if not os.path.isfile(param_file_path):
                    _logger.warn(
                        'Parameter file path is not a file: {}'.format(param_file_path))
                    # Don't skip adding the file to the parameter list since space has been
                    # reserved for it in the ros_specific_arguments.
                self.__expanded_parameter_files.append(param_file_path)
        # expand remappings too
        if self.__remappings is not None:
            self.__expanded_remappings = []
            for k, v in self.__remappings:
                key = perform_substitutions(context, normalize_to_list_of_substitutions(k))
                value = perform_substitutions(context, normalize_to_list_of_substitutions(v))
                self.__expanded_remappings.append((key, value))

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """
        Execute the action.

        Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
        """
        self._perform_substitutions(context)
        # Prepare the ros_specific_arguments list and add it to the context so that the
        # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
        ros_specific_arguments = []  # type: List[Text]
        if self.__node_name is not None:
            ros_specific_arguments.append('__node:={}'.format(self.__expanded_node_name))
        if self.__node_namespace is not None:
            ros_specific_arguments.append('__ns:={}'.format(self.__expanded_node_namespace))
        if self.__expanded_parameter_files is not None:
            for param_file_path in self.__expanded_parameter_files:
                ros_specific_arguments.append('__params:={}'.format(param_file_path))
        if self.__expanded_remappings is not None:
            for remapping_from, remapping_to in self.__expanded_remappings:
                ros_specific_arguments.append('{}:={}'.format(remapping_from, remapping_to))
        context.extend_locals({'ros_specific_arguments': ros_specific_arguments})
        return super().execute(context)
