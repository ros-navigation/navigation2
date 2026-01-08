#!/usr/bin/python3
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

# This tool converts a behavior tree XML file to a PNG image. Run bt2img.py -h
# for instructions

import argparse
import logging
import os
import xml.etree.ElementTree as ET

import graphviz  # pip3 install graphviz

control_nodes = [
    'Fallback',
    'Parallel',
    'ReactiveFallback',
    'ReactiveSequence',
    'Sequence',
    'SequenceWithMemory',
    'BlackboardCheckInt',
    'BlackboardCheckDouble',
    'BlackboardCheckString',
    'ForceFailure',
    'ForceSuccess',
    'Inverter',
    'Repeat',
    'Subtree',
    'Timeout',
    'RecoveryNode',
    'PipelineSequence',
    'RoundRobin',
    'Control',
]
action_nodes = [
    'AlwaysFailure',
    'AlwaysSuccess',
    'SetBlackboard',
    'ComputePathToPose',
    'FollowPath',
    'BackUp',
    'Spin',
    'Wait',
    'ClearEntireCostmap',
    'ReinitializeGlobalLocalization',
    'Action',
]
condition_nodes = [
    'IsStuck',
    'GoalReached',
    'initialPoseReceived',
    'GoalUpdated',
    'DistanceTraveled',
    'TimeExpired',
    'TransformAvailable',
    'Condition',
]
decorator_nodes = [
    'Decorator',
    'RateController',
    'DistanceController',
    'SpeedController',
]
subtree_nodes = [
    'SubTree',
]


def resolve_ros_package_path(ros_pkg: str, path: str) -> str | None:
    """
    Resolve a ROS package path to an actual filesystem path.

    For example, if you have:
    <include ros_pkg="nav2_bt_navigator" path="behavior_trees/navigate_to_pose.xml"/>

    This function returns the actual filesystem path.
    """
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
        pkg_share_dir = get_package_share_directory(ros_pkg)
        return os.path.join(pkg_share_dir, path)
    except ImportError as e:
        logging.error(f'Failed to import ament_index_python: {e}')
        return None
    except PackageNotFoundError as e:
        logging.error(f'ROS package "{ros_pkg}" not found: {e}')
        return None


def load_includes(
    xml_element: ET.Element,
    base_dir: str,
    processed_files: set[str] | None = None,
) -> ET.Element:
    """
    Recursively load and merge included XML files into the tree.

    For example:
    <include ros_pkg="nav2_bt_navigator" path="behavior_trees/navigate_to_pose.xml"/>

    This function:
    1. Finds all <include> tags
    2. Loads those XML files
    3. Copies <BehaviorTree> elements from them into our XML
    4. Removes the <include> tag
    """
    if processed_files is None:
        processed_files = set()

    # Get all <include> elements
    includes = [elem for elem in xml_element if elem.tag == 'include']

    for include in includes:
        ros_pkg = include.get('ros_pkg')
        path = include.get('path')

        # Resolve the path
        if ros_pkg and path:
            include_path = resolve_ros_package_path(ros_pkg, path)
        else:
            include_path = os.path.join(base_dir, path) if path else None

        if include_path:
            include_path = os.path.abspath(include_path)

            # Check if we already processed this file (prevent infinite loops)
            if include_path in processed_files:
                print(f'Warning: Circular include detected for {include_path}, skipping')
                if include in xml_element:
                    xml_element.remove(include)
                continue

            processed_files.add(include_path)

        # Load file if it exists
        if include_path and os.path.exists(include_path):
            try:
                included_tree = ET.parse(include_path)
                included_root = included_tree.getroot()
                # Recursively load includes in this file first
                included_base_dir = os.path.dirname(include_path)
                load_includes(included_root, included_base_dir, processed_files)
                # Copy all <BehaviorTree> elements from this file
                for behavior_tree in included_root.findall('BehaviorTree'):
                    xml_element.append(behavior_tree)
            except (ET.ParseError, OSError) as e:
                print(f'Warning: Could not load included file {include_path}: {e}')
        else:
            if path:
                if ros_pkg:
                    file_desc = f'{ros_pkg}/{path}'
                else:
                    file_desc = path
                print(f'Warning: Could not resolve included file {file_desc}')

        # Remove the <include> element
        if include in xml_element:
            xml_element.remove(include)

    return xml_element


def main() -> None:
    args = parse_command_line()
    xml_tree = ET.parse(args.behavior_tree)
    root = xml_tree.getroot()
    # Process includes before parsing the tree structure
    base_dir = os.path.dirname(os.path.abspath(args.behavior_tree))
    load_includes(root, base_dir, set())

    root_tree_name = find_root_tree_name(xml_tree)
    behavior_tree = find_behavior_tree(xml_tree, root_tree_name)
    dot = convert2dot(behavior_tree, xml_tree)
    if args.legend:
        legend = make_legend()
        legend.format = 'png'
        legend.render(args.legend)
    dot.format = 'png'
    if args.save_dot:
        print(f'Saving dot to {args.save_dot}')
        args.save_dot.write(dot.source)
    dot.render(args.image_out, view=args.display)


def parse_command_line() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Convert a behavior tree XML file to an image'
    )
    parser.add_argument(
        '--behavior_tree',
        required=True,
        help='the behavior tree XML file to convert to an image',
    )
    parser.add_argument(
        '--image_out',
        required=True,
        help='The name of the output image file. Leave off the .png extension',
    )
    parser.add_argument(
        '--display',
        action='store_true',
        help='If specified, opens the image in the default viewer',
    )
    parser.add_argument(
        '--save_dot',
        type=argparse.FileType('w'),
        help='Saves the intermediate dot source to the specified file',
    )
    parser.add_argument('--legend', help='Generate a legend image as well')
    return parser.parse_args()


def find_root_tree_name(xml_tree: ET.ElementTree) -> str:
    root = xml_tree.getroot()
    main_tree = root.get('main_tree_to_execute')
    if main_tree is None:
        raise RuntimeError('No main_tree_to_execute attribute found in XML root')
    return main_tree


def find_behavior_tree(xml_tree: ET.ElementTree, tree_name: str) -> ET.Element:
    trees = xml_tree.findall('BehaviorTree')
    if len(trees) == 0:
        raise RuntimeError('No behavior trees were found in the XML file')

    for tree in trees:
        if tree_name == tree.get('ID'):
            return tree

    raise RuntimeError(f'No behavior tree for name {tree_name} found in the XML file')


# Generate a dot description of the root of the behavior tree.
def convert2dot(behavior_tree: ET.Element, xml_tree: ET.ElementTree) -> graphviz.Digraph:
    dot = graphviz.Digraph()
    root = behavior_tree
    parent_dot_name = str(hash(root))
    dot.node(parent_dot_name, root.get('ID'), shape='box')
    convert_subtree(dot, root, parent_dot_name, xml_tree)
    return dot


# Recursive function. We add the children to the dot file, and then recursively
# call this function on the children. Nodes are given an ID that is the hash
# of the node to ensure each is unique.
def convert_subtree(
    dot: graphviz.Digraph,
    parent_node: ET.Element,
    parent_dot_name: str,
    xml_tree: ET.ElementTree,
) -> None:
    if parent_node.tag == 'SubTree':
        add_sub_tree(dot, parent_dot_name, parent_node, xml_tree)
    else:
        add_nodes(dot, parent_dot_name, parent_node, xml_tree)


def add_sub_tree(
    dot: graphviz.Digraph,
    parent_dot_name: str,
    parent_node: ET.Element,
    xml_tree: ET.ElementTree,
) -> None:
    subtree_id = parent_node.get('ID')
    if subtree_id is None:
        raise RuntimeError('SubTree node has no ID attribute')

    # Create a unique dot node for this SubTree element
    subtree_dot_name = str(hash(parent_node))
    dot.node(
        subtree_dot_name,
        f'SubTree: {subtree_id}',
        color=node_color('SubTree'),
        style='filled',
        shape='box'
    )
    dot.edge(parent_dot_name, subtree_dot_name)

    # Try to expand it if present, otherwise leave it as a leaf
    try:
        behavior_tree = find_behavior_tree(xml_tree, subtree_id)
    except RuntimeError:
        # Subtree definition not found in the loaded XML; it may be missing
        # entirely or expected from an <include> that was not loaded or does
        # not contain the requested BehaviorTree.
        return

    # Recurse into the referenced tree
    convert_subtree(dot, behavior_tree, subtree_dot_name, xml_tree)


def add_nodes(
    dot: graphviz.Digraph,
    parent_dot_name: str,
    parent_node: ET.Element,
    xml_tree: ET.ElementTree,
) -> None:
    for node in list(parent_node):
        label = make_label(node)
        dot.node(
            str(hash(node)),
            label,
            color=node_color(node.tag),
            style='filled',
            shape='box',
        )
        dot_name = str(hash(node))
        dot.edge(parent_dot_name, dot_name)
        convert_subtree(dot, node, dot_name, xml_tree)


# The node label contains the:
# type, the name if provided, and the parameters.
def make_label(node: ET.Element) -> str:
    label = "< <table border='0' cellspacing='0' cellpadding='0'>"
    label += f"<tr><td align='text'><i>{node.tag}</i></td></tr>"
    name = node.get('name')
    if name:
        label += f"<tr><td align='text'><b>{name}</b></td></tr>"

    for param_name, value in node.items():
        label += f"<tr><td align='left'><sub>{param_name}={value}</sub></td></tr>"
    label += '</table> >'
    return label


def node_color(node_type: str) -> str:
    if node_type in control_nodes:
        return 'chartreuse4'
    if node_type in action_nodes:
        return 'cornflowerblue'
    if node_type in condition_nodes:
        return 'yellow2'
    if node_type in decorator_nodes:
        return 'darkorange1'
    if node_type in subtree_nodes:
        return 'darkorchid1'
    # else it's unknown
    return 'grey'


# creates a legend which can be provided with the other images.
def make_legend() -> graphviz.Digraph:
    legend = graphviz.Digraph(graph_attr={'rankdir': 'LR'})
    legend.attr(label='Legend')
    legend.node('Unknown', shape='box', style='filled', color='grey')
    legend.node(
        'Action', 'Action Node', shape='box', style='filled', color='cornflowerblue'
    )
    legend.node(
        'Condition', 'Condition Node', shape='box', style='filled', color='yellow2'
    )
    legend.node(
        'Control', 'Control Node', shape='box', style='filled', color='chartreuse4'
    )

    return legend


if __name__ == '__main__':
    main()
