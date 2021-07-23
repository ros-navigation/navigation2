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
import xml.etree.ElementTree
import graphviz  # pip3 install graphviz

control_nodes = [
    "Fallback",
    "Parallel",
    "ReactiveFallback",
    "ReactiveSequence",
    "Sequence",
    "SequenceStar",
    "BlackboardCheckInt",
    "BlackboardCheckDouble",
    "BlackboardCheckString",
    "ForceFailure",
    "ForceSuccess",
    "Inverter",
    "Repeat",
    "Subtree",
    "Timeout",
    "RecoveryNode",
    "PipelineSequence",
    "RoundRobin",
    "Control",
    ]
action_nodes = [
    "AlwaysFailure",
    "AlwaysSuccess",
    "SetBlackboard",
    "ComputePathToPose",
    "FollowPath",
    "BackUp",
    "Spin",
    "Wait",
    "ClearEntireCostmap",
    "ReinitializeGlobalLocalization",
    "Action",
    ]
condition_nodes = [
    "IsStuck",
    "GoalReached",
    "initialPoseReceived",
    "GoalUpdated",
    "DistanceTraveled",
    "TimeExpired",
    "TransformAvailable",
    "Condition",
    ]
decorator_nodes = [
    "Decorator",
    "RateController",
    "DistanceController",
    "SpeedController",
]
subtree_nodes = [
    "SubTree",
]

global xml_tree

def main():
    global xml_tree
    args = parse_command_line()
    xml_tree = xml.etree.ElementTree.parse(args.behavior_tree)
    root_tree_name = find_root_tree_name(xml_tree)
    behavior_tree = find_behavior_tree(xml_tree, root_tree_name)
    dot = convert2dot(behavior_tree)
    if args.legend:
        legend = make_legend()
        legend.format = 'png'
        legend.render(args.legend)
    dot.format = 'png'
    if args.save_dot:
        print(f'Saving dot to {args.save_dot}')
        args.save_dot.write(dot.source)
    dot.render(args.image_out, view=args.display)

def parse_command_line():
    parser = argparse.ArgumentParser(description='Convert a behavior tree XML file to an image')
    parser.add_argument('--behavior_tree', required=True,
                        help='the behavior tree XML file to convert to an image')
    parser.add_argument('--image_out', required=True,
                        help='The name of the output image file. Leave off the .png extension')
    parser.add_argument('--display', action="store_true",
                        help='If specified, opens the image in the default viewer')
    parser.add_argument('--save_dot', type=argparse.FileType('w'),
                        help='Saves the intermediate dot source to the specified file')
    parser.add_argument('--legend',
                        help='Generate a legend image as well')
    return parser.parse_args()

def find_root_tree_name(xml_tree):
    return xml_tree.getroot().get('main_tree_to_execute')

def find_behavior_tree(xml_tree, tree_name):
    trees = xml_tree.findall('BehaviorTree')
    if len(trees) == 0:
        raise RuntimeError("No behavior trees were found in the XML file")

    for tree in trees:
        if tree_name == tree.get('ID'):
            return tree

    raise RuntimeError(f'No behavior tree for name {tree_name} found in the XML file')

# Generate a dot description of the root of the behavior tree.
def convert2dot(behavior_tree):
    dot = graphviz.Digraph()
    root = behavior_tree
    parent_dot_name = str(hash(root))
    dot.node(parent_dot_name, root.get('ID'), shape='box')
    convert_subtree(dot, root, parent_dot_name)
    return dot

# Recursive function. We add the children to the dot file, and then recursively
# call this function on the children. Nodes are given an ID that is the hash
# of the node to ensure each is unique.
def convert_subtree(dot, parent_node, parent_dot_name):
    if parent_node.tag == "SubTree":
        add_sub_tree(dot, parent_dot_name, parent_node)
    else:
        add_nodes(dot, parent_dot_name, parent_node)

def add_sub_tree(dot, parent_dot_name, parent_node):
    root_tree_name = parent_node.get('ID')
    dot.node(parent_dot_name, root_tree_name, shape='box')
    behavior_tree = find_behavior_tree(xml_tree, root_tree_name)
    convert_subtree(dot, behavior_tree, parent_dot_name)

def add_nodes(dot, parent_dot_name, parent_node):
    for node in list(parent_node):
        label = make_label(node)
        dot.node(str(hash(node)), label, color=node_color(node.tag), style='filled', shape='box')
        dot_name = str(hash(node))
        dot.edge(parent_dot_name, dot_name)
        convert_subtree(dot, node, dot_name)

# The node label contains the:
# type, the name if provided, and the parameters.
def make_label(node):
    label = '< <table border="0" cellspacing="0" cellpadding="0">'
    label += '<tr><td align="text"><i>' + node.tag + '</i></td></tr>'
    name = node.get('name')
    if name:
        label += '<tr><td align="text"><b>' + name + '</b></td></tr>'

    for (param_name, value) in node.items():
        label += '<tr><td align="left"><sub>' + param_name + '=' + value + '</sub></td></tr>'
    label += '</table> >'
    return label

def node_color(type):
    if type in control_nodes:
        return "chartreuse4"
    if type in action_nodes:
        return "cornflowerblue"
    if type in condition_nodes:
        return "yellow2"
    if type in decorator_nodes:
        return "darkorange1"
    if type in subtree_nodes:
        return "darkorchid1"
    #else it's unknown
    return "grey"

# creates a legend which can be provided with the other images.
def make_legend():
    legend = graphviz.Digraph(graph_attr={'rankdir': 'LR'})
    legend.attr(label='Legend')
    legend.node('Unknown', shape='box', style='filled', color="grey")
    legend.node('Action', 'Action Node', shape='box', style='filled', color="cornflowerblue")
    legend.node('Condition', 'Condition Node', shape='box', style='filled', color="yellow2")
    legend.node('Control', 'Control Node', shape='box', style='filled', color="chartreuse4")

    return legend


if __name__ == '__main__':
    main()
