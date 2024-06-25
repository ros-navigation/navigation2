#!/usr/bin/python3
# Copyright (c) 2024 Open Navigation LLC
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

# This tool populates the README table of build status for each package

import requests

# Global information about current distributions, shouldn't need to update
OSs = {'humble': 'jammy', 'iron': 'jammy', 'jazzy': 'noble'}
Prefixs = {'humble': 'H', 'iron': 'I', 'jazzy': 'J'}

# Set your packages here
Packages = [
    'navigation2',
    'nav2_amcl',
    'nav2_behavior_tree',
    'nav2_behaviors',
    'nav2_bringup',
    'nav2_bt_navigator',
    'nav2_collision_monitor',
    'nav2_common',
    'nav2_constrained_smoother',
    'nav2_controller',
    'nav2_core',
    'nav2_costmap_2d',
    'opennav_docking',
    'nav2_dwb_controller',  # Controller plugin for DWB packages
    'nav2_graceful_controller',
    'nav2_lifecycle_manager',
    'nav2_map_server',
    'nav2_mppi_controller',
    'nav2_msgs',
    'nav2_navfn_planner',
    'nav2_planner',
    'nav2_regulated_pure_pursuit_controller',
    'nav2_rotation_shim_controller',
    'nav2_rviz_plugins',
    'nav2_simple_commander',
    'nav2_smac_planner',
    'nav2_smoother',
    'nav2_system_tests',
    'nav2_theta_star_planner',
    'nav2_util',
    'nav2_velocity_smoother',
    'nav2_voxel_grid',
    'nav2_waypoint_follower',
]

# Set which distributions you care about
Distros = ['humble', 'iron', 'jazzy']

def getSrcPath(package, prefix, OS):
    return f'https://build.ros2.org/job/{prefix}src_u{OS[0]}__{package}__ubuntu_{OS}__source/'

def getBinPath(package, prefix, OS):
    return f'https://build.ros2.org/job/{prefix}bin_u{OS[0]}64__{package}__ubuntu_{OS}_amd64__binary/'

def createPreamble(Distros):
    table = '| Package | '
    for distro in Distros:
        table += distro + ' Source | ' + distro + ' Debian | '
    table += '\n'
    table += '| :---: |'
    for distro in Distros:
        table += ' :---: | :---: |'
    return table

def main():
    header = createPreamble(Distros)

    body = ''
    for package in Packages:
        entry = f'| {package} | '
        for distro in Distros:
            # TODO check if website exists with requests
            prefix = Prefixs[distro]
            OS = OSs[distro]
            srcURL = getSrcPath(package, prefix, OS)
            binURL = getBinPath(package, prefix, OS)
            response = requests.get(srcURL)
            # Check if package isn't in a given distribution
            if response.status_code != 200:
                entry += 'N/A | N/A | '
            else:
                entry += f'[![Build Status]({srcURL}badge/icon)]({srcURL}) | '
                entry += f'[![Build Status]({binURL}badge/icon)]({binURL}) | '
        entry += '\n'
        body += entry
    
    # Special case for Opennav Docking for directory structure of Nav2
    body = body.replace('| opennav_docking |', '| nav2_docking |')
    # Special case for reducing the label length
    body = body.replace('| nav2_regulated_pure_pursuit_controller |', '| nav2_regulated_pure_pursuit |')
    
    print(header + '\n' + body)

if __name__ == '__main__':
    main()
