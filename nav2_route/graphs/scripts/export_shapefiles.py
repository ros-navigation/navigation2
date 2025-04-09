#! /usr/bin/env python3
# Copyright 2021 Josh Wallace
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

from datetime import datetime
import sys

import geopandas as gpd
import pandas as pd

try:
    file_prefix = sys.argv[1]
    edges = gpd.read_file(sys.argv[2])
    nodes = gpd.read_file(sys.argv[3])
except Exception:
    raise Exception('Incorrect arguments provide')

# Rename columns to match the expected output
nodes = nodes.rename(columns={'start_node': 'startid'})
edges = edges.rename(columns={'start_node': 'startid'})
nodes = nodes.rename(columns={'end_node': 'endid'})
edges = edges.rename(columns={'end_node': 'endid'})

# Remove edgeid and use standard id
edges = edges.rename(columns={'edge_id': 'id'})

# Make sure IDs are integers
if 'id' in nodes.columns:
    nodes['id'] = nodes['id'].astype(int)
if 'id' in edges.columns:
    edges['id'] = edges['id'].astype(int)

now = datetime.now()

graph = pd.concat([nodes, edges])

# Set start/endids to integers, for nodes, set to -1 (not used)
if 'startid' in graph.columns:
    graph['startid'] = graph['startid'].fillna(-1).astype(int)
if 'endid' in graph.columns:
    graph['endid'] = graph['endid'].fillna(-1).astype(int)
if 'startid' in graph.columns:
    graph['startid'] = graph['startid'].astype(int)
if 'endid' in graph.columns:
    graph['endid'] = graph['endid'].astype(int)

file_name = file_prefix + '_' + now.strftime('%m_%d_%Y_%H_%M_%S') + '.geojson'

graph.to_file(file_name, driver='GeoJSON')
