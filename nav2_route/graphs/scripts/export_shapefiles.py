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
    raise Exception('Incorrect arguements provide')

now = datetime.now()

graph = pd.concat([nodes, edges])

file_name = file_prefix + '_' + now.strftime('%m_%d_%Y_%H_%M_%S') + '.geojson'

graph.to_file(file_name, driver='GeoJSON')
