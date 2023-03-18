import geopandas as gpd
import pandas as pd
import sys
from datetime import datetime

try: 
    file_prefix  = sys.argv[1] 
    edges = gpd.read_file(sys.argv[2])
    nodes = gpd.read_file(sys.argv[3])
except: 
    raise Exception("Incorrect arguements provided")

now = datetime.now()

graph = pd.concat([nodes, edges])

file_name = file_prefix + "_" + now.strftime("%m_%d_%Y_%H_%M_%S") + ".geojson"

graph.to_file(file_name, driver='GeoJSON')