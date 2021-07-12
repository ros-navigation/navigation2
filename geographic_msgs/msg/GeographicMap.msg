# Geographic map for a specified region.

std_msgs/Header header            # stamp specifies time
                         # frame_id (normally /map)

unique_identifier_msgs/UUID id    # identifier for this map
BoundingBox  bounds      # 2D bounding box containing map

WayPoint[]   points      # way-points
MapFeature[] features    # map features
KeyValue[]   props       # map properties
