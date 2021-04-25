# Geographic map route network.
#
# A directed graph of WayPoint nodes and RouteSegment edges.  This
# information is extracted from the more-detailed contents of a
# GeographicMap.  A RouteNetwork contains only the way points and
# route segments of interest for path planning.

std_msgs/Header          header

unique_identifier_msgs/UUID id    # This route network identifier
BoundingBox     bounds   # 2D bounding box for network

WayPoint[]      points   # Way points in this network
RouteSegment[]  segments # Directed edges of this network

KeyValue[]      props    # Network key/value properties
