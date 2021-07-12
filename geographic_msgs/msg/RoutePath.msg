# Path through a route network.
#
# A path is a sequence of RouteSegment edges.  This information is
# extracted from a RouteNetwork graph.  A RoutePath lists the route
# segments needed to reach some chosen goal.

std_msgs/Header header

unique_identifier_msgs/UUID   network    # Route network containing this path
unique_identifier_msgs/UUID[] segments   # Sequence of RouteSegment IDs
KeyValue[]           props      # Key/value properties
