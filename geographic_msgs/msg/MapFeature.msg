# Geographic map feature.
#
# A list of WayPoint IDs for features like streets, highways, hiking
# trails, the outlines of buildings and parking lots in sequential
# order.
#
# Feature lists may also contain other feature lists as members.

unique_identifier_msgs/UUID   id         # Unique feature identifier
unique_identifier_msgs/UUID[] components # Sequence of feature components
KeyValue[] props                # Key/value properties for this feature
