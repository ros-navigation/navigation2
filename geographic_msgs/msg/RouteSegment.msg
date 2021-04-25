# Route network segment.
#
# This is one directed edge of a RouteNetwork graph. It represents a
# known path from one way point to another.  If the path is two-way,
# there will be another RouteSegment with "start" and "end" reversed.

unique_identifier_msgs/UUID id           # Unique identifier for this segment

unique_identifier_msgs/UUID start        # beginning way point of segment
unique_identifier_msgs/UUID end          # ending way point of segment

KeyValue[] props                # segment properties
