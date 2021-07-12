# A list of geographic map changes.

std_msgs/Header header                   # stamp specifies time of change
                                # frame_id (normally /map)

GeographicMap diffs             # new and changed points and features
unique_identifier_msgs/UUID[] deletes    # deleted map components
