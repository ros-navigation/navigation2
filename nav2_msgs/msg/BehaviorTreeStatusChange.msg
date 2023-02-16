builtin_interfaces/Time timestamp   # internal behavior tree event timestamp. Typically this is wall clock time
string node_name                    # name of the node
uint16 uid                          # unique ID for this node
string previous_status              # IDLE, RUNNING, SUCCESS or FAILURE
string current_status               # IDLE, RUNNING, SUCCESS or FAILURE