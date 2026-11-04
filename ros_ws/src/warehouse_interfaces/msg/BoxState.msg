# Ground-truth state of one box (published by the sim, Day 23).
string id                     # e.g. "box_03" (see warehouse_bringup/config/registry.yaml)
string shelf                  # shelf the box rests on, "" if held or not on a slot
int32 slot                    # slot index on that shelf, -1 if none
bool held                     # attached to the robot's gripper
geometry_msgs/Point position  # box centre in the map frame
