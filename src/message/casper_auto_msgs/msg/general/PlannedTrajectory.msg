# Trajectory estimated over time
WaypointArray trajectory_planned

# Gap we are trying to target
string gap

# Boolean to tell if the trajectory is valid
bool is_valid

# List of all cars we have to be careful
uint32[] caution_id_list

# List of all cars that are a hazard
uint32[] hazard_id_list
