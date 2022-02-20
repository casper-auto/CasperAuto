# Header
Header header

# Lifetime ID of the object
uint32 lifetime_id
# Local ID showing its relative position; Use 0 as a fake vehicle.
uint32 local_id

float32 width

float32 length

float64 s

float64 d

# Pose (Position + Orientation)
#   must be specified in the coordinate frame given by header.frame_id
#   For further details: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
geometry_msgs/PoseWithCovariance pose

# Velocity
#   must be specified in the coordinate frame given by child_frame_id
#   For further details: http://docs.ros.org/api/geometry_msgs/html/msg/TwistWithCovariance.html
geometry_msgs/TwistWithCovariance twist

# Acceleration
#   must be specified in the coordinate frame given by child_frame_id
#   For further details: http://docs.ros.org/api/geometry_msgs/html/msg/AccelWithCovariance.html
geometry_msgs/AccelWithCovariance accel
