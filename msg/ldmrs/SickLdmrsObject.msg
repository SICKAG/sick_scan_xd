int32 id

# ROS-1:
time tracking_time                      # since when the object is tracked
time last_seen

# ROS-2:
#builtin_interfaces/Time tracking_time  # since when the object is tracked
#builtin_interfaces/Time last_seen

geometry_msgs/TwistWithCovariance velocity

geometry_msgs/Pose bounding_box_center
geometry_msgs/Vector3 bounding_box_size

geometry_msgs/PoseWithCovariance object_box_center
geometry_msgs/Vector3 object_box_size

geometry_msgs/Point[] contour_points
