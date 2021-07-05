int32 id

builtin_interfaces/Time tracking_time  # ROS1: time tracking_time # since when the object is tracked
builtin_interfaces/Time last_seen      # ROS1: time last_seen

geometry_msgs/TwistWithCovariance velocity

geometry_msgs/Pose bounding_box_center
geometry_msgs/Vector3 bounding_box_size

geometry_msgs/PoseWithCovariance object_box_center
geometry_msgs/Vector3 object_box_size

geometry_msgs/Point[] contour_points
