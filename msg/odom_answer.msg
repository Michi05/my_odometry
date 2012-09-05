## Common answer message for the odometry package
##the services are suppossed to answer with this
##info everytime.

#   Global/Relative transform/odometry messages:
# Transform frame from tf2 package
tf2_msgs/TFMessage globalTF
tf2_msgs/TFMessage relativeTF
# System's odometry messages
nav_msgs/Odometry globalOdometry
nav_msgs/Odometry relativeOdometry

# Code number for the status of the whole algorithm
int64 statusCode

