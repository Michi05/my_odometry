FILE(REMOVE_RECURSE
  "../src/my_odometry/msg"
  "../src/my_odometry/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/my_odometry/emptyRequest.h"
  "../srv_gen/cpp/include/my_odometry/statusMsg.h"
  "../srv_gen/cpp/include/my_odometry/odom_update_srv.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
