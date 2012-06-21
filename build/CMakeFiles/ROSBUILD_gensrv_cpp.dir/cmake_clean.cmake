FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/my_odometry/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/my_odometry/emptyRequest.h"
  "../srv_gen/cpp/include/my_odometry/odometryAnswer.h"
  "../srv_gen/cpp/include/my_odometry/statusMsg.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
