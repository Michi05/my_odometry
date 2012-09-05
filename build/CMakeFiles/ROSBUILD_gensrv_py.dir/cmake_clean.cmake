FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/my_odometry/msg"
  "../src/my_odometry/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/my_odometry/srv/__init__.py"
  "../src/my_odometry/srv/_emptyRequest.py"
  "../src/my_odometry/srv/_statusMsg.py"
  "../src/my_odometry/srv/_odom_update_srv.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
