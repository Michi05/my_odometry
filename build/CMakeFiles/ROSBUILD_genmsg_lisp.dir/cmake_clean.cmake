FILE(REMOVE_RECURSE
  "../src/my_odometry/msg"
  "../src/my_odometry/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/odom_answer.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_odom_answer.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
