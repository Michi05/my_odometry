FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/my_odometry/msg"
  "../src/my_odometry/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/emptyRequest.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_emptyRequest.lisp"
  "../srv_gen/lisp/statusMsg.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_statusMsg.lisp"
  "../srv_gen/lisp/odom_update_srv.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_odom_update_srv.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
