FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/my_odometry/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/emptyRequest.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_emptyRequest.lisp"
  "../srv_gen/lisp/odometryAnswer.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_odometryAnswer.lisp"
  "../srv_gen/lisp/statusMsg.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_statusMsg.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
