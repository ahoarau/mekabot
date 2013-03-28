FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/m3_client/msg"
  "src/m3_client/srv"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/M3BaseStatus.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_M3BaseStatus.lisp"
  "msg_gen/lisp/M3OmnibaseJoy.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_M3OmnibaseJoy.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
