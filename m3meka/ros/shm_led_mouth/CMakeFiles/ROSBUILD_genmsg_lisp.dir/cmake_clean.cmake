FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/shm_led_mouth/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/LEDMatrixCmd.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_LEDMatrixCmd.lisp"
  "msg_gen/lisp/LEDMatrixRow.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_LEDMatrixRow.lisp"
  "msg_gen/lisp/LEDMatrixRGB.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_LEDMatrixRGB.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
