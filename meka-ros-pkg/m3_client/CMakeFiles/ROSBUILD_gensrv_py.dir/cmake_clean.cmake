FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/m3_client/msg"
  "src/m3_client/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/m3_client/srv/__init__.py"
  "src/m3_client/srv/_M3JointArrayParam.py"
  "src/m3_client/srv/_M3LoadX6Status.py"
  "src/m3_client/srv/_M3ComponentCmd.py"
  "src/m3_client/srv/_M3ComponentStatus.py"
  "src/m3_client/srv/_M3HumanoidStatus.py"
  "src/m3_client/srv/_M3JointArrayStatus.py"
  "src/m3_client/srv/_M3JointArrayCmd.py"
  "src/m3_client/srv/_M3LoadX6Cmd.py"
  "src/m3_client/srv/_M3ComponentParam.py"
  "src/m3_client/srv/_M3HumanoidParam.py"
  "src/m3_client/srv/_M3LoadX6Param.py"
  "src/m3_client/srv/_M3HumanoidCmd.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
