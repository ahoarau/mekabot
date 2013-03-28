FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/m3ctrl_msgs/msg"
  "src/m3ctrl_msgs/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/m3ctrl_msgs/srv/__init__.py"
  "src/m3ctrl_msgs/srv/_M3JointArrayParam.py"
  "src/m3ctrl_msgs/srv/_M3LoadX6Status.py"
  "src/m3ctrl_msgs/srv/_M3HumanoidStatus.py"
  "src/m3ctrl_msgs/srv/_M3JointArrayStatus.py"
  "src/m3ctrl_msgs/srv/_M3JointArrayCmd.py"
  "src/m3ctrl_msgs/srv/_M3LoadX6Cmd.py"
  "src/m3ctrl_msgs/srv/_M3HumanoidParam.py"
  "src/m3ctrl_msgs/srv/_M3LoadX6Param.py"
  "src/m3ctrl_msgs/srv/_M3HumanoidCmd.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
