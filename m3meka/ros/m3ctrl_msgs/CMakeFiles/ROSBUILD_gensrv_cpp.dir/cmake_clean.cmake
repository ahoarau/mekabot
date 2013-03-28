FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/m3ctrl_msgs/msg"
  "src/m3ctrl_msgs/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/m3ctrl_msgs/M3JointArrayParam.h"
  "srv_gen/cpp/include/m3ctrl_msgs/M3LoadX6Status.h"
  "srv_gen/cpp/include/m3ctrl_msgs/M3HumanoidStatus.h"
  "srv_gen/cpp/include/m3ctrl_msgs/M3JointArrayStatus.h"
  "srv_gen/cpp/include/m3ctrl_msgs/M3JointArrayCmd.h"
  "srv_gen/cpp/include/m3ctrl_msgs/M3LoadX6Cmd.h"
  "srv_gen/cpp/include/m3ctrl_msgs/M3HumanoidParam.h"
  "srv_gen/cpp/include/m3ctrl_msgs/M3LoadX6Param.h"
  "srv_gen/cpp/include/m3ctrl_msgs/M3HumanoidCmd.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
