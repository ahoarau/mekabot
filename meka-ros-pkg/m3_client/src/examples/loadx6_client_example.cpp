#include "../loadx6_client.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loadx6_client_example");  
  ros::NodeHandle node_handle;
  M3LoadX6Client loadx6 = M3LoadX6Client("m3loadx6_ma10_l0", &node_handle);
 
  // Print Loadx6 values at 10 Hz for 5 seconds
  
 ros::Rate r(10); // 10 hz
  loadx6.UpdateStatus();
  
  for (int i = 0; i < 50; i++)
  {
    ROS_INFO("LoadX6 Values:");
    ROS_INFO("Fx: %f", loadx6.GetFx_mNm());
    ROS_INFO("Fy: %f", loadx6.GetFy_mNm());
    ROS_INFO("Fz: %f", loadx6.GetFz_mNm());
    ROS_INFO("Tx: %f", loadx6.GetTx_mNm());
    ROS_INFO("Ty: %f", loadx6.GetTy_mNm());
    ROS_INFO("Tz: %f", loadx6.GetTz_mNm());
  }
  
 return 0;
}