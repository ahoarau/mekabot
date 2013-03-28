#include "ros/ros.h"
#include "m3_client/M3LoadX6Status.h"
#include <stdint.h>
#include <cstdlib>

using namespace ros;
using namespace m3_client;
using namespace std;

class M3LoadX6Client{
	public:
		M3LoadX6Client(string loadx6_name, NodeHandle * node_handle)
		{		  
		  status_client = node_handle->serviceClient<M3LoadX6Status>(loadx6_name+"_status");
		  
		  if(!status_client.call(status))  
		  {
		    ROS_ERROR(("Failed to connect to "+loadx6_name+"_status service.").c_str());      
		  } 
		  
		}
		
		int64_t GetTimeStamp(){return status.response.base.timestamp;} // degs
		double GetWrench(int i){return status.response.wrench[i];}		
		double GetAdcExt0(){return status.response.adc_ext_0;}
		double GetAdcExt1(){return status.response.adc_ext_1;}
		double GetAdcExt2(){return status.response.adc_ext_2;}
		double GetDigExt0(){return status.response.adc_ext_0;}
		double GetFx_mNm(){return status.response.wrench[0];}
		double GetFy_mNm(){return status.response.wrench[1];}
		double GetFz_mNm(){return status.response.wrench[2];}
		double GetTx_mNm(){return status.response.wrench[3];}
		double GetTy_mNm(){return status.response.wrench[4];}
		double GetTz_mNm(){return status.response.wrench[5];}
		void UpdateStatus(){status_client.call(status);}
		void SendCommand(){}
	private:		
		M3LoadX6Status status;		
		ServiceClient status_client;
		
};
		