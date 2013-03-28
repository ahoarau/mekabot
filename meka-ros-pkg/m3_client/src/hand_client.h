#include "joint_array_client.h"

using namespace ros;
using namespace m3_client;

class M3HandClient : public M3JointArrayClient{
	public:
		M3HandClient(string hand_name, NodeHandle * node_handle):M3JointArrayClient(hand_name, node_handle){}
		
};