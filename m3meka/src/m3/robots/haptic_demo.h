
#ifndef M3_HAPTIC_DEMO_H
#define M3_HAPTIC_DEMO_H

#include "m3rt/base/component.h"
#include <google/protobuf/message.h>
#include "m3/robots/haptic_demo.pb.h"
#include "m3/chains/arm.h"
#include "m3/robots/humanoid.h"

namespace m3
{
	using namespace std;	
	using namespace KDL;
//Example component class. Sums the current consumption of both arms and scales it by param.scalar
class M3HapticDemo : public m3rt::M3Component
{
	public:
		M3HapticDemo(): m3rt::M3Component(ROBOT_CTRL_PRIORITY),bot(NULL){RegisterVersion("default",DEFAULT);}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	public:
	protected:
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
	protected:
		M3HapticDemoStatus status;
		M3HapticDemoCommand command;
		M3HapticDemoParam param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
	private:
		enum {DEFAULT};
		string bot_name;
		M3Humanoid * bot;
		int tmp_cnt;
		M3PID pid_x;		
		M3PID pid_z;
		M3DFilter x_df;
		M3DFilter z_df;
};


}

#endif


