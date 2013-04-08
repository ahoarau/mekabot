
#ifndef M3_EXAMPLE_H
#define M3_EXAMPLE_H

#include "m3rt/base/component.h"
#include <google/protobuf/message.h>
#include "m3ens/controllers/example.pb.h"
#include "m3/chains/arm.h"
#include "m3/robots/humanoid.h"

namespace m3ens
{
	using namespace std;
	using namespace m3;
	using namespace KDL;
//Example component class. Sums the current consumption of both arms and scales it by param.scalar
class M3Example : public m3rt::M3Component
{
	public:
		M3Example(): m3rt::M3Component(ROBOT_PRIORITY),bot(NULL){RegisterVersion("default",DEFAULT);}
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
		M3ExampleStatus status;
		M3ExampleCommand command;
		M3ExampleParam param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
	private:
		enum {DEFAULT};
		string bot_name;
		M3Humanoid * bot;
		int tmp_cnt;
};


}

#endif


