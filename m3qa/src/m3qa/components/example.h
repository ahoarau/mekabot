#ifndef M3_XYZ_EXAMPLE_H
#define M3_XYZ_EXAMPLE_H

#include "m3rt/base/component.h"
#include <google/protobuf/message.h>
#include "m3xyz/components/example.pb.h"

namespace m3xyz
{
	using namespace std;
	using namespace m3;

class M3XYZExample : public m3rt::M3Component
{
	public:
		M3XYZExample(): m3rt::M3Component(CALIB_PRIORITY),foo(NULL){}
		google::protobuf::Message * GetCommand(){return &command;}
		google::protobuf::Message * GetStatus(){return &status;}
		google::protobuf::Message * GetParam(){return &param;}
	protected:
		bool ReadConfig(const char * filename);
		void Startup();
		void Shutdown();
                bool LinkDependentComponents();
		void StepStatus();
		void StepCommand();
		M3XYZExampleStatus status;
		M3XYZExampleCommand command;
		M3XYZExampleParam param;
		M3BaseStatus * GetBaseStatus(){return status.mutable_base();}
	private:
		string foo_name;
		M3Foo * foo;

};
}
#endif