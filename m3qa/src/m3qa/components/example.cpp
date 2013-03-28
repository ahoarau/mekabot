#include "m3xyz/components/example.h"
#include "m3rt/base/m3rt_def.h"
#include "m3rt/base/component_factory.h"

namespace m3xyz{
	
using namespace m3rt;
using namespace std;
	
		
///////////////////////////////////////////////////////


void M3XYZExample::Startup()
{
	if (foo==NULL)
		SetStateError();
	else
		SetStateSafeOp();
}

void M3XYZExample::Shutdown()
{
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
						  
bool M3XYZExample::ReadConfig(const char * filename)
{
	YAML::Node doc;
	GetYamlDoc(filename, doc);
	if (!M3Component::ReadConfig(filename))
		return false;
	doc["foo_name"] >> foo_name;
	double val;
	doc["param"]["gain"] >> val;
	param.set_gain(val);
	return true;
}

bool M3XYZExample::LinkDependentComponents()
{
	foo=(M3Foo*) factory->GetComponent(foo_name);
	if (foo==NULL)
	{
		M3_INFO("M3Foo component %s not found for component %s\n",foo_name.c_str(),GetName().c_str());
		return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void M3XYZExample::StepStatus()
{
	status.set_sensor(foo->GetSensorVal()*param.gain()); 
}

void M3XYZExample::StepCommand()
{
	if (command.enable())
	{
		foo->EnableControler();
	}

}

}