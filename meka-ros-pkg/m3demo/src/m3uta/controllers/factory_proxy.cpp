
#include <stdio.h>
#include <m3rt/base/component.h>
#include <m3uta/controllers/example.h>


///////////////////////////////////////////////////////
extern "C" 
{
///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 
#define M3_EXAMPLE_NAME "m3example"
///////////////////////////////////////////////////////
//Creators
m3rt::M3Component * create_m3example(){return new m3uta::M3Example;}

//Deletors
void destroy_m3example(m3rt::M3Component* c) {delete c;}

///////////////////////////////////////////////////////
class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{
		m3rt::creator_factory[M3_EXAMPLE_NAME] = create_m3example;
		m3rt::destroyer_factory[M3_EXAMPLE_NAME] =  destroy_m3example;
	}
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
