
#include <stdio.h>
#include <m3rt/base/component.h>
#include <m3xyz/components/example.h>


///////////////////////////////////////////////////////
extern "C" 
{
///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 
#define M3XYZ_EXAMPLE_NAME "m3xyz_example"
///////////////////////////////////////////////////////
//Creators
m3rt::M3Component * create_m3xyz_example(){return new m3xyz::M3XYZExample;}

//Deletors
void destroy_m3xyz_example(m3rt::M3Component* c) {delete c;}

///////////////////////////////////////////////////////
class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{
		m3rt::creator_factory[M3XYZ_EXAMPLE_NAME] =	create_m3xyz_example;
		m3rt::destroyer_factory[M3XYZ_EXAMPLE_NAME] =  destroy_m3xyz_example;
	}
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
