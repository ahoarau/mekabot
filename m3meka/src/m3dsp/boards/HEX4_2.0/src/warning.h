#ifndef DEF_WARNING_H	
#define DEF_WARNING_H	

//Compilation warnings:
#ifdef HEX4_2_0_S2R1
	#warning "Project: HEX4_2_0_S2R1"
#else
	#warning "How do you expect it to work if you don't specify what robot?"
#endif

#warning "=> Untested Code, Be Careful! <= (Remove this line when tested)"

#endif
