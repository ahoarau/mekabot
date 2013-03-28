#ifndef DEF_WARNING_H	
#define DEF_WARNING_H	

#warning "You should try the new 4Q table developped for LCM"
#warning "Sorry, had no time to integrate/test it in this project. -JF"

//Compilation warnings:
#ifdef BMW_0_5_A2R4
	#warning "Project: BMW_0_5_A2R4"
#elif BMW_0_6_A2R4
	#warning "Project: BMW_0_6_A2R4"
#elif BMW_1_0_A2R4
        #warning "Project: BWM_1_0_A2R4"
#else
	#warning "How do you expect it to work if you don't specify what robot?"
#endif

#endif
