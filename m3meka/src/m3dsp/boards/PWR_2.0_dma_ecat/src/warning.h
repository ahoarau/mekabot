#ifndef DEF_WARNING_H	
#define DEF_WARNING_H	

//Compilation warnings:
#ifdef PWR_0_5
	#warning "Project: PWR_0_5"
//#elif PWR_0_X
	//#warning "Project: PWR_0_X"
#else
	#warning "How do you expect it to work if you don't specify what robot?"
#endif

#ifdef USE_WATCHDOG
	#warning "WATCHDOG ENABLED"
#else
	#warning "WATCHDOG DISABLED"
#endif

#endif
