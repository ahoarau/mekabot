#ifndef DEF_WARNING_H    
#define DEF_WARNING_H 

//Compilation warnings:
#ifdef HB2_0_2_H2R3_J0J1
     #warning "Project: HB2_0_2_H2R3_J0J1"
#elif HB2_0_2_H2R3_J2J3J4
     #warning "Project: HB2_0_2_H2R3_J2J3J4"
#else
     #warning "How do you expect it to work if you don't specify what robot?"
#endif

#endif

