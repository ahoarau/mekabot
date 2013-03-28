#ifndef INC_BDF_H
#define INC_BDF_H

// bdf.h - Board Definition File
// =============================
// You need a unique bdf.h for each project!
// No more definitions in the compiler options, everything is now here
// 

//Hardware version:
#define USE_MAX2_0_3

//Functionnalities:
#define EMBEDDED 
#define USE_DIO 
#define USE_ADC 
#define USE_PWM 
#define USE_TIMESTAMP_DC 
#define USE_ETHERCAT 
#define M3_MAX2_BDC_A2R4 
#define USE_CURRENT 
#define USE_MAX2_0_3 
#define USE_CONTROL 
#define USE_ENCODER_VERTX 
#define USE_WATCHDOG

//Compilation warnings:
#warning "Project: MAX2_BDC_0_3_A2R2"
#ifdef USE_MAX2_0_2
#warning "Compiled for MAX2 v0.2."
#endif
#ifdef USE_MAX2_0_3
#warning "Compiled for MAX2 v0.3."
#endif

#endif
