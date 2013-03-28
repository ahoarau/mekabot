#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
include Makefile

# Environment
# Adding MPLAB X bin directory to path
PATH:=/opt/microchip/mplabx/mplab_ide/mplab_ide/modules/../../bin/:$(PATH)
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/m3_controller_hb2_h2r3_j2j3j4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/m3_controller_hb2_h2r3_j2j3j4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1472/adc.o ${OBJECTDIR}/_ext/1472/control.o ${OBJECTDIR}/_ext/1472/dio.o ${OBJECTDIR}/_ext/1472/ethercat.o ${OBJECTDIR}/_ext/1472/ethercat_appl.o ${OBJECTDIR}/_ext/1472/ethercat_hw.o ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/pwm.o ${OBJECTDIR}/_ext/1472/setup.o ${OBJECTDIR}/_ext/1472/timer3.o ${OBJECTDIR}/_ext/1472/encoder_ma3.o ${OBJECTDIR}/_ext/1472/encoder_vertx.o ${OBJECTDIR}/_ext/1472/spi1Drv.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1472/adc.o.d ${OBJECTDIR}/_ext/1472/control.o.d ${OBJECTDIR}/_ext/1472/dio.o.d ${OBJECTDIR}/_ext/1472/ethercat.o.d ${OBJECTDIR}/_ext/1472/ethercat_appl.o.d ${OBJECTDIR}/_ext/1472/ethercat_hw.o.d ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.d ${OBJECTDIR}/_ext/1472/main.o.d ${OBJECTDIR}/_ext/1472/pwm.o.d ${OBJECTDIR}/_ext/1472/setup.o.d ${OBJECTDIR}/_ext/1472/timer3.o.d ${OBJECTDIR}/_ext/1472/encoder_ma3.o.d ${OBJECTDIR}/_ext/1472/encoder_vertx.o.d ${OBJECTDIR}/_ext/1472/spi1Drv.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/adc.o ${OBJECTDIR}/_ext/1472/control.o ${OBJECTDIR}/_ext/1472/dio.o ${OBJECTDIR}/_ext/1472/ethercat.o ${OBJECTDIR}/_ext/1472/ethercat_appl.o ${OBJECTDIR}/_ext/1472/ethercat_hw.o ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/pwm.o ${OBJECTDIR}/_ext/1472/setup.o ${OBJECTDIR}/_ext/1472/timer3.o ${OBJECTDIR}/_ext/1472/encoder_ma3.o ${OBJECTDIR}/_ext/1472/encoder_vertx.o ${OBJECTDIR}/_ext/1472/spi1Drv.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

# Path to java used to run MPLAB X when this makefile was created
MP_JAVA_PATH="/usr/lib/jvm/java-6-openjdk/jre/bin/"
OS_CURRENT="$(shell uname -s)"
############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
MP_CC="/opt/microchip/mplabc30/v3.30c/bin/pic30-gcc"
# MP_BC is not defined
MP_AS="/opt/microchip/mplabc30/v3.30c/bin/pic30-as"
MP_LD="/opt/microchip/mplabc30/v3.30c/bin/pic30-ld"
MP_AR="/opt/microchip/mplabc30/v3.30c/bin/pic30-ar"
DEP_GEN=${MP_JAVA_PATH}java -jar "/opt/microchip/mplabx/mplab_ide/mplab_ide/modules/../../bin/extractobjectdependencies.jar" 
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps
MP_CC_DIR="/opt/microchip/mplabc30/v3.30c/bin"
# MP_BC_DIR is not defined
MP_AS_DIR="/opt/microchip/mplabc30/v3.30c/bin"
MP_LD_DIR="/opt/microchip/mplabc30/v3.30c/bin"
MP_AR_DIR="/opt/microchip/mplabc30/v3.30c/bin"
# MP_BC_DIR is not defined

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/m3_controller_hb2_h2r3_j2j3j4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33FJ64MC204
MP_LINKER_FILE_OPTION=,-Tp33FJ64MC204.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/adc.o: ../adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/adc.o.ok ${OBJECTDIR}/_ext/1472/adc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/adc.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/adc.o.d" -o ${OBJECTDIR}/_ext/1472/adc.o ../adc.c  
	
${OBJECTDIR}/_ext/1472/control.o: ../control.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/control.o.ok ${OBJECTDIR}/_ext/1472/control.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/control.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/control.o.d" -o ${OBJECTDIR}/_ext/1472/control.o ../control.c  
	
${OBJECTDIR}/_ext/1472/dio.o: ../dio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/dio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/dio.o.ok ${OBJECTDIR}/_ext/1472/dio.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/dio.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/dio.o.d" -o ${OBJECTDIR}/_ext/1472/dio.o ../dio.c  
	
${OBJECTDIR}/_ext/1472/ethercat.o: ../ethercat.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat.o.ok ${OBJECTDIR}/_ext/1472/ethercat.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ethercat.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/ethercat.o.d" -o ${OBJECTDIR}/_ext/1472/ethercat.o ../ethercat.c  
	
${OBJECTDIR}/_ext/1472/ethercat_appl.o: ../ethercat_appl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_appl.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_appl.o.ok ${OBJECTDIR}/_ext/1472/ethercat_appl.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ethercat_appl.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/ethercat_appl.o.d" -o ${OBJECTDIR}/_ext/1472/ethercat_appl.o ../ethercat_appl.c  
	
${OBJECTDIR}/_ext/1472/ethercat_hw.o: ../ethercat_hw.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_hw.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_hw.o.ok ${OBJECTDIR}/_ext/1472/ethercat_hw.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ethercat_hw.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/ethercat_hw.o.d" -o ${OBJECTDIR}/_ext/1472/ethercat_hw.o ../ethercat_hw.c  
	
${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o: ../ethercat_slave_fsm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.ok ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.d" -o ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o ../ethercat_slave_fsm.c  
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.ok ${OBJECTDIR}/_ext/1472/main.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c  
	
${OBJECTDIR}/_ext/1472/pwm.o: ../pwm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/pwm.o.ok ${OBJECTDIR}/_ext/1472/pwm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/pwm.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/pwm.o.d" -o ${OBJECTDIR}/_ext/1472/pwm.o ../pwm.c  
	
${OBJECTDIR}/_ext/1472/setup.o: ../setup.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/setup.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/setup.o.ok ${OBJECTDIR}/_ext/1472/setup.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/setup.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/setup.o.d" -o ${OBJECTDIR}/_ext/1472/setup.o ../setup.c  
	
${OBJECTDIR}/_ext/1472/timer3.o: ../timer3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/timer3.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/timer3.o.ok ${OBJECTDIR}/_ext/1472/timer3.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/timer3.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/timer3.o.d" -o ${OBJECTDIR}/_ext/1472/timer3.o ../timer3.c  
	
${OBJECTDIR}/_ext/1472/encoder_ma3.o: ../encoder_ma3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/encoder_ma3.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/encoder_ma3.o.ok ${OBJECTDIR}/_ext/1472/encoder_ma3.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/encoder_ma3.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/encoder_ma3.o.d" -o ${OBJECTDIR}/_ext/1472/encoder_ma3.o ../encoder_ma3.c  
	
${OBJECTDIR}/_ext/1472/encoder_vertx.o: ../encoder_vertx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/encoder_vertx.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/encoder_vertx.o.ok ${OBJECTDIR}/_ext/1472/encoder_vertx.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/encoder_vertx.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/encoder_vertx.o.d" -o ${OBJECTDIR}/_ext/1472/encoder_vertx.o ../encoder_vertx.c  
	
${OBJECTDIR}/_ext/1472/spi1Drv.o: ../spi1Drv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/spi1Drv.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/spi1Drv.o.ok ${OBJECTDIR}/_ext/1472/spi1Drv.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/spi1Drv.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/spi1Drv.o.d" -o ${OBJECTDIR}/_ext/1472/spi1Drv.o ../spi1Drv.c  
	
else
${OBJECTDIR}/_ext/1472/adc.o: ../adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/adc.o.ok ${OBJECTDIR}/_ext/1472/adc.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/adc.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/adc.o.d" -o ${OBJECTDIR}/_ext/1472/adc.o ../adc.c  
	
${OBJECTDIR}/_ext/1472/control.o: ../control.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/control.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/control.o.ok ${OBJECTDIR}/_ext/1472/control.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/control.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/control.o.d" -o ${OBJECTDIR}/_ext/1472/control.o ../control.c  
	
${OBJECTDIR}/_ext/1472/dio.o: ../dio.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/dio.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/dio.o.ok ${OBJECTDIR}/_ext/1472/dio.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/dio.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/dio.o.d" -o ${OBJECTDIR}/_ext/1472/dio.o ../dio.c  
	
${OBJECTDIR}/_ext/1472/ethercat.o: ../ethercat.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat.o.ok ${OBJECTDIR}/_ext/1472/ethercat.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ethercat.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/ethercat.o.d" -o ${OBJECTDIR}/_ext/1472/ethercat.o ../ethercat.c  
	
${OBJECTDIR}/_ext/1472/ethercat_appl.o: ../ethercat_appl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_appl.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_appl.o.ok ${OBJECTDIR}/_ext/1472/ethercat_appl.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ethercat_appl.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/ethercat_appl.o.d" -o ${OBJECTDIR}/_ext/1472/ethercat_appl.o ../ethercat_appl.c  
	
${OBJECTDIR}/_ext/1472/ethercat_hw.o: ../ethercat_hw.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_hw.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_hw.o.ok ${OBJECTDIR}/_ext/1472/ethercat_hw.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ethercat_hw.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/ethercat_hw.o.d" -o ${OBJECTDIR}/_ext/1472/ethercat_hw.o ../ethercat_hw.c  
	
${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o: ../ethercat_slave_fsm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.ok ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o.d" -o ${OBJECTDIR}/_ext/1472/ethercat_slave_fsm.o ../ethercat_slave_fsm.c  
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.ok ${OBJECTDIR}/_ext/1472/main.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/main.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c  
	
${OBJECTDIR}/_ext/1472/pwm.o: ../pwm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/pwm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/pwm.o.ok ${OBJECTDIR}/_ext/1472/pwm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/pwm.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/pwm.o.d" -o ${OBJECTDIR}/_ext/1472/pwm.o ../pwm.c  
	
${OBJECTDIR}/_ext/1472/setup.o: ../setup.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/setup.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/setup.o.ok ${OBJECTDIR}/_ext/1472/setup.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/setup.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/setup.o.d" -o ${OBJECTDIR}/_ext/1472/setup.o ../setup.c  
	
${OBJECTDIR}/_ext/1472/timer3.o: ../timer3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/timer3.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/timer3.o.ok ${OBJECTDIR}/_ext/1472/timer3.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/timer3.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/timer3.o.d" -o ${OBJECTDIR}/_ext/1472/timer3.o ../timer3.c  
	
${OBJECTDIR}/_ext/1472/encoder_ma3.o: ../encoder_ma3.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/encoder_ma3.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/encoder_ma3.o.ok ${OBJECTDIR}/_ext/1472/encoder_ma3.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/encoder_ma3.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/encoder_ma3.o.d" -o ${OBJECTDIR}/_ext/1472/encoder_ma3.o ../encoder_ma3.c  
	
${OBJECTDIR}/_ext/1472/encoder_vertx.o: ../encoder_vertx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/encoder_vertx.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/encoder_vertx.o.ok ${OBJECTDIR}/_ext/1472/encoder_vertx.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/encoder_vertx.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/encoder_vertx.o.d" -o ${OBJECTDIR}/_ext/1472/encoder_vertx.o ../encoder_vertx.c  
	
${OBJECTDIR}/_ext/1472/spi1Drv.o: ../spi1Drv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1472 
	@${RM} ${OBJECTDIR}/_ext/1472/spi1Drv.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/spi1Drv.o.ok ${OBJECTDIR}/_ext/1472/spi1Drv.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1472/spi1Drv.o.d" $(SILENT) -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -omf=elf -x c -c -mcpu=$(MP_PROCESSOR_OPTION) -Wall -DEMBEDDED -DUSE_DIO -DUSE_ETHERCAT -DM3_HB2_H2R3_J2J3J4 -DUSE_PWM -DUSE_ENCODER_MA3 -DUSE_TIMESTAMP_DC -DUSE_ADC -DUSE_CONTROL -DUSE_TIMER3 -I".." -O3 -MMD -MF "${OBJECTDIR}/_ext/1472/spi1Drv.o.d" -o ${OBJECTDIR}/_ext/1472/spi1Drv.o ../spi1Drv.c  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/m3_controller_hb2_h2r3_j2j3j4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -o dist/${CND_CONF}/${IMAGE_TYPE}/m3_controller_hb2_h2r3_j2j3j4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}        -Wl,--defsym=__MPLAB_BUILD=1,-L"..",-Map="${DISTDIR}/m3_controller_hb2_h2r3_j2j3j4.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__ICD2RAM=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1
else
dist/${CND_CONF}/${IMAGE_TYPE}/m3_controller_hb2_h2r3_j2j3j4.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -omf=elf -mcpu=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/m3_controller_hb2_h2r3_j2j3j4.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}        -Wl,--defsym=__MPLAB_BUILD=1,-L"..",-Map="${DISTDIR}/m3_controller_hb2_h2r3_j2j3j4.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION)
	${MP_CC_DIR}/pic30-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/m3_controller_hb2_h2r3_j2j3j4.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -omf=elf
endif


# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf:
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "/opt/microchip/mplabx/mplab_ide/mplab_ide/modules/../../bin/"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
