/*! \mainpage notitle

\htmlinclude "MainPageSpi.htm"

*/
/** 
\defgroup ecatappl ecatappl.c: EtherCAT Slave - application interface
\brief This file contains the EtherCAT State Machine and Process Data interface.\n
\brief Changes to version V3.11:\n
\brief The function DC_SyncCycle and most parts of the main-program are implemented in\n
\brief COE_Application now, which will be called from the ESC-ISR (in spihw.c/mcihw.c) in\n
\brief synchronous mode (bSynchronMode=1) and from main in free run mode (bSynchronMode=0)\n
\brief Changes to version V3.01:\n
\brief DC_SyncCycle: the example for the equalization of the interrupt jitter for\n
\brief the PIC was changed\n
\author  Holger Buettner
\version 3.20
\date    26.10.2006
*/

//---------------------------------------------------------------------------------------
/**
\ingroup ecatappl
\file ecatappl.c
\brief Implementation.
*/
//---------------------------------------------------------------------------------------

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/
#ifdef USE_ETHERCAT
#define	_ECATAPPL_ 1
#include "ethercat_appl.h"
#undef _ECATAPPL_
#define	_ECATAPPL_ 0


/*--------------------------------------------------------------------------------------
------	
------	local Types and Defines
------	
--------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------  
------	local variables and constants
------	
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------  
------	local functions
------	
-----------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------
------  
------	Functions
------	
-----------------------------------------------------------------------------------------*/

/**
\addtogroup ecatappl
@{
*/


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief	The function is called in the state transition from INIT to PREOP when
 			all general settings were checked to start the mailbox handler. This function
 			informs the application about the state transition, the application can refuse 
 			the state transition when returning an AL Status error code.
			The return code NOERROR_INWORK can be used, if the application cannot confirm
			the state transition immediately, in that case this function will be called cyclically
			until a value unequal NOERROR_INWORK is returned

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 APPL_StartMailboxHandler(void)
{
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	 0, NOERROR_INWORK

 \brief	The function is called in the state transition from PREEOP to INIT
 			to stop the mailbox handler. This functions informs the application 
 			about the state transition, the application cannot refuse 
 			the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 APPL_StopMailboxHandler(void)
{
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief	The function is called in case of a state transition from PREOP to SAFEOP before
 			the general settings were checked and shall generate the input and output mapping:
 			the variable nPdInputData and nPdOutputdata shall be adapted to the PDO mapping and Sync Manager PDO assign, 
 			this variable will be compared with the configured Sync Manager channel 2 and 3 size

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 APPL_GenerateMapping(void)
{

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief	The function is called in the state transition from PREOP to SAFEOP when
 			all general settings were checked to start the input handler. This function
 			informs the application about the state transition, the application can refuse 
 			the state transition when returning an AL Status error code.
 			When returning ALSTATUSCODE_NOERROR, the inputs has to be updated once before return.
			The return code NOERROR_INWORK can be used, if the application cannot confirm
			the state transition immediately, in that case this function will be called cyclically
			until a value unequal NOERROR_INWORK is returned
			The variable nEscAddrInputData includes the offset of the input data inside 
			the EtherCAT Slave Controller physical memory

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 APPL_StartInputHandler(void)
{
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	 0, NOERROR_INWORK

 \brief	The function is called in the state transition from SAFEOP to PREEOP
 			to stop the input handler. This functions informs the application 
 			about the state transition, the application cannot refuse 
 			the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 APPL_StopInputHandler(void)
{
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief	The function is called in the state transition from SAFEOP to OP when
 			all general settings were checked to start the output handler. This function
 			informs the application about the state transition, the application can refuse 
 			the state transition when returning an AL Status error code.
			The return code NOERROR_INWORK can be used, if the application cannot confirm
			the state transition immediately, in that case this function will be called cyclically
			until a value unequal NOERROR_INWORK is returned
			The variable nEscAddrOutputData includes the offset of the output data inside 
			the EtherCAT Slave Controller physical memory

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 APPL_StartOutputHandler(void)
{
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	 0, NOERROR_INWORK

 \brief	The function is called in the state transition from OP to SAFEOP
 			to stop the output handler. This functions informs the application 
 			about the state transition, the application cannot refuse 
 			the state transition.

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 APPL_StopOutputHandler(void)
{
	return 0;
}

#endif
/** @} */
