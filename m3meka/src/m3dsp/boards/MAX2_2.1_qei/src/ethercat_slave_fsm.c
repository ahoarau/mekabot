/** 
\defgroup ehthercat_slave_fsm ehthercat_slave_fsm.c: EtherCAT Slave State Machine
\brief This file contains the EtherCAT State Machine\n
\brief Changes to version V3.11:\n
\brief The operation mode (free run, synchronous, distributed clocks) will only be selected\n
\brief by register settings now (functions CheckSmChannelParameters, StartInputHandler)\n
\brief The SM-Parameter display the actual synchronization mode and cycle time (functions\n
\brief StartInputHandler, ECAT_Init)\n
\brief The state change from SAFEOP to OP is now in the function StartOutputHandler\n
\brief There were little changes in the functions StopInputHandler and StopOutputHandler too\n
\brief Changes to version V3.10:\n
\brief CheckSmChannelParameters: if no ouputs were configured the checking of the SM 2-settings were wrong,\n
\brief if no inputs were configured the checking of the SM 2-settings were wrong\n
\brief SendSmFailedEmergency: the code in the emergency was wrong\n
\brief StartInputHandler: the sync manager areas will be checked for overlaps\n
\brief StopOutputHandler: if the SM 2-event was activated, the mask was reset incrrrectly\n
\brief AL_ControlInd: MBX_StartMailboxHandler may return an error code\n
\brief Changes to version V3.02:\n
\brief AL_ControlInd: AL-Status has to be updated if the the error bit was acknowledged\n 
\brief AL_ControlInd: The AL Status Code register shall not be modified if the function AL_ControlInd\n
\brief is called in case of SM change event or an AL-Control event with the same state\n
\brief Changes to version V3.01:\n
\brief the watchdog bit in the Sync Manager settings of SM 2 was not checked\n
\brief for the watchdog functionality\n 
\brief CheckSmChannelParameters: if the ESC supports more than 4 Sync Manager\n
\brief the additional channels shall be checked too\n
\brief CheckSmSettings: the wrong channel was checked (copy and paste bug)\n
\brief StartInputHandler: the Distributed Clocks were not enabled if the input\n
\brief size was ß\n
\brief AL_ControlInd: the watchdog bit of SM 2 was not checked that could be\n
\brief a problem when disabling the watchdog when it was enabled before because\n
\brief the Reg 400 is not transmitted by TwinCAT when it is 0\n
\brief Changes to version V3.00:\n
\brief for enabling the watchdog the watchdog divider (Reg 0x400),\n
\brief the watchdog cycle time (Reg 0x420) and the watchdog bit shall be\n
\brief checked\n

\author  Holger Buettner
\version 3.20
\date    26.10.2006
*/

//---------------------------------------------------------------------------------------
/**
\ingroup ehthercat_slave_fsm	
\file ehthercat_slave_fsm.c
\brief Implementation.
*/
//---------------------------------------------------------------------------------------

/*-----------------------------------------------------------------------------------------
------	
------	Includes
------ 
-----------------------------------------------------------------------------------------*/
#ifdef USE_ETHERCAT

#define	_ECATSLV_	1
#include "ethercat_slave_fsm.h"
#undef	_ECATSLV_
#define	_ECATSLV_	0

#include "ethercat_appl.h"
#include "setup.h"
#include "dio.h"
#if SMPAR_SUPPORTED
#include "objdef.h"
#endif


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
------	functions
------	
-----------------------------------------------------------------------------------------*/

/**
\addtogroup ecatslv
@{
*/


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param  channel		SM channel
 \param  pSettings	SM settings
 \param	size			size of the SM channel
 \return 				0: okay, 1..4: faulty SM channel word
 \brief	This function is called to check the SM channel

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 CheckSmChannelParameters(UINT8 channel, TSYNCMAN ESCMEM *pSyncMan)
{
	switch (channel)
	{
#if MAX_RX_PDOS
	case PROCESS_DATA_OUT:
		/* Sync-Manager 0 is for Outputs (Master writes) */
		/* HBu 21.03.06: if no outputs are configured, the SM 0 should be disabled andd the other settings should not be checked */
		// HBu 04.05.06: for microcontrollers with BYTE_NOT_SUPPORTED the next byte should be masked
		if ( (pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & ((~SM_TOGGLEMASTER) & 0xFF)) == 0 && 
			pSyncMan->Length == 0 && nPdOutputSize == 0 )
			return 0;
		else if ( (pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & ((~SM_TOGGLEMASTER) & 0xFF)) == SM_ECATENABLE && pSyncMan->Length == nPdOutputSize )
		{
			/* sizes matches */
			if ( (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & SM_PDINITMASK) == SM_WRITESETTINGS )
			{
				/* settings matches */
				if ( ( ( nAlStatus == STATE_PREOP )&&( pSyncMan->PhysicalStartAddress >= MIN_PD_WRITE_ADDRESS )&&( pSyncMan->PhysicalStartAddress <= MAX_PD_WRITE_ADDRESS ) )
				   ||( ( nAlStatus != STATE_PREOP )&&( pSyncMan->PhysicalStartAddress == nEscAddrOutputData ) )
					) 
				{
					/* address matches */
					/* HBu 4.12.05: check, if watchdog is enabled */
					if (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & WATCHDOG_TRIGGER)
						bWdActive = 1;
					else
						bWdActive = 0;
#if ECAT_PROCESS_OUTPUT_INT
					/* if the process input data size is zero, the slave always runs in 
					   synchronous mode */
					if (nPdInputSize == 0)
					{
						bSynchronMode = TRUE;
					}
#endif

					return 0;
				}
				else
					return SYNCMANCHADDRESS+1;
			}
			else
				return SYNCMANCHSETTINGS+1;
		}
		else
			return SYNCMANCHSIZE+1;

#endif
#if MAX_TX_PDOS
	case PROCESS_DATA_IN:	
		/* Sync-Manager 1 is for Inputs (Master reads) */
		/* HBu 21.03.06: if no inputs are configured, the SM 1 should be disabled and the other settings should not be checked */
		// HBu 04.05.06: for microcontrollers with BYTE_NOT_SUPPORTED the next byte should be masked
		if ( (pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & ((~SM_TOGGLEMASTER) & 0xFF)) == 0 && pSyncMan->Length == 0 && nPdInputSize == 0 )
			return 0;
		else if ( (pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & ((~SM_TOGGLEMASTER) & 0xFF)) == SM_ECATENABLE && pSyncMan->Length == nPdInputSize )
		{
			/* sizes matches */
			if ( (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & SM_PDINITMASK) == SM_READSETTINGS )
			{
				/* settings matches */
				if ( ( ( nAlStatus == STATE_PREOP )&&( pSyncMan->PhysicalStartAddress >= MIN_PD_READ_ADDRESS )&&( pSyncMan->PhysicalStartAddress <= MAX_PD_READ_ADDRESS ) )
				   ||( ( nAlStatus != STATE_PREOP )&&( pSyncMan->PhysicalStartAddress == nEscAddrInputData ) )
					) 
				{
					/* address matches */
#if ECAT_PROCESS_INPUT_INT	|| ECAT_PROCESS_OUTPUT_INT	
					/* if the process input data size is non-zero, the slave is running in
					   synchronous mode if the PDI-event is enabled */
					if (nPdInputSize != 0)
					{
						if ( (pSyncMan->Settings.Byte[ESC_OFFS_SMCTRL] & (SM_PDINITMASK | SM_PDIEVENT)) == (SM_READSETTINGS | SM_PDIEVENT) )
							bSynchronMode = TRUE;
						else
							bSynchronMode = FALSE;
					}
#endif
					return 0;
				}
				else
					return SYNCMANCHADDRESS+1;
			}
			else
				return SYNCMANCHSETTINGS+1;
		}
		else
			return SYNCMANCHSIZE+1;
#endif

	default:
		/* V3.02 ECAT_CHANGE_BEGIN */
		/* HBu 24.01.06: has to be checked if the ESC supports more than 4 Sync Manager */
		/* ECAT_CHANGE_BEGIN V3.11 */
		// HBu 04.05.06: for microcontrollers with BYTE_NOT_SUPPORTED the next byte should be masked
		if ((pSyncMan->Settings.Byte[ESC_OFFS_ECATENABLE] & ((~SM_TOGGLEMASTER) & 0xFF)) != 0 || pSyncMan->Length != 0)
			return SYNCMANCHSIZE+1;
		/* ECAT_CHANGE_END V3.11 */
		/* V3.02 ECAT_CHANGE_END */
			
	}

	/* V3.02 ECAT_CHANGE_BEGIN */
	return 0;
	/* V3.02 ECAT_CHANGE_END */
}



/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param  lastChannel	last SM channel which should be checked
 \return 				0: okay, 1 failed

 \brief	This function checks all SM channels

*////////////////////////////////////////////////////////////////////////////////////////

UINT8	CheckSmSettings(UINT8 maxChannel)
{
#if SM_CHANGE_SUPPORTED
	static UINT8 u8Dummy;
#endif
	UINT8 i;
	UINT8 result;
	UINT8 ret = 0;

	for (i = 0; i < maxChannel; i++)
	{
		/* check SM settings for inputs before outputs, because the first SM with an error
		   generates the emergency message and the al status code */
		UINT8 j = i;
		if (i == PROCESS_DATA_OUT)
			j = PROCESS_DATA_IN;
		/* V3.02 ECAT_CHANGE_BEGIN */
		/* HBu 24.01.06: wrong channel was checked */
		else if (i == PROCESS_DATA_IN)
		/* V3.02 ECAT_CHANGE_END */
			j = PROCESS_DATA_OUT;

		result = CheckSmChannelParameters(j, HW_GetSyncMan(j));

		if ( result && ret == 0 )
		{
			if (1) //j > MAILBOX_READ), No mailboxes for this version...
			{
#if COE_SUPPORTED || SOE_SUPPORTED
				SendSmFailedEmergency(j, result);
#endif
				ret = (ALSTATUSCODE_INVALIDSMOUTCFG-PROCESS_DATA_OUT)+j;
			}
			else
				ret = ALSTATUSCODE_INVALIDMBXCFGINPREOP;
		}
	}

#if SM_CHANGE_SUPPORTED
/* ECAT_CHANGE_BEGIN V3.20 */
	/* the Enable-Byte of the rest of the SM channels has to be read to acknowledge the SM-Change-Interrupt */
	for (i = maxChannel; i < nMaxSyncMan; i++)
	{
		HW_EscReadAccess( &u8Dummy, ESC_ADDR_SYNCMAN+sizeof(TSYNCMAN)*(i-1)+ESC_OFFS_SMSETTINGS+ESC_OFFS_ECATENABLE, 1 );
	}
/* ECAT_CHANGE_END V3.20 */
#endif

	return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief	This function is called in case of the state transition from PREOP to SAFEOP

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 StartInputHandler(void)
{
	UINT8 result;
	TSYNCMAN ESCMEM * pSyncMan;
/* ECAT_CHANGE_BEGIN V3.20 */
#if SMPAR_SUPPORTED
	UINT16 syncType;
#endif
/* ECAT_CHANGE_END V3.20 */
#if DC_SUPPORTED
#endif
	/* V3.02 ECAT_CHANGE_BEGIN */
	UINT16 intMask = 0;
	/* V3.02 ECAT_CHANGE_END */

#if MAX_RX_PDOS	
	pSyncMan = HW_GetSyncMan(PROCESS_DATA_OUT);
	/* store the address of the output sync manager channel */
	nEscAddrOutputData = pSyncMan->PhysicalStartAddress;
#endif
#if MAX_TX_PDOS	
	pSyncMan = HW_GetSyncMan(PROCESS_DATA_IN);
	/* store the address of the input sync manager channel */
	nEscAddrInputData = pSyncMan->PhysicalStartAddress;
#endif
	
	/* ECAT_CHANGE_BEGIN V3.11 */
	// HBu 02.05.06: it should be checked if there are overlaps in the sync manager areas
	if (0)//( ((nEscAddrInputData+nPdInputSize*NO_OF_PD_INPUT_BUFFER) > u16EscAddrSendMbx && (nEscAddrInputData < (u16EscAddrSendMbx+u16SendMbxSize)))
		//||((nEscAddrInputData+nPdInputSize*NO_OF_PD_INPUT_BUFFER) > u16EscAddrReceiveMbx && (nEscAddrInputData < (u16EscAddrReceiveMbx+u16ReceiveMbxSize)))
		//)
	{
		//SendSmFailedEmergency(PROCESS_DATA_IN, SYNCMANCHADDRESS+1);
		return ALSTATUSCODE_INVALIDSMINCFG;
	}

	if //( ((nEscAddrOutputData+nPdOutputSize*NO_OF_PD_OUTPUT_BUFFER) > u16EscAddrSendMbx && (nEscAddrOutputData < (u16EscAddrSendMbx+u16SendMbxSize)))
		//||((nEscAddrOutputData+nPdOutputSize*NO_OF_PD_OUTPUT_BUFFER) > u16EscAddrReceiveMbx && (nEscAddrOutputData < (u16EscAddrReceiveMbx+u16ReceiveMbxSize)))
		((nEscAddrOutputData+nPdOutputSize*NO_OF_PD_OUTPUT_BUFFER) > nEscAddrInputData 
		&& (nEscAddrOutputData < (nEscAddrInputData+nPdInputSize)))
	{
		//SendSmFailedEmergency(PROCESS_DATA_OUT, SYNCMANCHADDRESS+1);
		return ALSTATUSCODE_INVALIDSMOUTCFG;
	}
	/* ECAT_CHANGE_END V3.11 */

/* ECAT_CHANGE_BEGIN V3.20 */
#if DC_SUPPORTED
	/* check the DC-Registers */
	HW_EscReadAccess(&nDcSyncControl, ESC_ADDR_SYNCCONTROL+1, 1);
	
	if ( nDcSyncControl & DC_SYNC_ACTIVE )
	{
		/* Distributed Clocks are enabled, the DC_SYNC_MODE is defined in ecat_def.h 
		   and shall match to the set value */
		if ( nDcSyncControl != DC_SYNC_MODE )
			return ALSTATUSCODE_DCINVALIDSYNCCFG;

		/* DC_SYNC_EVENT defines the mask (SYNC0 and/or SYNC1-event) for the AL-Event-Mask-Register (0x204) */
		intMask = DC_SYNC_EVENT;
		/* slave is running in DC-mode */
		bDcSyncActive = TRUE;
		/* slave is running in synchronus mode */
		bSynchronMode = TRUE;
	}
	else 
#endif	/* DC_SUPPORTED */
#if ECAT_PROCESS_INPUT_INT
	if ( bSynchronMode && nPdOutputSize == 0 )
	{
		/* if no outputs are available the INPUT (SM1)-event has to activated in synchronous mode */
		intMask |= PROCESS_INPUT_EVENT; 
	}
#else
	{
	}
#endif

#if DC_SUPPORTED
	{
		UINT32 cycleTime;

		HW_EscReadAccess((UINT8 *) &cycleTime, ESC_ADDR_SYNC_CYCLETIME, 4);
		/* check if the cycle time to the minimum and maximum vales (MIN_PDCYCLE_TIME and MAX_PD_CYCLE_TIME)
		   are defined in ecat_def.h */
		if ( (cycleTime || bDcSyncActive) && (cycleTime < MIN_PD_CYCLE_TIME || cycleTime > MAX_PD_CYCLE_TIME) )
			return ALSTATUSCODE_DCINVALIDSYNCCYCLETIME;						
#if SMPAR_SUPPORTED
		/* the requested cycle time shall be stored in the SM-Parameter objects */
#if MAX_RX_PDOS
		sSyncManOutPar.u32CycleTime = cycleTime;
#endif
#if MAX_TX_PDOS
		sSyncManInPar.u32CycleTime = cycleTime;
#endif
#endif
	}

#endif
#if SMPAR_SUPPORTED
	/* the actaual sync type shall be stored in the SM-Parameter objects */
	if (bDcSyncActive)
		syncType = SYNCTYPE_DCSYNC0;
	else if (bSynchronMode)
		syncType = SYNCTYPE_SYNCHRON;
	else
		syncType = SYNCTYPE_FREERUN;
#if MAX_RX_PDOS
	sSyncManOutPar.u16SyncType = syncType;
#endif
#if MAX_TX_PDOS
#if MAX_RX_PDOS
	if (bDcSyncActive || !bSynchronMode)
		sSyncManInPar.u16SyncType = syncType;
	else
		sSyncManInPar.u16SyncType = SYNCTYPE_SM2INT;
#else
	sSyncManInPar.u16SyncType = syncType;
#endif
#endif
#endif // SMPAR_SUPPORTED
	/* V3.02 ECAT_CHANGE_BEGIN */
	/* HBu 24.01.06: DC should be supported too if the input size is zero,
						  SM1-Event should be enabled, if the ECAT_PROCESS_INPUT_INT
						  switch is set */
#if MAX_TX_PDOS	
	/* update the input data once */
	if ( nPdInputSize > 0 )
	{
		/* the SM1-channel has to be enabled (was disabled in StopInputHandler) */
		HW_EnableSyncManChannel(PROCESS_DATA_IN);
		/* first input update has to be done before confirming SAFE-OP */
		result = APPL_StartInputHandler();
		if (result)
			return result;

		bEcatInputUpdateRunning = 1;
	}
#endif /* MAX_TX_PDOS */
#if MAX_RX_PDOS
	if ( nPdOutputSize > 0 )
	{
		/* the SM0-channel has to be enabled (was disabled in StopInputHandler), this will
		   be done here because normally the inputs and outputs will be transmit with the
		   same telegram */
		HW_EnableSyncManChannel(PROCESS_DATA_OUT);
	}

#endif

#if AL_EVENT_ENABLED
	HW_SetIntMask( intMask );
#endif
	/* V3.02 ECAT_CHANGE_END */
/* ECAT_CHANGE_END V3.20 */

	return 0;
}

/* ECAT_CHANGE_BEGIN V3.20 */
/* The contents of StartOutputHandler were coded in AL_ControlInd (SAFEOP_2_OP) before */
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	AL Status Code (see ecatslv.h ALSTATUSCODE_....)

 \brief	This function is called in case of the state transition from SAFEOP to OP

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 StartOutputHandler(void)
{
	UINT8 result = 0;
	UINT16 wdiv;
	UINT16 wd;
	HW_EscReadAccess((UINT8 *) &wdiv, ESC_WATCHDOG_DIVIDER, 2);
	/* V3.01 ECAT_CHANGE_BEGIN */
	/* HBu 24.01.06: bWdActive has to be checked too, because the 
	                 Watchdog Divider Register will not be always 
	                 overwritten during start up */
	if (wdiv && bWdActive)
	/* V3.01 ECAT_CHANGE_END */
	{
		/* WD in ms units */
		HW_EscReadAccess((UINT8 *) &wd, ESC_SM_WATCHDOG, 2);
		if (wd > 0)
		{
			/* set WD */
			UINT32 d = wd;
			d *= wdiv;
			d /= 25000;
			u16WdValue = (UINT16) d;
		}
	}
	/* V3.01 ECAT_CHANGE_BEGIN */
	else
	{
		wd = 0;
		/* V3.02 ECAT_CHANGE_BEGIN */
		/* HBu 24.01.06: the value has to be reset here */
		u16WdValue = 0;
		/* V3.02 ECAT_CHANGE_END */
	}

	/* HBu 04.12.05: it will be checked if the watchdog registers are consistent
	                 Reg 0x400 = 25000 (1 ms time base), Reg 0x420 > 0 and
						  WD-Bit of SM-Channel 2 is set (bWdActive) (watchdog active) 
						  otherwise at least Reg 0x400 has to be unequal 25000 
						  or Reg 0x420 has to be 0 and bWdActive has to FALSE
						  (watchdog not active) */
	if (wd == 0)
	{
		if (bWdActive)
		{
			return ALSTATUSCODE_INVALIDWDCFG;
		}
	}
	else
	{
		if (!bWdActive)
		{
			return ALSTATUSCODE_INVALIDWDCFG;
		}
	}
	/* V3.01 ECAT_CHANGE_END */

	if (nPdOutputSize > 0)
	{
		result = APPL_StartOutputHandler();
		if (result != 0)
			return result;
	}
	bEcatOutputUpdateRunning = TRUE;
#if AL_EVENT_ENABLED
#if MAX_RX_PDOS
#if ECAT_PROCESS_OUTPUT_INT
	if ( !bDcSyncActive && bSynchronMode && nPdOutputSize != 0 )
	{
		HW_SetIntMask(PROCESS_OUTPUT_EVENT); 
	}
#endif
#endif // MAX_RX_PDOS
#endif 

	return 0;
}
/* ECAT_CHANGE_END V3.20 */

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	 0, NOERROR_INWORK

 \brief	This function is called in case of the state transition from OP to SAFEOP

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 StopOutputHandler(void)
{
	UINT8 result = 0;

#if ECAT_PROCESS_OUTPUT_INT
	HW_ResetIntMask( ~PROCESS_OUTPUT_EVENT );
#endif
	bEcatFirstOutputsReceived = FALSE;
 	bEcatOutputUpdateRunning = FALSE;
	if ( nPdOutputSize > 0 )
		result = APPL_StopOutputHandler();

	return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return	 0, NOERROR_INWORK

 \brief	This function is called in case of the state transition from SAFEOP to OP

*////////////////////////////////////////////////////////////////////////////////////////

UINT8 StopInputHandler(void)
{
	UINT8 result = 0;

#if MAX_RX_PDOS
	HW_DisableSyncManChannel(PROCESS_DATA_OUT);
#endif
	/* V3.02 ECAT_CHANGE_BEGIN */
	/* HBu 24.01.06: SM3-Event should be disabled, if the ECAT_PROCESS_INPUT_INT
						  switch is set */
#if AL_EVENT_ENABLED
	HW_ResetIntMask( ~(SYNC0_EVENT | SYNC1_EVENT | PROCESS_INPUT_EVENT | PROCESS_OUTPUT_EVENT) );
#endif
	/* V3.02 ECAT_CHANGE_END */
/* ECAT_CHANGE_BEGIN V3.20 */
	/* flag for syncvhronous mode should be reset when switching to PREOP */
	bSynchronMode = FALSE;
/* ECAT_CHANGE_END V3.20 */
	bDcSyncActive = FALSE;
#if MAX_TX_PDOS
 	bEcatInputUpdateRunning = FALSE;
	HW_DisableSyncManChannel(PROCESS_DATA_IN);
#endif
	result = APPL_StopInputHandler();

	return result;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param	alControl		content of the AL Control register

 \brief	This function is called in case of an AL Control indication, when the Master
			has written the AL Control Register

*////////////////////////////////////////////////////////////////////////////////////////

void AL_ControlInd(UINT8 alControl)
{
	UINT8			result = 0;
	UINT8 		stateTrans;

	/* reset the Error Flag in case of acknowledge by the Master */
	if ( alControl & STATE_CHANGE )
		nAlStatus &= ~STATE_CHANGE;
	else if ( (nAlStatus & STATE_CHANGE)&&( (alControl & STATE_MASK) > (nAlStatus & STATE_MASK) ) )
		/* if the error flag is set in the AL-Status the state can only be set back */
		return;

	alControl &= STATE_MASK;

	/* generate a variable for the state transition */
	stateTrans = nAlStatus;
	stateTrans <<= 4;
	stateTrans += alControl;

	nStateTrans = stateTrans;

	switch ( stateTrans )
	{
	case INIT_2_PREOP :
	case OP_2_PREOP:
	case SAFEOP_2_PREOP:
	case PREOP_2_PREOP:
		//result = CheckSmSettings(MAILBOX_READ+1); 
		break;
	case PREOP_2_SAFEOP:
		result = APPL_GenerateMapping();
		if (result != 0)
			break;
	case SAFEOP_2_OP:
	case OP_2_SAFEOP:
	case SAFEOP_2_SAFEOP:
	case OP_2_OP:
		result = CheckSmSettings(nMaxSyncMan);
		break;
	}
	
	if ( result == 0 )
	{	
		/* execute the corresponding local management service(s) depending on the state transition */
		switch ( stateTrans )
		{
		case INIT_2_BOOT	:
			result = ALSTATUSCODE_BOOTNOTSUPP;
			break;

		case INIT_2_PREOP :
			break;

		case PREOP_2_SAFEOP:
			result = StartInputHandler();
			break;	
	
		case SAFEOP_2_OP:
/* ECAT_CHANGE_BEGIN V3.20 */
			result = StartOutputHandler();
/* ECAT_CHANGE_END V3.20 */
			break;

		case OP_2_SAFEOP:
			result = StopOutputHandler();
			break;

		case OP_2_PREOP:
			result = StopOutputHandler();
			if (result != 0)
				break;

		case SAFEOP_2_PREOP:
			result = StopInputHandler();
			break;

		case OP_2_INIT:
			result = StopOutputHandler();
			if (result != 0)
				break;

		case SAFEOP_2_INIT:
			result = StopInputHandler();
			if (result != 0)
				break;

		case PREOP_2_INIT:	 
			break;

		case INIT_2_INIT:
		case PREOP_2_PREOP:
		case SAFEOP_2_SAFEOP:
		case OP_2_OP:
			result = NOERROR_NOSTATECHANGE;
			break;

		case INIT_2_SAFEOP:
		case INIT_2_OP:
		case PREOP_2_OP:
		case PREOP_2_BOOT:
		case SAFEOP_2_BOOT:
		case OP_2_BOOT:
			result = ALSTATUSCODE_INVALIDALCONTROL;
			break;

		default:
			result = ALSTATUSCODE_UNKNOWNALCONTROL; 
			break;	
		}
	}
	else
	{
		switch (nAlStatus)
		{
		case STATE_OP:
			StopOutputHandler();
		case STATE_SAFEOP:
   		if ( result != ALSTATUSCODE_INVALIDSMOUTCFG )
			{
				StopInputHandler();
				/* V3.02 ECAT_CHANGE_BEGIN */
				/* HBu 24.01.06: Slave shall switch to state PREOP */
				nAlStatus = STATE_PREOP;
				/* V3.02 ECAT_CHANGE_END */
			}
			else
			{
				nAlStatus = STATE_SAFEOP;
				break;
			}

		case STATE_PREOP:
   		if ( result == ALSTATUSCODE_INVALIDMBXCFGINPREOP )
			{
				nAlStatus = STATE_INIT;
			}
			else		
				nAlStatus = STATE_PREOP;
		}
	}

	if ( result == NOERROR_INWORK )
	{
		/* state transition is still in work 
			HW_SetAlStatus must be called from the application */
	}
	else
	/* V3.10: ECAT_CHANGE_BEGIN */
	/* HBu 20.02.06: The AL Status Code register shall not be modified if the function is called
	   in case of SM change event or an AL-Control event with the same state */
	if ( alControl != (nAlStatus & STATE_MASK) )
	/* V3.10: ECAT_CHANGE_END */
	{
		if ( result != 0 )
		{
			/* save the failed status to be able to decide, if the AL Status Code shall be
			   reset in case of a coming successful state transition */
			nAlStatusFailed = nAlStatus;
			nAlStatus |= STATE_CHANGE;
		}
		else
		{
			/* state transition was succesful */
			if ( nAlStatusCode != 0 )
			{
				/* state change request from the user */
				result = nAlStatusCode;
				nAlStatusFailed = alControl;
				alControl |= STATE_CHANGE;
			}
			else if ( alControl <= nAlStatusFailed ) 
			{
				/* the old AL Status Code register shall not be overwritten */
				result = 0xFF;
			}
/* ECAT_CHANGE_BEGIN V3.20 */
			/* reset failed status, when state transition successful */
			else
				nAlStatusFailed = 0;
/* ECAT_CHANGE_END V3.20 */
			/* acknowledge the new state */
			nAlStatus = alControl;
		}
		
		/* write the AL Status register */
		HW_SetAlStatus(nAlStatus, result);
		nAlStatusCode = 0;
	}
	else
	{
		/* V3.10: ECAT_CHANGE_BEGIN */
		/* AL-Status has to be updated if the the error bit was acknowledged */
/* ECAT_CHANGE_BEGIN V3.20 */
		/* AL-Status-Code has to be reset if the the error bit was acknowledged */
		HW_SetAlStatus(nAlStatus, 0);
/* ECAT_CHANGE_END V3.20 */
		/* V3.10: ECAT_CHANGE_END */
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
/**

 \brief	This function intializes the EtherCAT Slave Interface.
*////////////////////////////////////////////////////////////////////////////////////////

void ECAT_Init(void)
{
	/* initialize variables */
	bWdActive = 0;
	nAlStatusFailed = 0;
	nAlStatusCode = 0;
	/* initialize flags */
	bEcatFirstOutputsReceived = FALSE;
 	bEcatOutputUpdateRunning = FALSE;
 	bEcatInputUpdateRunning = FALSE;
	bDcSyncActive = FALSE;
	bSynchronMode = FALSE;
	/* initialize the AL Status register */
	nAlStatus	= STATE_INIT;
	HW_SetAlStatus(nAlStatus, 0);

#if SMPAR_SUPPORTED
/* ECAT_CHANGE_BEGIN V3.20 */
	/* initialize SM-Paramter */
#if MAX_RX_PDOS
	sSyncManOutPar.u16SyncType = SYNCTYPE_SYNCHRON;
	sSyncManOutPar.u32CycleTime = MIN_PD_CYCLE_TIME;
	sSyncManOutPar.u32ShiftTime = PD_OUTPUT_SHIFT_TIME;
#endif
#if MAX_TX_PDOS
	sSyncManInPar.u16SyncType = SYNCTYPE_SM2INT;
	sSyncManInPar.u32CycleTime = MIN_PD_CYCLE_TIME;
	sSyncManInPar.u32ShiftTime = PD_INPUT_SHIFT_TIME;
#endif
/* ECAT_CHANGE_END V3.20 */

#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief		This function has to be called cyclically.
*////////////////////////////////////////////////////////////////////////////////////////

void ECAT_Main(void)
{
	/* check if masked interrupts were received */
	HW_Main();
}
/** @} */

#endif
