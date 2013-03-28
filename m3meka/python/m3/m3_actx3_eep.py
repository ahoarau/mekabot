#! /usr/bin/python

#M3 -- Meka Robotics Robot Components
#Copyright (c) 2010 Meka Robotics
#Author: edsinger@mekabot.com (Aaron Edsinger)

#M3 is free software: you can redistribute it and/or modify
#it under the terms of the GNU Lesser General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#M3 is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU Lesser General Public License for more details.

#You should have received a copy of the GNU Lesser General Public License
#along with M3.  If not, see <http://www.gnu.org/licenses/>.



# See the EtherCAT Device Description Documentation for format details

# ###################################### V0.4 #########################################################
slave_cfg_dict_v1 = {
    # ######################## HEADER ############################
    'Version Notes': 'V1: Adds periodic 500us Sync0. Utilizes PDI_IRQ.',
    #Bit 7:0 0b0000 0101  SPI=5
    #8: 0b0 AL status is written by PDI ( no Device Emulation) 
    #9: 0b1 Enhanced link detection
    #10: 0b0 No Enable Distributed Clocks Sync out 
    #11: 0b0 No Enable Distributed Clocks Latch in 
    #15:12: 0b0000 Reserved 
    #Result: 0b 0000 0010 0000 0101
    #'PDI Control': 0x0205, 
    'PDI Control': 0x0005, 
    #Bit 1:0 0b11 : SPI Mode 3 , (recommended for the SPI PIC sample code)
    #Bit 2: 0 : Push-pull SPI_IRQ output driver
    #Bit 3: 0 : SPI_IRQ is active low
    #Bit 4: 0 : SPI_SEL is active low
    #Bit 5: 0 : Normal data out sample mode, (recommended for the SPI PIC sample code)
    #Bit 7:6  : Reserved
    #Bit 15:8 : 0b00000000 : Default Sync/Latch PDI configuration
    #Result   : 0b00000000 00000011    
    'PDI Configuration': 0x0603,    #Enable bits 0,1,10 (SPI MODE(0:1), SYNC0 Output (10))
    'SyncImpulseLen': 50000,        #Sync signal is cleared by reading Sync0/Sync1
    'Configured Station Alias': 0,  #Use Auto-Increment-Addressing on slave
    'Extended PDI': 0x0000,         #Reserved
    'Reserved': [0, 0, 0, 0],
    'Checksum': 0x88A4,           #Disables checksum for debugging only
    'Vendor ID': 1304L,           #Meka Vendor ID=0x00000518
    'Product Code': 0,            #Meka Product number
    'Revision Number': 5L,
    'Serial Number': 1L,
    'Execution Delay': 0,
    'Port0 Delay': 0,
    'Port1 Delay': 0,
    'Reserved (2)': 0,
    'Bootstrap Receive Mailbox Offset': 0, 
    'Bootstrap Receive Mailbox Size': 0,
    'Bootstrap Send Mailbox Offset': 0,
    'Bootstrap Send Mailbox Size': 0,
    'Standard Receive Mailbox Offset': 0,
    'Standard Receive Mailbox Size': 0,
    'Standard Send Mailbox Offset': 0,
    'Standard Send Mailbox Size': 0,
    'Mailbox Protocol': 0,         #No Mailboxes
    'Size': 16,                    #EEPROM is 16Kbit =16*1Kbit
    'Version': 1,                  #This Version is 1 according to Datasheet
    'Reserved (3)': [0]*66,
    'Categories': [   {
        # ######################## STRINGS ############################
        'Category Data Word Size': -1,
        'Category Type': 10,
        #STRING INDICES START AT 1
        'Data': [   'Meka Robotics', 
                    'M3EC ActX3 Controller',
                    'M3EC',
                    'Process Inputs (Status)',
                    'Process Outputs (Command)',
                    'StatusA',
                    'StatusB',
                    'StatusC',
		    'CommandA',
		    'CommandB',
                    'CommandC',
                    'CommandD'],
        'Vendor Specific': 0},
        # ######################## GENERAL ############################
        {   'Category Data Word Size': -1,
            'Category Type': 30,
            'Data': {   #Etherlab expects signed 16, typo in EtherCAT Device Datasheet	
                        'CurrentOnEBus': 68,#ET1200 Datasheet shows 68ma current consumption for ports. 
                        'CoE Details': 0,   #Disable
                        'DS402Channels': 0, #Reserved
                        'EoE Details': 0,   #Disable
                        'FoE Details': 0,   #Disable
                        'Flags': 0,         #Disable SafeOp and Disable notLRW (Etherlab Master uses LRW datagrams)
                        'GroupIdx': 1, # String IDX
                        'ImgIdx': 3,   # String IDX
                        'NameIdx': 2,  # String IDX
                        'OrderIdx': 3, # String IDX
                        # EBUS, EBUS, EBUS, EBUS
                        'Physical Layer Ports': [0, 0, 0, 0], 
                        'SoEChannels': 0,  #Reserved
                        'SysmanClass': 0,  #Reserved
                        'PAD_Byte': [0xFF]*18},
            'Vendor Specific': 0},
        # ######################## FMMU ############################
        {   'Category Data Word Size': -1,
            'Category Type': 40,
            'Data': {'FMMU0': 1, 'FMMU1': 2}, #FMMU0 used for Outputs, FMMU1 used for Inputs
            'Vendor Specific': 0},
        # ######################## SYNCM ############################
        {   'Category Data Word Size': -1,
            'Category Type': 41,
            'Data': [   
                {   #SYNCM0 PROCESS (COMMAND)
                    'Activate': 0X01,                 #Enable sync manager    
                    'Control Register': 0X24,         #Ecat write, 3 Buffer mode, PDI interrupt, no Watchdog  (0b0010 0100)
                    'Length': 120,                     #120 BYTES
                    'PDI CTRL': 0X00,                 #No PDI control of SyncManager
                    'Physical Start Address': 0X1000, #Define command values to appear here
                    'Status Register': 0X00},         #Read-only         
                {    #SYNCM1 PROCESS (STATUS)
                    'Activate': 0X01,                 #Enable sync manager    
                    'Control Register': 0X20,         #Ecat read, 3 Buffer mode, PDI interrupt, no Watchdog (0b0010 0000)
                    'Length': 90,                    #90 BYTES
                    'PDI CTRL': 0X00,                 #No PDI control of SyncManager
                    'Physical Start Address': 0X1200, #Define staus values to appear here
                    'Status Register': 0X00},         #Read-only      
                []],
            'Vendor Specific': 0},
        # ######################## STATUS TXPDO ############################
        {   'Category Data Word Size': -1,
            'Category Type': 50,
            'Data': [   
                {   #STATUS PDO Definition
                    'PDO Index': 0X1A00,             #For PDOy, RXPDO is at 0x160y, TXPDO is at 0x1a0y
                    'nEntry': 3,                     #Number of Entries in PDO 
		    'SyncM': 1,                      #SyncM1 is for inputs
		    'Synchronization': 0,            #No DC Sync
		    'Flags (1)': 0,                  #N/A,    
		    'NameIdx': 4 },                  #String IDX
		{    #Entry 1: StatusA
		    'Entry Index': 0X6000,           #Output module memory space
                    'Entry Name Idx': 6,             #String IDX
                    'BitLen': 240,                   #Size of entry=30 Bytes
                    'Flags (2)': 0,                  #N/A
                    'Data Type': 0,                  #N/A for Etherlab
                    'Subindex': 1},                  #PDO Entry Number 
		{   #Entry 2: StatusB
		    'Entry Index': 0X6000,           #Output module memory space
                    'Entry Name Idx': 7,             #String IDX
		    'BitLen': 240,                   #Size of entry=30 Bytes
                    'Flags (2)': 0,                  #N/A
		    'Data Type': 0,                  #N/A for Etherlab
                    'Subindex': 2},                  #PDO Entry Number      
                {   #Entry 3: StatusC
		    'Entry Index': 0X6000,           #Output module memory space
                    'Entry Name Idx': 8,             #String IDX
		    'BitLen': 240,                   #Size of entry=30 Bytes
                    'Flags (2)': 0,                  #N/A
		    'Data Type': 0,                  #N/A for Etherlab
                    'Subindex': 3},                  #PDO Entry Number      
                []],
            'Vendor Specific': 0},
        # ######################## COMMAND RXPDO ############################
        {   'Category Data Word Size': -1,
            'Category Type': 51,
            'Data': [   
                {   
                    #COMMAND PDO Definition
                    'PDO Index': 0X1600,             #For PDOy, RXPDO is at 0x160y, TXPDO is at 0x1a0y
                    'nEntry': 4,                     #Number of Entries in PDO 
		    'SyncM': 0,                      #SyncM0 is for outputs
                    'Synchronization': 0,            #No DC Sync
		    'Flags (1)': 0,                  #N/A
		    'NameIdx': 5},                   #String IDX,                
		{   #Entry 1: CommandA
                    'Entry Index': 0X7000,           #Input module memory space
                    'Entry Name Idx': 9,             #String IDX
		    'BitLen': 240,                   #Size of entry=30 Bytes
                    'Flags (2)': 0,                  #N/A
		    'Data Type': 0,                  #N/A for Etherlab
                    'Subindex': 1},                  #PDO Entry Number    
		{   #Entry 2: CommandB
                    'Entry Index': 0X7000,           #Input module memory space
                    'Entry Name Idx': 10,             #String IDX
		    'BitLen': 240,                   #Size of entry=30 Bytes
                    'Flags (2)': 0,                  #N/A
		    'Data Type': 0,                  #N/A for Etherlab
                    'Subindex': 2},                   #PDO Entry Number  
                {   #Entry 3: CommandC
                    'Entry Index': 0X7000,           #Input module memory space
                    'Entry Name Idx': 11,             #String IDX
		    'BitLen': 240,                   #Size of entry=30 Bytes
                    'Flags (2)': 0,                  #N/A
		    'Data Type': 0,                  #N/A for Etherlab
                    'Subindex': 3},                   #PDO Entry Number  
                 {   #Entry 4: CommandD
                    'Entry Index': 0X7000,           #Input module memory space
                    'Entry Name Idx': 12,             #String IDX
		    'BitLen': 240,                   #Size of entry=30 Bytes
                    'Flags (2)': 0,                  #N/A
		    'Data Type': 0,                  #N/A for Etherlab
                    'Subindex': 4},                   #PDO Entry Number  
                []],
            'Vendor Specific': 0}]}

# #################################################################################################

slave_cfg_dicts= [slave_cfg_dict_v1] 

