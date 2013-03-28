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
#along with M3.  If not, see <http://www.gnu.org/licenses/>..

# See the EtherCAT Device Description Documentation for format details


slave_cfg_dict_v0 = {   
    # ######################## HEADER ############################
    'Version Notes': 'V0: Initial release. Works with ECHUB v0.1.',
    #Bit 7:0 0b0111  EtherCAT Bridge (port 3)
    #8: 0b1 AL status is written by AL control reg (Device Emulation for Hub) 
    #9: 0b0 No Enhanced link detection
    #10: 0b0 No Enable Distributed Clocks Sync out 
    #11: 0b0 No Enable Distributed Clocks Latch in 
    #15:12: 0b0000 Reserved 
    #Result: 0b 0000 0001 0000 0111
    #NOTE: Enhanced Link Detect on Hub only. This works for now, but all slaves should use it.
    #'PDI Control': 0x0307, Found that Enhanced Link Detect causes Port3 to not work
    'PDI Control': 0x0107,
    #Default Sync/Latch Configuration 
    #Active High outputs 
    #Unidirectional mode 
    #Outputs reset after watchdog expire 
    #Input sampled at start of frame
    'PDI Configuration': 0x0000,

    'Configured Station Alias': 0,  #Use Auto-Increment-Addressing on slave
    'SyncImpulseLen': 0,            #Sync signal is cleared by reading Sync0/Sync1
    
    #DIO for ET1200 
    'Extended PDI': 0,
    
    'Reserved': [0, 0, 0, 0],
    'Checksum': 0x88A4,           #Disables checksum for debugging only
    'Vendor ID': 1304L,           #Meka Vendor ID=0x00000518
    'Product Code': 0,            #Meka Product number
    'Revision Number': 0L,
    'Serial Number': 0L,
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
                    'M3 ECHUB EtherCat Hub',
                    'M3ECHUB'],
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
                        # 100BASE-TX, EBUS, EBUS, EBUS
                        'Physical Layer Ports': [1, 0, 0, 0], 
                        'SoEChannels': 0,  #Reserved
                        'SysmanClass': 0,  #Reserved
                        'PAD_Byte': [0xFF]*18},
            'Vendor Specific': 0},
    ]}

slave_cfg_dicts= [slave_cfg_dict_v0] 