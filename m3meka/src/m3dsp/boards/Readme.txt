New 2011 M3 dsPIC Code organization

Main SVN repo: ..\meka_local\projects\m3meka\trunk\src\
dsPIC code: \meka_local\projects\m3meka\trunk\src\m3dsp\

Structure:

This example is for the MAX2 board.

=> m3dsp
     // Legacy projects are here...
     => boards                               // add new projects here
          => MAX2                            // This is for the MAX2 boards
               => misc                       // linker, simulation files, etc
               => src                        // common sources for all MAX2 projects
               => max2_bdc_0_3_a2r2          // This is a project, linked to a particular robot and hardware
                    => temp                  // .o temp build folder
                    => output                // .hex w/ project's name
               => max2_bldc_0_3_a2r2         // Another MAX2 project
          => PWR
              // ...

Notes:

	* The .mcp contains the pound defines
	* Make sure that all the projects' path are relatives! (there is an option in MPLAB to make it U by default when 	you add files)
	* Do the SVN update in the boards directory, not on the specific project (ex.: SVN Update MAX2 instead of SVN 	Update max2_bdc_0_3_a2r2, so you'll have all the up to date source files)
	* Optimized for maximum speed (-O3)
	* Here is a good .mcp, as a reference:


[HEADER]
magic_cookie={66E99B07-E706-4689-9E80-9B2582898A13}
file_version=1.0
device=dsPIC33FJ32MC204
[PATH_INFO]
BuildDirPolicy=BuildDirIsProjectDir
dir_src=
dir_bin=output
dir_tmp=temp
dir_sin=
dir_inc=
dir_lib=
dir_lkr=
[CAT_FILTERS]
filter_src=*.s;*.c
filter_inc=*.h;*.inc
filter_obj=*.o
filter_lib=*.a
filter_lkr=*.gld
[CAT_SUBFOLDERS]
subfolder_src=
subfolder_inc=
subfolder_obj=
subfolder_lib=
subfolder_lkr=
[FILE_SUBFOLDERS]
file_000=.
file_001=.
file_002=.
file_003=.
file_004=.
file_005=.
file_006=.
file_007=.
file_008=.
file_009=.
file_010=.
file_011=.
file_012=.
file_013=.
file_014=.
file_015=.
file_016=.
file_017=.
file_018=.
file_019=.
file_020=.
file_021=.
file_022=.
file_023=.
file_024=.
file_025=.
file_026=.
file_027=.
file_028=.
file_029=.
file_030=.
file_031=.
file_032=.
[GENERATED_FILES]
file_000=no
file_001=no
file_002=no
file_003=no
file_004=no
file_005=no
file_006=no
file_007=no
file_008=no
file_009=no
file_010=no
file_011=no
file_012=no
file_013=no
file_014=no
file_015=no
file_016=no
file_017=no
file_018=no
file_019=no
file_020=no
file_021=no
file_022=no
file_023=no
file_024=no
file_025=no
file_026=no
file_027=no
file_028=no
file_029=no
file_030=no
file_031=no
file_032=no
[OTHER_FILES]
file_000=no
file_001=no
file_002=no
file_003=no
file_004=no
file_005=no
file_006=no
file_007=no
file_008=no
file_009=no
file_010=no
file_011=no
file_012=no
file_013=no
file_014=no
file_015=no
file_016=no
file_017=no
file_018=no
file_019=no
file_020=no
file_021=no
file_022=no
file_023=no
file_024=no
file_025=no
file_026=no
file_027=no
file_028=no
file_029=no
file_030=no
file_031=no
file_032=no
[FILE_INFO]
file_000=..\src\adc.c
file_001=..\src\control.c
file_002=..\src\current.c
file_003=..\src\dio.c
file_004=..\src\encoder_vertx.c
file_005=..\src\ethercat.c
file_006=..\src\ethercat_appl.c
file_007=..\src\ethercat_hw.c
file_008=..\src\ethercat_slave_fsm.c
file_009=..\src\main.c
file_010=..\src\pwm.c
file_011=..\src\setup.c
file_012=..\src\timer1.c
file_013=..\src\timer3.c
file_014=..\src\adc.h
file_015=..\src\control.h
file_016=..\src\current.h
file_017=..\src\dio.h
file_018=..\src\encoder_vertx.h
file_019=..\src\ethercat.h
file_020=..\src\ethercat_appl.h
file_021=..\src\ethercat_def.h
file_022=..\src\ethercat_esc.h
file_023=..\src\ethercat_hw.h
file_024=..\src\ethercat_slave_fsm.h
file_025=..\src\inttypes.h
file_026=..\src\p33FJ32MC204.h
file_027=..\src\pwm.h
file_028=..\src\setup.h
file_029=..\src\timer1.h
file_030=..\src\timer3.h
file_031=..\src\warning.h
file_032=..\misc\p33FJ32MC204_APP.gld
[SUITE_INFO]
suite_guid={479DDE59-4D56-455E-855E-FFF59A3DB57E}
suite_state=
[TOOL_SETTINGS]
TS{7D9C6ECE-785D-44CB-BA22-17BF2E119622}=-g
TS{25AC22BD-2378-4FDB-BFB6-7345A15512D3}=-g -Wall -DEMBEDDED -DUSE_DIO -DUSE_ADC -DUSE_PWM -DUSE_TIMESTAMP_DC -DUSE_ETHERCAT -DUSE_CURRENT -DUSE_MAX2_0_3 -DUSE_CONTROL -DUSE_ENCODER_VERTX -DUSE_WATCHDOG -DMAX2_BDC_0_3_A2R2 -O3
TS{7DAC9A1D-4C45-45D6-B25A-D117C74E8F5A}=-o"$(BINDIR_)$(TARGETBASE).$(TARGETSUFFIX)" -Map="$(BINDIR_)$(TARGETBASE).map" --report-mem
TS{509E5861-1E2A-483B-8B6B-CA8DB7F2DD78}=
[INSTRUMENTED_TRACE]
enable=0
transport=0
format=0
[CUSTOM_BUILD]
Pre-Build=
Pre-BuildEnabled=1
Post-Build=
Post-BuildEnabled=1
