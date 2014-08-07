Copyright (c) 2011, Universal Air Ltd. All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

 -  Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
 -  Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 -  Neither the name of the organization nor the names of its contributors may 
    be used to endorse or promote products derived from this software without
    specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This is the template project containing the Forebrain Function Set which allows
allows users to easily make use of the LPC1343's hardware featuress.  Also
included are the functions and files for the Forebrain-compatible you may 
safely ignore or delete files whose functions you are not using.

This function set is designed for use with Forebrain-family development
boards but most functions will be useable with other LPC1343 boards or
projects.


.cproject				    Eclipse project file
.project				    Eclipse project file
config.h				    Configurations for Forebrain
main.c					    Your code here!
readme_forebrain.txt        This file
build/core_cm3.c		    Core LPC1343 file
build/core_cm3.h		    Core LPC1343 file
build/hivebrain.c		    Functions for Hivebrain (Xbee)
build/hivebrain.h		    Functions for Hivebrain (Xbee)
build/linker.ld			    Linker script
build/LPC13xx.h			    Core LPC1343 file
build/Makefile			    Compiler script
build/seraphim.c            Functions for Seraphim (IMU and GPS)
build/seraphim.h            Functions for Seraphim (IMU and GPS)
build/startup.c			    Core LPC1343 file
build/uafunc.c			    Universal Air Forebrain function set
build/uafunc.h			    Universal Air Forebrain function set
build/uasd.c			    SD card functions
build/uasd.h			    SD card functions
build/FatFs                 ChaN's FatFs libs


TODO:
    Forebrain:
    - RS-485 support and modem control
	- SSP slave modes and other modes other than SPI master
    - Streamline Timer duty change
    - Streamline I2C RAM usage
    Hivebrain:
    - XBee API mode
    Seraphim:
    - GPS: low power modes
    - GPS: NMEA mode
    - Use GPS for filesystem timer
        
Known issues:
    Forebrain:
	- EEPROMWrite sometimes writes corrupted data, we recommend using EEPROMWriteByte
    Seraphim:
    - During power-on, the uBlox NEO-6Q module will attempt to take over the
      I2C as bus master the easiest way to prevent this is to run
      GPSWorkaround() at the beginning of the setup() or main() function.

Changelog:

Revision 15 (2011-01-18)
    - Changed this readme file name
    - Combined config files, and moved back to root folder (better this way, yes?)
    - Added Port#GetInterrupt()
    - Fixed WDT interrupt
    - Added Startup timer settings
    - Added SysTickTimer startup settings
    - Copied startup timer code to startup.c to reduce flash usage in most situations
    - Added ability to disable boot-up port initialisation, reduces flash usage in some situations
    - Added Timer#EnableInterrupt() and Timer#DisableInterrupt() functions
    - Added option to turn off fractional baud rate calculations
    - Fixed bug in WriteBootData4()
    
Revision 14 (2011-12-18)
    - Combined Seraphim, Hivebrain and Forebrain Function Sets into one template
    - Added SD card functions
    - Added ChaN's FatFS library
    - Reordered SSPInit() to work for SD cards
    - Added SDTimer() function to System tick timer for use with SD cards
    - Added better TRUE/FALSE and NULL definitions
    - Moved all config files into /build folder
    - Changed ADCInit to accept unsigned short type

Seraphim Function Set Revision 5 (2011-12-16)
    - Changed BaroConvAlt() to return altitude in millimeters!
    - Fixed GPSPoll to prevent return on fail due to left over data in array
    - Changed axes to convention (x: forward, y: starboard, z: down)

Seraphim Function Set Revision 4 (2011-11-29)
    - Changed GPSChecksum to add start argument

Seraphim Function Set Revision 3 (2011-11-28)
    - Removed redundant GPS_EN definition
    - Removed GPSDisable() function
    - Added GPSSleep() function
    - Added GPS startup command to GPSInit() function

Revision 13 (2011-11-28)
    - Fixed bug in Timer1Reset(), Timer2Reset() and Timer3Reset()
    - Changed UART code to use unsigned char instead of char
    - Commented out a couple of lines the UARTWrite functions relating to loopback fixes
    
Seraphim Function Set Revision 2 (2011-11-27)
    - Added GPSDisable() function
    - Moved GPS precalulated messages to local scope within GPSInit()
    
Seraphim Function Set Revision 1 (2011-11-24)
    - Added GPS functions
    - Added Gyro functions
    - Added Accelerometer functions
    - Added Magneto functions
    - Added Barometer functions
	Original version
    
Revision 12 (2011-11-02)
    - Changed definitions in uafunc.h to declarations, thanks Tim Hutt

Revision 11 (2011-09-23)
    - Fixed bug in UART on-demand mode
    - Changed order of arguments on UARTSetInterrupt

Revision 10 (2011-09-16)
    - Changed version numbering format (retroactive), thanks Stephen Swindley!
    - Added comments and annotations
    - Removed set data line of Port#Init functions
    - Changed PULL_UP and PULL_DOWN definitions to PULLUP and PULLDOWN
    - Changed OUTPUT_HIGH, etc. to OUTPUTHIGH, etc.
    - Fixed bug in timer match and interrupt functions for all timers
    - Fixed bug in hardware PWM
    - Added Timer#SetStatus functions

Revision 9 (2011-09-11)
    - Added UART auto-flow control option in config.h
    - Fixed bug in UART fractional buad generator presets at 9600 baud on 12MHz clock
    - Added SSP functions for SPI mode
    
Revision 8 (2011-09-10)
    - Added Brownout functions
    - DeepSleep() timed wakeup
    - Added ClockMode() function
    - Added 12MHz and 72MHz clock modes
    - Added fractional baud rate calibrations for UART in 12MHz mode
    - Added Boot-up clock mode selection in config.h
    - Added ResetStatus() function
    - Changed WDTInit() to accept WDT period in ms and interrupt/reset mode
    - Added WDTStop()
    
Revision 7 (2011-09-07)
    - Added CRP definitions
    - Added condition to Delay() to not use SysTick when it is enabled but stopped
    - Added Sleep(), DeepSleep() and PowerDown() (and Read/Write Boot Data()) functions
    - Added ClockOut() function

Revision 6 (2011-08-30)
    - Prefixed all global vars used by Forebrain Function Set (FFS) with "FBR" to avoid potential collision with user code
    - Fixed bug in EEPROMRead() function
    - Fixed bug in MakeInt() function
    - Removed Soft PWM functions (these will be available in a separate library)
    - Added GPIO Interrupts
    - Added Timer and PWM functions (should work but not extensively tested)

Revision 5 (2011-08-28)
    - Added USB MSC built-in functions
    - Changed USB HID functions to accept sample rate
    - Changed EEPROMRead to EEPROMReadByte and EEPROMPageRead to EEPROMRead (similarly for EEPROMWrite) for consistency
    - Added Port#Data() functions
    - Added LEDWrite() function
    
Revision 4 (2011-08-27)
    - Changed versioning numbers
    - Added UART functions
    - Added UART autobaud and fractional baud

Revision 3 (2011-08-26)
    - Changed some variable names in the byte manipulation functions
    - Added ResetInit()
    - Added Random functions
    - Added EEPROM functions

Revision 2 (2011-08-26)
    - Simplified I2C functions
    - Added some stuff I can't quite remember...

Revision 1 (2011-08-01)
	Original version