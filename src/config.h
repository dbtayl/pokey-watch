// ****************************************************************************
// *** Copyright (c) 2011, Universal Air Ltd. All rights reserved.
// *** Source and binaries are released under the BSD 3-Clause license
// *** See readme_forebrain.txt files for the text of the license
// ****************************************************************************
 
#ifndef __CONFIG_H__
#define __CONFIG_H__

// ****************************************************************************
// *** Misc Functions
// ****************************************************************************

#define IAP_EN              0           // Set to 1 to enable the IAP functions like Reprogram() (set to 0 to disable and save some RAM)

#define RAND_A              16644525    // Coefficient for the linear congruential random number generator
#define RAND_C              32767       // Coefficient for the linear congruential random number generator
 
#define CRP                 0xffffffff  // Code read protection settings
                                        // 0xffffffff = No CRP
                                        // 0x4e697370 = No ISP
                                        // 0x12345678 = CRP1
                                        // 0x87654321 = CRP2
                                        // ********** = CRP3  WARNING! THIS STOPS EXTERNAL ACCESS TO PROGRAMMING MODE (user code must provide Reprogram() functionality to update flash)
                                        //                      This is so dangerous, we're not giving you the value, it is up to you to seek this value out from the
                                        //                      LPC1343 manuals and datasheets.

                                        
// ****************************************************************************
// *** Clock Functions
// ****************************************************************************
#define DEFAULT_CLOCK       0           // Default start-up/wake-up clock mode (0=72MHz crystal, 1=72MHz IRC, 2=12MHz IRC)
 
#define SYSTICK_EN          0           // Set to 1 to enable SysTick (set to 0 to disable and save some RAM)
#define SYSTICK_STARTUP     0           // Set to 1 to enable SysTick timer during startup
#define SYSTICK_MS          1           // SysTick period in milliseconds
#define SYSTICK_PRIORITY    0           // SysTick interrupt priority

#define WDT_PRIORITY        0           // WDT interrupt priority


// ****************************************************************************
// *** Power mode Config
// ****************************************************************************
#define AUTOSLEEP           0           // Set to 1 to sleep on end of interrupt function
#define WAKEUP_HYS          0           // Set 1 to enable hysteresis on the WAKEUP pin

#define STARTUP_DELAY       0       // Startup delay (in terms of empty loop

// ****************************************************************************
// *** GPIO Config
// ****************************************************************************
#define PORT_STARTUP_INIT     1         // Whether to initialise all ports during startup
#define PORT0_PRIORITY        2         // Port 0 pins interrupt priority
#define PORT1_PRIORITY        2         // Port 1 pins interrupt priority
#define PORT2_PRIORITY        2         // Port 2 pins interrupt priority
#define PORT3_PRIORITY        2         // Port 3 pins interrupt priority


// ****************************************************************************
// *** USB HID and MSC functions
// ****************************************************************************
#define USB_EN              0           // Set to 1 to enable USB ROM functions

#define USB_VENDOR_ID       0x1FC9      // The default NXP Vendor ID is 0x1FC9
#define USB_PRODUCT_ID      0x0003
#define USB_DEVICE_ID       0x0001
#define USB_IN_BYTES        64          // The number of bytes for the HID input report (Forebrain to computer) (max. 64)
#define USB_OUT_BYTES       64          // The number of bytes for the HID output report (computer to Forebrain) (max. 64)

#define MSC_BLOCK_SIZE      512         // The block size (bytes) of the MSC device

#define USB_PRIORITY        2           // USB interrupt priority


// ****************************************************************************
// *** UART Config
// ****************************************************************************
#define UART_EN             1           // Set to 1 to enable UART (set to 0 to disable and save some RAM)
#define UART_MODE           1           // UART read mode (0=on-demand, 1=interrupt)
#define UART_DATA_SIZE      64          // Size of UART data buffer
#define UART_FLOW           0           // Set to 1 to enable UART auto-flow control (CTS and RTS)

#define UART_BIT_LENGTH     3           // Bits per UART "word" (3=8-bit, 2=7-bit, 1=6-bit, 0=5-bit)
#define UART_STOP_BITS      0           // Stop bits (0=1 stop bit, 1=2 stop bits (or 1.5 stop bits if 5-bit data mode))
#define UART_PARITY_EN      0           // Parity (0=disable, 1=enable)
#define UART_PARITY         0           // Parity type (0=Odd parity, 1=Even parity, 2=force mark parity, 3=force space parity)

#define UART_USE_FBR        1           // Set to 1 to use pre-defined fractional baud rates
#define UART_PRIORITY       2           // UART interrupt priority


// ****************************************************************************
// *** I2C Config
// ****************************************************************************
#define I2C_EN              0           // Set to 1 to enable I2C (set to 0 to disable and save some RAM)
#define I2C_SLAVE_EN        0           // Set to 1 to enable slave mode
#define I2C_DATA_SIZE       68          // Size of I2C buffer
#define I2C_TIMEOUT         0xFFFFF     // Timeout for I2C

#define I2C_SLAVE_ADR0      0xe8        // Slave address 0 when Forebrain is in slave mode
#define I2C_SLAVE_ADR1      0x00        // Slave address 1 when Forebrain is in slave mode
#define I2C_SLAVE_ADR2      0x00        // Slave address 2 when Forebrain is in slave mode
#define I2C_SLAVE_ADR3      0x00        // Slave address 3 when Forebrain is in slave mode

#define I2C_EEPROM_ADR      0xA0        // address of the 24FC256 EEPROM on Forebrain

#define I2C_PRIORITY        1           // I2C interrupt priority


// ****************************************************************************
// *** SSP/SPI Config
// ****************************************************************************
#define SSP_EN              0           // Set to 1 to enable SSP (set to 0 to disable and save some RAM)

#define SSP_SCK             0           // Select SCK pin (0=P0[10], 1=P2[11], 2=P0[6])
#define SSP_SSEL            1           // Use SSEL pin (1=use SSEL, 0=don't use)
#define SSP_MODE            0           // SSP Mode, currently not supported, leave as 0
#define SSP_SD_SPEED				20000				// SPI speed (for SD card)- 25000 is max (?), 20000 seems good for short wires

// ****************************************************************************
// *** Timer Config
// ****************************************************************************
#define TIMER0_PRIORITY     2           // Timer priority
#define TIMER1_PRIORITY     2           // Timer priority
#define TIMER2_PRIORITY     2           // Timer priority
#define TIMER3_PRIORITY     2           // Timer priority


// ****************************************************************************
// *** ADC Config
// ****************************************************************************
#define ADC_MODE            0           // ADC mode (0=on-demand, 1=interrupt)
#define ADC_PRIORITY        3           // ADC interrupt priority

#endif // __CONFIG_H__
