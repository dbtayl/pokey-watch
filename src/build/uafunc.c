// ****************************************************************************
// *** Copyright (c) 2011, Universal Air Ltd. All rights reserved.
// *** Source and binaries are released under the BSD 3-Clause license
// *** See readme_forebrain.txt files for the text of the license
// ****************************************************************************

#include "LPC13xx.h"
#include "uafunc.h"
#include "config.h"

// ****************************************************************************
// *** Misc Functions
// ****************************************************************************

// *** In-application programmingfunctions
#if IAP_EN
    unsigned int FBRIAPCommand[5], FBRIAPResult[4];
	FBRIAP FBRIAPEntry = (FBRIAP)0x1fff1ff1;
	
    // *** Reprogram function
	void Reprogram(void) {
		FBRIAPCommand[0] = 57;					    // 57 is code to reinvoke ISP
		LPC_SYSCON->SYSAHBCLKDIV = 1;			    // make sure clock divider is 1
		LPC_SYSCON->SYSAHBCLKCTRL |= 0x14440;	    // turn on USB clock, Timer 32B1, GPIO, and IO blocks 

		__set_MSP(*((unsigned int *)0x1FFF0000));	// set pointer to bootloader ROM location
		FBRIAPEntry(FBRIAPCommand, FBRIAPResult);
	}
#endif

// *** Reset function
void Reset(void) {
    NVIC_SystemReset();
}

// *** Random number functions
unsigned int FBRRandomNumber;

// *** Set Code Read Protection
__attribute__ ((section(".crp"))) const unsigned int CRP_WORD = CRP;


// ****************************************************************************
// *** Clock Functions
// ****************************************************************************

// *** Sets up main clock mode
void ClockMode(unsigned char mode) {
    unsigned int i;
    switch(mode) {
        case XTAL:
        	// Clock setup for 72MHz from the external crystal
            LPC_SYSCON->PDRUNCFG &= ~(1 << 5);				// Power up system oscillator
            for(i = 0; i < 100; i++);						// Brief delay
            LPC_SYSCON->SYSOSCCTRL = 0;						// System oscillator setup - not bypass, 1-20MHz range
            LPC_SYSCON->SYSPLLCLKSEL = 0x01;   				// Select system oscillator as PLL source
            LPC_SYSCON->SYSPLLCLKUEN = 0x00;               	// Update clock source
            LPC_SYSCON->SYSPLLCLKUEN = 0x01;
            while(!(LPC_SYSCON->SYSPLLCLKUEN & 0x01));     // Wait for update
            LPC_SYSCON->MAINCLKSEL = 0x01;     				// Select sys osc as main clock source
            LPC_SYSCON->MAINCLKUEN = 0x00;               	// Update clock source
            LPC_SYSCON->MAINCLKUEN = 0x01;
            while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       // Wait for clock update
            LPC_SYSCON->SYSPLLCTRL = 0x25;					// Select PLL divider to 6 (12MHz - 72MHz)
            LPC_SYSCON->PDRUNCFG &= ~(1 << 7);          	// Power up PLL
            while(!(LPC_SYSCON->SYSPLLSTAT & 0x01));	    // Wait for PLL lock
            LPC_SYSCON->MAINCLKSEL = 0x03;     				// Select PLL as main clock source
            LPC_SYSCON->MAINCLKUEN = 0x00;               	// Update clock source
            LPC_SYSCON->MAINCLKUEN = 0x01;
            while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       // Wait for clock update
            //LPC_SYSCON->PDRUNCFG |= ((1<<0) | (1<<1));      // Power down IRC oscillator (edit: don't power down, causes USB to choke)
            break;
        case IRC72:
            // Clock setup for 72MHz from the internal oscillator
            LPC_SYSCON->PDRUNCFG &= ~((1<<0) | (1<<1));     // Power up IRC oscillator
            for(i = 0; i < 100; i++);						// Brief delay
            LPC_SYSCON->MAINCLKSEL = 0x00;     				// Select IRC as main clock source
            LPC_SYSCON->MAINCLKUEN = 0x00;               	// Update clock source
            LPC_SYSCON->MAINCLKUEN = 0x01;
            while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       // Wait for clock update
            LPC_SYSCON->SYSPLLCLKSEL = 0x00;   				// Select IRC as PLL source
            LPC_SYSCON->SYSPLLCLKUEN = 0x00;               	// Update clock source
            LPC_SYSCON->SYSPLLCLKUEN = 0x01;
            while(!(LPC_SYSCON->SYSPLLCLKUEN & 0x01));      // Wait for update
            LPC_SYSCON->SYSPLLCTRL = 0x25;					// Select PLL divider to 6 (12MHz - 72MHz)
            LPC_SYSCON->PDRUNCFG &= ~(1<<7);          	    // Power up PLL
            while(!(LPC_SYSCON->SYSPLLSTAT & 0x01));	    // Wait for PLL lock
            LPC_SYSCON->MAINCLKSEL = 0x03;     				// Select PLL as main clock source
            LPC_SYSCON->MAINCLKUEN = 0x00;               	// Update clock source
            LPC_SYSCON->MAINCLKUEN = 0x01;
            while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       // Wait for clock update
            LPC_SYSCON->PDRUNCFG |= (1<<5);                 // Power down system oscillator
            break;
        case IRC12:
            // Clock setup for 12MHz from the internal oscillator
            LPC_SYSCON->PDRUNCFG &= ~((1<<0) | (1<<1));     // Power up IRC oscillator
            for(i = 0; i < 100; i++);						// Brief delay
            LPC_SYSCON->MAINCLKSEL = 0x00;     				// Select IRC as main clock source
            LPC_SYSCON->MAINCLKUEN = 0x00;               	// Update clock source
            LPC_SYSCON->MAINCLKUEN = 0x01;
            while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       // Wait for clock update
            LPC_SYSCON->PDRUNCFG |= ((1<<5) | (1<<7));      // Power down system oscillator and IRC
            break;
    }
    #if SYSTICK_EN
        if(SysTick->CTRL & 0x1) SysTickInit(); // reinitialise SysTick to update clock times, user should reinitialise any other peripherals that may be using the clock.
    #endif
}

// *** Delay functions
void Delay(unsigned int milliseconds) {
    // detect if Systick is enabled (and turned on) and use the relevant delay function
    #if SYSTICK_EN
        if(SysTick->CTRL & 0x1) SysTickDelay(milliseconds);
        else WaitDelay(milliseconds);
    #else
        WaitDelay(milliseconds);
    #endif
}

void WaitDelay(unsigned int milliseconds) {
	volatile unsigned int i, j;
    // Some empty loops depending on the clock setting
    if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
        // assume 72MHz operation
        for(j=0; j<milliseconds; j++) {
            for(i=0; i<6000; i++);
        }
    }
    else {
        // assume 12MHz operation
        for(j=0; j<milliseconds; j++) {
            for(i=0; i<1000; i++);
        }
    }
}

#if SYSTICK_EN

    // *** System tick timer functions
	void SysTickInit(void) {	// this function is run automaticlly by the startup script if SysTick is enabled
		// configure the timer based on oscillator speed, these are already defined in core_cm3
        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            // assume 72MHz operation
            SysTick_Config(72000 * SYSTICK_MS);
            NVIC_ClearPendingIRQ(SysTick_IRQn);
            NVIC_EnableIRQ(SysTick_IRQn);
            NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIORITY);
        }
        else {
            // assume 12MHz operation
            SysTick_Config(12000 * SYSTICK_MS);
            NVIC_ClearPendingIRQ(SysTick_IRQn);
            NVIC_EnableIRQ(SysTick_IRQn);
            NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIORITY);
        }
		FBRSysTicks = 0;
	}

	void SysTickStop(void) {
		SysTick->CTRL = 0x00; // Stop the SysTick timer
	}

	void SysTick_Handler(void) {
		FBRSysTicks++; // increment tick timer variable for use with Delays
		if(TickInterrupt) TickInterrupt();  // Run user-supplied interrupt function if available
        if(SDTick) SDTick();
	}

    // Systick-based delay
	void SysTickDelay(unsigned int milliseconds) {
		FBRSysTicks = 0; // zero tick timer variable
		while(FBRSysTicks < milliseconds/SYSTICK_MS); // wait until tick timer variable reaches required number
	}
#endif

// *** Watchdog timer initialise
void WDTInit(unsigned int milliseconds, unsigned char mode) {
    LPC_SYSCON->WDTOSCCTRL = 0x020;		// Default rate of divider of 2, clock rate of 0.5MHz
    LPC_SYSCON->WDTCLKSEL = 0x02;		// Select the watchdog oscillator as input
    LPC_SYSCON->WDTCLKUEN = 0x01;		// Update clock source
    LPC_SYSCON->WDTCLKUEN = 0x00;	  
    LPC_SYSCON->WDTCLKUEN = 0x01;	  
    while (!(LPC_SYSCON->WDTCLKUEN & 0x01));     // Wait for update
    LPC_SYSCON->WDTCLKDIV = 1;			// Set divider to 1
    LPC_SYSCON->PDRUNCFG &= ~(1 << 6);	// Power up the WDT oscillator

    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 15);		// Enable clock to WDT
    
    if(mode == INTERRUPT) {
        // in interrupt mode, enable interrupts
        NVIC_ClearPendingIRQ(WDT_IRQn);
        NVIC_EnableIRQ(WDT_IRQn);
        NVIC_SetPriority(WDT_IRQn, WDT_PRIORITY);
        LPC_WDT->MOD = 0x1;
    }
    else {
        // in reset mode, enable reset
        LPC_WDT->MOD = 0x3;
    }
    
    // Set up WDT period with minimum and maximum values
    if(milliseconds < 1) {
        LPC_WDT->TC = 1;
    }
    else if(milliseconds > 14400000) {
        LPC_WDT->TC = 14400000;
    }
    else {
        LPC_WDT->TC = 83 * milliseconds;
    }
    
    WDTFeed(); // feed WDT
}

// *** Watchdog timer stop
void WDTStop(void) {
    LPC_WDT->MOD &= ~0x1;
    LPC_SYSCON->WDTCLKDIV = 0;
    NVIC_DisableIRQ(WDT_IRQn);
}

// *** Watchdog timer interrupt
void WDT_IRQHandler(void) {
    if(WDTInterrupt) WDTInterrupt();
}

// *** Clockout confiig
void ClockOut(unsigned char mode, unsigned char divider) {
    if(mode == USBSOF) {
        // Set up USB start-of-frame output (1ms)
        LPC_IOCON->PIO0_1 = 0x53;
    }
    else if((mode & 0x3) == 0) {
        // Turn clockout off
        LPC_SYSCON->CLKOUTDIV = 0;
    } 
    else {
        // Otherwise one of the other modes
        mode--;
        LPC_IOCON->PIO0_1 = 0x51;
        LPC_SYSCON->CLKOUTDIV = divider;
        LPC_SYSCON->CLKOUTCLKSEL = mode;
        LPC_SYSCON->CLKOUTUEN = 0;
        LPC_SYSCON->CLKOUTUEN = 1;
    }
}


// ****************************************************************************
// *** Power mode Functions
// ****************************************************************************

// *** Sleep mode, peripherals remain active, and processor wakes up on interrupt
void Sleep(void) {
    LPC_PMU->PCON |= (1 << 11); // Clear the deep power-down power flag
    SCB->SCR &= ~(1<<2); // Deselect deep sleep/power-down mode
    __WFI(); // Sleep and wait for interrupt
}

// *** Deep sleep, chip shuts down, wakes up on GPIO or timer
void DeepSleep(unsigned long long startpins, unsigned long long startdirection, unsigned int timer) {
    unsigned int i;
    
    // *** If timer is enabled
    if(timer) {
        // *** Use T3[3]/P1[4]
        if(timer > 0x63FF9C) timer = 0x63FF9C;
        Timer2Init(9); // 10 prescale = ms
        
        if(startdirection & PORT0PIN1) {
            // User demanded a rising edge on this pin, Timer wakeup must also make use of rising edge
            Timer2Match2(timer, OUTPUTHIGH | RESET); // set match to measure in ms
        }
        else {
            // User demanded a falling edge on this pin (or did not specify), Timer wakeup to use falling edge
            Timer2Match2(timer, OUTPUTLOW | RESET); // set match to measure in ms
        }
        startpins |= PORT0PIN1;         // P0[1] is also the T2[2] pin
        
        LPC_SYSCON->PDRUNCFG &= ~(1<<6 | 1<<9 | 0x3);	// Power up the WDT oscillator
        LPC_SYSCON->PDAWAKECFG = LPC_SYSCON->PDRUNCFG;  // Set wakeup power config
        LPC_SYSCON->WDTOSCCTRL = 0x03F;		            // Lowest rate: 0.5MHz
        LPC_SYSCON->MAINCLKSEL = 0x02;     				// Select WDT as main clock source
        LPC_SYSCON->MAINCLKUEN = 0x00;               	// Update clock source
        LPC_SYSCON->MAINCLKUEN = 0x01;
        while (!(LPC_SYSCON->MAINCLKUEN & 0x01));       // Wait for clock update
        
        LPC_SYSCON->PDRUNCFG = ~((1<<6) | (1<<2) | (1<<9)); // Switch off everything unneeded
        
        if(LPC_SYSCON->BODCTRL & (1<<4)) {
            LPC_SYSCON->PDSLEEPCFG = 0x00000FB7;        // Use brown-out detect
        }
        else {
            LPC_SYSCON->PDSLEEPCFG = 0x00000FBF;        // Don't use brown-out
        }
    }
    else {
        LPC_SYSCON->PDRUNCFG &= ~(0x3);                 // Power up the IRC
        LPC_SYSCON->PDAWAKECFG = LPC_SYSCON->PDRUNCFG;  //Set wakeup power config
        LPC_SYSCON->MAINCLKSEL = 0;                     // Select IRC as main clock source
        LPC_SYSCON->MAINCLKUEN = 0;                     // Update clock source
        LPC_SYSCON->MAINCLKUEN = 1;
        while(!(LPC_SYSCON->MAINCLKUEN & 0x1));         // Wait for clock update
        
        LPC_SYSCON->PDRUNCFG = ~(0x7 | 1<<9);
        
        if(LPC_SYSCON->BODCTRL & (1<<4)) {
            LPC_SYSCON->PDSLEEPCFG = 0x00000FF7;        // Use brown-out detect
        }
        else {
            LPC_SYSCON->PDSLEEPCFG = 0x00000FFF;        // Don't use brown-out
        }
    }

    LPC_PMU->PCON |= (1<<8);    // Clear sleep flag
    
    SCB->SCR |=	(1<<2);         // Select deep sleep/power-down mode
    
    // Set wakeup pins
    LPC_SYSCON->STARTAPRP0 = startdirection & 0xffffffff;
    LPC_SYSCON->STARTRSRP0CLR = startpins & 0xffffffff;
    LPC_SYSCON->STARTERP0 = startpins & 0xffffffff;
    LPC_SYSCON->STARTAPRP1 = (startdirection >> 32) & 0xff;
    LPC_SYSCON->STARTRSRP1CLR = (startpins >> 32) & 0xff;
    LPC_SYSCON->STARTERP1 = (startpins >> 32) & 0xff;
    
    // Set wakeup interrupts
    for(i=0; i<40; i++) {
        if(startpins & (1<<i)) {
            NVIC_ClearPendingIRQ(i);
            NVIC_EnableIRQ(i);
        }
        else {
            NVIC_DisableIRQ(i);
        }
    }
    
    __WFI();    // Sleep and wait for interrupt
    
    if(timer) Timer2Stop(); // Stop timer

    ClockMode(DEFAULT_CLOCK);   // Reinstate clock
}

// *** Wakeup interrupt handler
void WAKEUP_IRQHandler(void){
	LPC_SYSCON->STARTRSRP0CLR |= LPC_SYSCON->STARTERP0; // Clear wakeup pin interrupts
	LPC_SYSCON->STARTRSRP1CLR |= LPC_SYSCON->STARTERP1;
}

// *** Deep power-down mode, only wakes up on low signal on P1[4]
void PowerDown(void) {
    #if WAKEUP_HYS
        LPC_PMU->GPREG4 | = 1 << 10;    // Enable hysteresis
    #endif
    
    LPC_PMU->PCON = (1<<1) | (1<<11);   // Select deep power-down mode, and clear flag
    SCB->SCR |= (1<<2);                 // Select deep power-down/sleep mode
    
    LPC_SYSCON->PDRUNCFG &= ~(0x3);     // Turn on IRC

    __WFI();    // Sleep
}

// *** Brownout detect
void BODInit(unsigned char interruptlevel) {
    LPC_SYSCON->BODCTRL = ((interruptlevel & 0x3) << 2) | (1<<4); // Set interrupt level
    NVIC_ClearPendingIRQ(BOD_IRQn);     // Clear and enable BOD
    NVIC_EnableIRQ(BOD_IRQn);
}

// *** Stop Brownout
void BODStop(void) {
    LPC_SYSCON->BODCTRL = 0;
    NVIC_DisableIRQ(BOD_IRQn);
}

// *** Brownout interrupt handler
void BOD_IRQHandler(void) {
    if(BODInterrupt) BODInterrupt();
}

// *** Reset status
unsigned char ResetStatus(void) {
    unsigned char status;
    status = LPC_SYSCON->SYSRESSTAT & 0x1f; // Read reset status
    LPC_SYSCON->SYSRESSTAT = status;        // Clear reset status
    if(LPC_PMU->PCON & (1<<11)) {           // Check deep power-down flag
        LPC_PMU->PCON |= (1<<11);
        status |= POWEREDDOWN;
    }
    return status;
}


// ****************************************************************************
// *** GPIO Functions
// ****************************************************************************

// *** Initialise pins as digital IO mode (note: default state is digital input)
void Port0Init(unsigned short pins) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->RESET_PIO0_0 = 0x51; // Set various pins to GPIO when specified
	if(pins & PIN1) LPC_IOCON->PIO0_1 = 0x50;
	if(pins & PIN2) LPC_IOCON->PIO0_2 = 0x50;
	if(pins & PIN3) LPC_IOCON->PIO0_3 = 0x50;
	if(pins & PIN4) LPC_IOCON->PIO0_4 = 0x00;
	if(pins & PIN5) LPC_IOCON->PIO0_5 = 0x00;
	if(pins & PIN6) LPC_IOCON->PIO0_6 = 0x50;
	if(pins & PIN7) LPC_IOCON->PIO0_7 = 0x50;
	if(pins & PIN8) LPC_IOCON->PIO0_8 = 0x50;
	if(pins & PIN9) LPC_IOCON->PIO0_9 = 0x50;
	if(pins & PIN10) LPC_IOCON->SWCLK_PIO0_10 = 0x51;
	if(pins & PIN11) LPC_IOCON->R_PIO0_11 = 0xD1;
    Port0SetIn(pins);                               // Set the specified pins as input
}

void Port1Init(unsigned short pins) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->R_PIO1_0 = 0xD1;     // Set various pins to GPIO when specified
	if(pins & PIN1) LPC_IOCON->R_PIO1_1 = 0xD1;
	if(pins & PIN2) LPC_IOCON->R_PIO1_2 = 0xD1;
	if(pins & PIN3) LPC_IOCON->SWDIO_PIO1_3 = 0xD1;
	if(pins & PIN4) LPC_IOCON->PIO1_4 = 0xD0;
	if(pins & PIN5) LPC_IOCON->PIO1_5 = 0x50;
	if(pins & PIN6) LPC_IOCON->PIO1_6 = 0x50;
	if(pins & PIN7) LPC_IOCON->PIO1_7 = 0x50;
	if(pins & PIN8) LPC_IOCON->PIO1_8 = 0x50;
	if(pins & PIN9) LPC_IOCON->PIO1_9 = 0x50;
	if(pins & PIN10) LPC_IOCON->PIO1_10 = 0xD0;
	if(pins & PIN11) LPC_IOCON->PIO1_11 = 0xD0;
    Port1SetIn(pins);                               // Set the specified pins as input
}

void Port2Init(unsigned short pins) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->PIO2_0 = 0x50;       // Set various pins to GPIO when specified
	if(pins & PIN1) LPC_IOCON->PIO2_1 = 0x50;
	if(pins & PIN2) LPC_IOCON->PIO2_2 = 0x50;
	if(pins & PIN3) LPC_IOCON->PIO2_3 = 0x50;
	if(pins & PIN4) LPC_IOCON->PIO2_4 = 0x50;
	if(pins & PIN5) LPC_IOCON->PIO2_5 = 0x50;
	if(pins & PIN6) LPC_IOCON->PIO2_6 = 0x50;
	if(pins & PIN7) LPC_IOCON->PIO2_7 = 0x50;
	if(pins & PIN8) LPC_IOCON->PIO2_8 = 0x50;
	if(pins & PIN9) LPC_IOCON->PIO2_9 = 0x50;
	if(pins & PIN10) LPC_IOCON->PIO2_10 = 0x50;
	if(pins & PIN11) LPC_IOCON->PIO2_11 = 0x50;
    Port2SetIn(pins);                               // Set the specified pins as input
}

void Port3Init(unsigned short pins) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->PIO3_0 = 0x50;       // Set various pins to GPIO when specified
	if(pins & PIN1) LPC_IOCON->PIO3_1 = 0x50;
	if(pins & PIN2) LPC_IOCON->PIO3_2 = 0x50;
	if(pins & PIN3) LPC_IOCON->PIO3_3 = 0x50;
    Port3SetIn(pins);                               // Set the specified pins as input
}

// *** Configure hysteresis on a pin (note: default stat is hysteresis disabled)
void Port0Hysteresis(unsigned short pins, unsigned char value) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->RESET_PIO0_0 = (LPC_IOCON->RESET_PIO0_0 & 0xDF) | (value << 5);  // Configure hysteresis on pin when specified
	if(pins & PIN1) LPC_IOCON->PIO0_1 = (LPC_IOCON->PIO0_1 & 0xDF) | (value << 5);
	if(pins & PIN2) LPC_IOCON->PIO0_2 = (LPC_IOCON->PIO0_2 & 0xDF) | (value << 5);
	if(pins & PIN3) LPC_IOCON->PIO0_3 = (LPC_IOCON->PIO0_3 & 0xDF) | (value << 5);
	if(pins & PIN6) LPC_IOCON->PIO0_6 = (LPC_IOCON->PIO0_6 & 0xDF) | (value << 5);
	if(pins & PIN7) LPC_IOCON->PIO0_7 = (LPC_IOCON->PIO0_7 & 0xDF) | (value << 5);
	if(pins & PIN8) LPC_IOCON->PIO0_8 = (LPC_IOCON->PIO0_8 & 0xDF) | (value << 5);
	if(pins & PIN9) LPC_IOCON->PIO0_9 = (LPC_IOCON->PIO0_9 & 0xDF) | (value << 5);
	if(pins & PIN10) LPC_IOCON->SWCLK_PIO0_10 = (LPC_IOCON->SWCLK_PIO0_10 & 0xDF) | (value << 5);
	if(pins & PIN11) LPC_IOCON->R_PIO0_11 = (LPC_IOCON->R_PIO0_11 & 0xDF) | (value << 5);
}

void Port1Hysteresis(unsigned short pins, unsigned char value) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->R_PIO1_0 = (LPC_IOCON->R_PIO1_0 & 0xDF) | (value << 5);          // Configure hysteresis on pin when specified
	if(pins & PIN1) LPC_IOCON->R_PIO1_1 = (LPC_IOCON->R_PIO1_1 & 0xDF) | (value << 5);
	if(pins & PIN2) LPC_IOCON->R_PIO1_2 = (LPC_IOCON->R_PIO1_2 & 0xDF) | (value << 5);
	if(pins & PIN3) LPC_IOCON->SWDIO_PIO1_3 = (LPC_IOCON->SWDIO_PIO1_3 & 0xDF) | (value << 5);
	if(pins & PIN4) LPC_IOCON->PIO1_4 = (LPC_IOCON->PIO1_4 & 0xDF) | (value << 5);
	if(pins & PIN5) LPC_IOCON->PIO1_5 = (LPC_IOCON->PIO1_5 & 0xDF) | (value << 5);
	if(pins & PIN6) LPC_IOCON->PIO1_6 = (LPC_IOCON->PIO1_6 & 0xDF) | (value << 5);
	if(pins & PIN7) LPC_IOCON->PIO1_7 = (LPC_IOCON->PIO1_7 & 0xDF) | (value << 5);
	if(pins & PIN8) LPC_IOCON->PIO1_8 = (LPC_IOCON->PIO1_8 & 0xDF) | (value << 5);
	if(pins & PIN9) LPC_IOCON->PIO1_9 = (LPC_IOCON->PIO1_9 & 0xDF) | (value << 5);
	if(pins & PIN10) LPC_IOCON->PIO1_10 = (LPC_IOCON->PIO1_10 & 0xDF) | (value << 5);
	if(pins & PIN11) LPC_IOCON->PIO1_11 = (LPC_IOCON->PIO1_11 & 0xDF) | (value << 5);
}

void Port2Hysteresis(unsigned short pins, unsigned char value) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->PIO2_0 = (LPC_IOCON->PIO2_0 & 0xDF) | (value << 5);              // Configure hysteresis on pin when specified
	if(pins & PIN1) LPC_IOCON->PIO2_1 = (LPC_IOCON->PIO2_1 & 0xDF) | (value << 5);
	if(pins & PIN2) LPC_IOCON->PIO2_2 = (LPC_IOCON->PIO2_2 & 0xDF) | (value << 5);
	if(pins & PIN3) LPC_IOCON->PIO2_3 = (LPC_IOCON->PIO2_3 & 0xDF) | (value << 5);
	if(pins & PIN4) LPC_IOCON->PIO2_4 = (LPC_IOCON->PIO2_4 & 0xDF) | (value << 5);
	if(pins & PIN5) LPC_IOCON->PIO2_5 = (LPC_IOCON->PIO2_5 & 0xDF) | (value << 5);
	if(pins & PIN6) LPC_IOCON->PIO2_6 = (LPC_IOCON->PIO2_6 & 0xDF) | (value << 5);
	if(pins & PIN7) LPC_IOCON->PIO2_7 = (LPC_IOCON->PIO2_7 & 0xDF) | (value << 5);
	if(pins & PIN8) LPC_IOCON->PIO2_8 = (LPC_IOCON->PIO2_8 & 0xDF) | (value << 5);
	if(pins & PIN9) LPC_IOCON->PIO2_9 = (LPC_IOCON->PIO2_9 & 0xDF) | (value << 5);
	if(pins & PIN10) LPC_IOCON->PIO2_10 = (LPC_IOCON->PIO2_10 & 0xDF) | (value << 5);
	if(pins & PIN11) LPC_IOCON->PIO2_11 = (LPC_IOCON->PIO2_11 & 0xDF) | (value << 5);
}

void Port3Hysteresis(unsigned short pins, unsigned char value) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->PIO3_0 = (LPC_IOCON->PIO3_0 & 0xDF) | (value << 5);              // Configure hysteresis on pin when specified
	if(pins & PIN1) LPC_IOCON->PIO3_1 = (LPC_IOCON->PIO3_1 & 0xDF) | (value << 5);
	if(pins & PIN2) LPC_IOCON->PIO3_2 = (LPC_IOCON->PIO3_2 & 0xDF) | (value << 5);
	if(pins & PIN3) LPC_IOCON->PIO3_3 = (LPC_IOCON->PIO3_3 & 0xDF) | (value << 5);
}

// *** Configure pull-up/-down resistors on a pin (note: default state is pull-up enabled)
void Port0Pull(unsigned short pins, unsigned char value) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->RESET_PIO0_0 = (LPC_IOCON->RESET_PIO0_0 & 0xe7) | (value << 3);  // Configure pull-up/-down on pin when specified
	if(pins & PIN1) LPC_IOCON->PIO0_1 = (LPC_IOCON->PIO0_1 & 0xe7) | (value << 3);
	if(pins & PIN2) LPC_IOCON->PIO0_2 = (LPC_IOCON->PIO0_2 & 0xe7) | (value << 3);
	if(pins & PIN3) LPC_IOCON->PIO0_3 = (LPC_IOCON->PIO0_3 & 0xe7) | (value << 3);
	if(pins & PIN6) LPC_IOCON->PIO0_6 = (LPC_IOCON->PIO0_6 & 0xe7) | (value << 3);
	if(pins & PIN7) LPC_IOCON->PIO0_7 = (LPC_IOCON->PIO0_7 & 0xe7) | (value << 3);
	if(pins & PIN8) LPC_IOCON->PIO0_8 = (LPC_IOCON->PIO0_8 & 0xe7) | (value << 3);
	if(pins & PIN9) LPC_IOCON->PIO0_9 = (LPC_IOCON->PIO0_9 & 0xe7) | (value << 3);
	if(pins & PIN10) LPC_IOCON->SWCLK_PIO0_10 = (LPC_IOCON->SWCLK_PIO0_10 & 0xe7) | (value << 3);
	if(pins & PIN11) LPC_IOCON->R_PIO0_11 = (LPC_IOCON->R_PIO0_11 & 0xe7) | (value << 3);
}

void Port1Pull(unsigned short pins, unsigned char value) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->R_PIO1_0 = (LPC_IOCON->R_PIO1_0 & 0xe7) | (value << 3);          // Configure pull-up/-down on pin when specified
	if(pins & PIN1) LPC_IOCON->R_PIO1_1 = (LPC_IOCON->R_PIO1_1 & 0xe7) | (value << 3);
	if(pins & PIN2) LPC_IOCON->R_PIO1_2 = (LPC_IOCON->R_PIO1_2 & 0xe7) | (value << 3);
	if(pins & PIN3) LPC_IOCON->SWDIO_PIO1_3 = (LPC_IOCON->SWDIO_PIO1_3 & 0xe7) | (value << 3);
	if(pins & PIN4) LPC_IOCON->PIO1_4 = (LPC_IOCON->PIO1_4 & 0xe7) | (value << 3);
	if(pins & PIN5) LPC_IOCON->PIO1_5 = (LPC_IOCON->PIO1_5 & 0xe7) | (value << 3);
	if(pins & PIN6) LPC_IOCON->PIO1_6 = (LPC_IOCON->PIO1_6 & 0xe7) | (value << 3);
	if(pins & PIN7) LPC_IOCON->PIO1_7 = (LPC_IOCON->PIO1_7 & 0xe7) | (value << 3);
	if(pins & PIN8) LPC_IOCON->PIO1_8 = (LPC_IOCON->PIO1_8 & 0xe7) | (value << 3);
	if(pins & PIN9) LPC_IOCON->PIO1_9 = (LPC_IOCON->PIO1_9 & 0xe7) | (value << 3);
	if(pins & PIN10) LPC_IOCON->PIO1_10 = (LPC_IOCON->PIO1_10 & 0xe7) | (value << 3);
	if(pins & PIN11) LPC_IOCON->PIO1_11 = (LPC_IOCON->PIO1_11 & 0xe7) | (value << 3);
}

void Port2Pull(unsigned short pins, unsigned char value) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->PIO2_0 = (LPC_IOCON->PIO2_0 & 0xe7) | (value << 3);              // Configure pull-up/-down on pin when specified
	if(pins & PIN1) LPC_IOCON->PIO2_1 = (LPC_IOCON->PIO2_1 & 0xe7) | (value << 3);
	if(pins & PIN2) LPC_IOCON->PIO2_2 = (LPC_IOCON->PIO2_2 & 0xe7) | (value << 3);
	if(pins & PIN3) LPC_IOCON->PIO2_3 = (LPC_IOCON->PIO2_3 & 0xe7) | (value << 3);
	if(pins & PIN4) LPC_IOCON->PIO2_4 = (LPC_IOCON->PIO2_4 & 0xe7) | (value << 3);
	if(pins & PIN5) LPC_IOCON->PIO2_5 = (LPC_IOCON->PIO2_5 & 0xe7) | (value << 3);
	if(pins & PIN6) LPC_IOCON->PIO2_6 = (LPC_IOCON->PIO2_6 & 0xe7) | (value << 3);
	if(pins & PIN7) LPC_IOCON->PIO2_7 = (LPC_IOCON->PIO2_7 & 0xe7) | (value << 3);
	if(pins & PIN8) LPC_IOCON->PIO2_8 = (LPC_IOCON->PIO2_8 & 0xe7) | (value << 3);
	if(pins & PIN9) LPC_IOCON->PIO2_9 = (LPC_IOCON->PIO2_9 & 0xe7) | (value << 3);
	if(pins & PIN10) LPC_IOCON->PIO2_10 = (LPC_IOCON->PIO2_10 & 0xe7) | (value << 3);
	if(pins & PIN11) LPC_IOCON->PIO2_11 = (LPC_IOCON->PIO2_11 & 0xe7) | (value << 3);
}

void Port3Pull(unsigned short pins, unsigned char value) {
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);			// Enable clock to IOCON block
	if(pins & PIN0) LPC_IOCON->PIO3_0 = (LPC_IOCON->PIO3_0 & 0xe7) | (value << 3);              // Configure pull-up/-down on pin when specified
	if(pins & PIN1) LPC_IOCON->PIO3_1 = (LPC_IOCON->PIO3_1 & 0xe7) | (value << 3);
	if(pins & PIN2) LPC_IOCON->PIO3_2 = (LPC_IOCON->PIO3_2 & 0xe7) | (value << 3);
	if(pins & PIN3) LPC_IOCON->PIO3_3 = (LPC_IOCON->PIO3_3 & 0xe7) | (value << 3);
}

// *** Configure interrupts on a pin (note: default state is interrupts disabled)
void Port0SetInterrupt(unsigned short pins, unsigned char mode) {
    // turn on and configure interrupt for specified pins if mode isn't OFF, otherwise disable
    if(mode) {
        LPC_GPIO0->IE |= pins;  // enable interrupts
        switch(mode) {
            case FALLING:
                LPC_GPIO0->IS &= ~(pins);   // edge or level sensitive setting
                LPC_GPIO0->IBE &= ~(pins);  // both or single edges setting
                LPC_GPIO0->IEV &= ~(pins);  // rising/falling edge or high/low level setting
                break;
            case RISING:
                LPC_GPIO0->IS &= ~(pins);
                LPC_GPIO0->IBE &= ~(pins);
                LPC_GPIO0->IEV |= pins;
                break;
            case BOTH:
                LPC_GPIO0->IS &= ~(pins);
                LPC_GPIO0->IBE |= pins;
                //LPC_GPIO0->IEV &= ~(pins);    // doesn't matter
                break;
            case LOW:
                LPC_GPIO0->IS |= pins;
                LPC_GPIO0->IBE &= ~(pins);
                LPC_GPIO0->IEV &= ~(pins);
                break;
            case HIGH:
                LPC_GPIO0->IS |= pins;
                LPC_GPIO0->IBE &= ~(pins);
                LPC_GPIO0->IEV |= pins;
                break;
            default:
                LPC_GPIO0->IE &= ~(pins);
        }
    }
    else {
        LPC_GPIO0->IE &= ~(pins);
    }
    
    // check if there are any interrupts enabled interrupts, or disable if there are none
    if(LPC_GPIO0->IE & 0xfff) {
        NVIC_ClearPendingIRQ(EINT0_IRQn);
        NVIC_EnableIRQ(EINT0_IRQn);
        NVIC_SetPriority(EINT0_IRQn, PORT0_PRIORITY);
    }
    else {
        NVIC_DisableIRQ(EINT0_IRQn);
    }
}

void Port1SetInterrupt(unsigned short pins, unsigned char mode) {
    // turn on and configure interrupt for specified pins if mode isn't OFF, otherwise disable
    if(mode) {
        LPC_GPIO1->IE |= pins;  // enable interrupts
        switch(mode) {
            case FALLING:
                LPC_GPIO1->IS &= ~(pins);   // edge or level sensitive setting
                LPC_GPIO1->IBE &= ~(pins);  // both or single edges setting
                LPC_GPIO1->IEV &= ~(pins);  // rising/falling edge or high/low level setting
                break;
            case RISING:
                LPC_GPIO1->IS &= ~(pins);
                LPC_GPIO1->IBE &= ~(pins);
                LPC_GPIO1->IEV |= pins;
                break;
            case BOTH:
                LPC_GPIO1->IS &= ~(pins);
                LPC_GPIO1->IBE |= pins;
                //LPC_GPIO1->IEV &= ~(pins);    // doesn't matter
                break;
            case LOW:
                LPC_GPIO1->IS |= pins;
                LPC_GPIO1->IBE &= ~(pins);
                LPC_GPIO1->IEV &= ~(pins);
                break;
            case HIGH:
                LPC_GPIO1->IS |= pins;
                LPC_GPIO1->IBE &= ~(pins);
                LPC_GPIO1->IEV |= pins;
                break;
            default:
                LPC_GPIO1->IE &= ~(pins);
        }
    }
    else {
        LPC_GPIO1->IE &= ~(pins);
    }
    
    // check if there are any interrupts enabled interrupts, or disable if there are none
    if(LPC_GPIO1->IE & 0xfff) {
        NVIC_ClearPendingIRQ(EINT1_IRQn);
        NVIC_EnableIRQ(EINT1_IRQn);
        NVIC_SetPriority(EINT1_IRQn, PORT1_PRIORITY);
    }
    else {
        NVIC_DisableIRQ(EINT1_IRQn);
    }
}

void Port2SetInterrupt(unsigned short pins, unsigned char mode) {
    // turn on and configure interrupt for specified pins if mode isn't OFF, otherwise disable
    if(mode) {
        LPC_GPIO2->IE |= pins;  // enable interrupts
        switch(mode) {
            case FALLING:
                LPC_GPIO2->IS &= ~(pins);   // edge or level sensitive setting
                LPC_GPIO2->IBE &= ~(pins);  // both or single edges setting
                LPC_GPIO2->IEV &= ~(pins);  // rising/falling edge or high/low level setting
                break;
            case RISING:
                LPC_GPIO2->IS &= ~(pins);
                LPC_GPIO2->IBE &= ~(pins);
                LPC_GPIO2->IEV |= pins;
                break;
            case BOTH:
                LPC_GPIO2->IS &= ~(pins);
                LPC_GPIO2->IBE |= pins;
                //LPC_GPIO2->IEV &= ~(pins);    // doesn't matter
                break;
            case LOW:
                LPC_GPIO2->IS |= pins;
                LPC_GPIO2->IBE &= ~(pins);
                LPC_GPIO2->IEV &= ~(pins);
                break;
            case HIGH:
                LPC_GPIO2->IS |= pins;
                LPC_GPIO2->IBE &= ~(pins);
                LPC_GPIO2->IEV |= pins;
                break;
            default:
                LPC_GPIO2->IE &= ~(pins);
        }
    }
    else {
        LPC_GPIO2->IE &= ~(pins);
    }
    
    // check if there are any interrupts enabled interrupts, or disable if there are none
    if(LPC_GPIO2->IE & 0xfff) {
        NVIC_ClearPendingIRQ(EINT2_IRQn);
        NVIC_EnableIRQ(EINT2_IRQn);
        NVIC_SetPriority(EINT2_IRQn, PORT2_PRIORITY);
    }
    else {
        NVIC_DisableIRQ(EINT2_IRQn);
    }
}

void Port3SetInterrupt(unsigned short pins, unsigned char mode) {
    // turn on and configure interrupt for specified pins if mode isn't OFF, otherwise disable
    if(mode) {
        LPC_GPIO3->IE |= pins;  // enable interrupts
        switch(mode) {
            case FALLING:
                LPC_GPIO3->IS &= ~(pins);   // edge or level sensitive setting
                LPC_GPIO3->IBE &= ~(pins);  // both or single edges setting
                LPC_GPIO3->IEV &= ~(pins);  // rising/falling edge or high/low level setting
                break;
            case RISING:
                LPC_GPIO3->IS &= ~(pins);
                LPC_GPIO3->IBE &= ~(pins);
                LPC_GPIO3->IEV |= pins;
                break;
            case BOTH:
                LPC_GPIO3->IS &= ~(pins);
                LPC_GPIO3->IBE |= pins;
                //LPC_GPIO3->IEV &= ~(pins);    // doesn't matter
                break;
            case LOW:
                LPC_GPIO3->IS |= pins;
                LPC_GPIO3->IBE &= ~(pins);
                LPC_GPIO3->IEV &= ~(pins);
                break;
            case HIGH:
                LPC_GPIO3->IS |= pins;
                LPC_GPIO3->IBE &= ~(pins);
                LPC_GPIO3->IEV |= pins;
                break;
            default:
                LPC_GPIO3->IE &= ~(pins);
        }
    }
    else {
        LPC_GPIO3->IE &= ~(pins);
    }
    
    // check if there are any interrupts enabled interrupts, or disable if there are none
    if(LPC_GPIO3->IE & 0xfff) {
        NVIC_ClearPendingIRQ(EINT3_IRQn);
        NVIC_EnableIRQ(EINT3_IRQn);
        NVIC_SetPriority(EINT3_IRQn, PORT3_PRIORITY);
    }
    else {
        NVIC_DisableIRQ(EINT3_IRQn);
    }
}

// *** Port nterrupt functions
void PIOINT0_IRQHandler(void) {
    if(Port0Interrupt) Port0Interrupt(LPC_GPIO0->MIS);  // Port-wide interrupt, passing the pins that interrupted
    if(LPC_GPIO0->MIS & 0x001 && Port0Pin0Interrupt) Port0Pin0Interrupt();  // Pin-specific interrupt
    if(LPC_GPIO0->MIS & 0x002 && Port0Pin1Interrupt) Port0Pin1Interrupt();
    if(LPC_GPIO0->MIS & 0x004 && Port0Pin2Interrupt) Port0Pin2Interrupt();
    if(LPC_GPIO0->MIS & 0x008 && Port0Pin3Interrupt) Port0Pin3Interrupt();
    if(LPC_GPIO0->MIS & 0x010 && Port0Pin4Interrupt) Port0Pin4Interrupt();
    if(LPC_GPIO0->MIS & 0x020 && Port0Pin5Interrupt) Port0Pin5Interrupt();
    if(LPC_GPIO0->MIS & 0x040 && Port0Pin6Interrupt) Port0Pin6Interrupt();
    if(LPC_GPIO0->MIS & 0x080 && Port0Pin7Interrupt) Port0Pin7Interrupt();
    if(LPC_GPIO0->MIS & 0x100 && Port0Pin8Interrupt) Port0Pin8Interrupt();
    if(LPC_GPIO0->MIS & 0x200 && Port0Pin9Interrupt) Port0Pin9Interrupt();
    if(LPC_GPIO0->MIS & 0x400 && Port0Pin10Interrupt) Port0Pin10Interrupt();
    if(LPC_GPIO0->MIS & 0x800 && Port0Pin11Interrupt) Port0Pin11Interrupt();
    LPC_GPIO0->IC = LPC_GPIO0->RIS; // Clear the interrupt
}

void PIOINT1_IRQHandler(void) {
    if(Port1Interrupt) Port1Interrupt(LPC_GPIO1->MIS);  // Port-wide interrupt, passing the pins that interrupted
    if(LPC_GPIO1->MIS & 0x001 && Port1Pin0Interrupt) Port1Pin0Interrupt();  // Pin-specific interrupt
    if(LPC_GPIO1->MIS & 0x002 && Port1Pin1Interrupt) Port1Pin1Interrupt();
    if(LPC_GPIO1->MIS & 0x004 && Port1Pin2Interrupt) Port1Pin2Interrupt();
    if(LPC_GPIO1->MIS & 0x008 && Port1Pin3Interrupt) Port1Pin3Interrupt();
    if(LPC_GPIO1->MIS & 0x010 && Port1Pin4Interrupt) Port1Pin4Interrupt();
    if(LPC_GPIO1->MIS & 0x020 && Port1Pin5Interrupt) Port1Pin5Interrupt();
    if(LPC_GPIO1->MIS & 0x040 && Port1Pin6Interrupt) Port1Pin6Interrupt();
    if(LPC_GPIO1->MIS & 0x080 && Port1Pin7Interrupt) Port1Pin7Interrupt();
    if(LPC_GPIO1->MIS & 0x100 && Port1Pin8Interrupt) Port1Pin8Interrupt();
    if(LPC_GPIO1->MIS & 0x200 && Port1Pin9Interrupt) Port1Pin9Interrupt();
    if(LPC_GPIO1->MIS & 0x400 && Port1Pin10Interrupt) Port1Pin10Interrupt();
    if(LPC_GPIO1->MIS & 0x800 && Port1Pin11Interrupt) Port1Pin11Interrupt();
    LPC_GPIO1->IC = LPC_GPIO1->RIS; // Clear the interrupt
}

void PIOINT2_IRQHandler(void) {
    if(Port2Interrupt) Port2Interrupt(LPC_GPIO2->MIS);  // Port-wide interrupt, passing the pins that interrupted
    if(LPC_GPIO2->MIS & 0x001 && Port2Pin0Interrupt) Port2Pin0Interrupt();  // Pin-specific interrupt
    if(LPC_GPIO2->MIS & 0x002 && Port2Pin1Interrupt) Port2Pin1Interrupt();
    if(LPC_GPIO2->MIS & 0x004 && Port2Pin2Interrupt) Port2Pin2Interrupt();
    if(LPC_GPIO2->MIS & 0x008 && Port2Pin3Interrupt) Port2Pin3Interrupt();
    if(LPC_GPIO2->MIS & 0x010 && Port2Pin4Interrupt) Port2Pin4Interrupt();
    if(LPC_GPIO2->MIS & 0x020 && Port2Pin5Interrupt) Port2Pin5Interrupt();
    if(LPC_GPIO2->MIS & 0x040 && Port2Pin6Interrupt) Port2Pin6Interrupt();
    if(LPC_GPIO2->MIS & 0x080 && Port2Pin7Interrupt) Port2Pin7Interrupt();
    if(LPC_GPIO2->MIS & 0x100 && Port2Pin8Interrupt) Port2Pin8Interrupt();
    if(LPC_GPIO2->MIS & 0x200 && Port2Pin9Interrupt) Port2Pin9Interrupt();
    if(LPC_GPIO2->MIS & 0x400 && Port2Pin10Interrupt) Port2Pin10Interrupt();
    if(LPC_GPIO2->MIS & 0x800 && Port2Pin11Interrupt) Port2Pin11Interrupt();
    LPC_GPIO2->IC = LPC_GPIO2->RIS; // Clear the interrupt
}

void PIOINT3_IRQHandler(void) {
    if(Port3Interrupt) Port3Interrupt(LPC_GPIO3->MIS);  // Port-wide interrupt, passing the pins that interrupted
    if(LPC_GPIO3->MIS & 0x001 && Port3Pin0Interrupt) Port3Pin0Interrupt();  // Pin-specific interrupt
    if(LPC_GPIO3->MIS & 0x002 && Port3Pin1Interrupt) Port3Pin1Interrupt();
    if(LPC_GPIO3->MIS & 0x004 && Port3Pin2Interrupt) Port3Pin2Interrupt();
    if(LPC_GPIO3->MIS & 0x008 && Port3Pin3Interrupt) Port3Pin3Interrupt();
    LPC_GPIO3->IC = LPC_GPIO3->RIS; // Clear the interrupt
}


// ****************************************************************************
// *** USB HID and MSC functions
// ****************************************************************************

#if USB_EN
    // *** ROM function location
	ROM ** FBRUSBRom = (ROM **)0x1fff1ff8;

	// *** Information constants
	// these can be changed, however ensure the size of the array does not change
    const char FBRUSBHIDString[] = {
        0x04, 3, 0x08, 0x09,					// Language Code (0x0809 is UK English)
        0x1c, 3, 'N',0,'X',0,'P',0,' ',0,'S',0,'E',0,'M',0,'I',0,'C',0,'O',0,'N',0,'D',0,' ',0,	// Manufacturer String
        0x28, 3, 'N',0,'X',0,'P',0,' ',0, 'L',0,'P',0,'C',0,'1',0,'3',0,'X',0,'X',0,' ',0,'H',0,'I',0,'D',0, ' ',0,' ',0, ' ',0, ' ',0,	// Product String
        0x1a, 3, 'F',0,'O',0,'R',0,'E',0,'B',0,'R',0,'A',0,'I',0,'N',0,	// Serial Number String
        0x0e, 3, 'H',0,'I',0,'D',0,' ',0,' ',0,' ',0,	// Interface 0, Setting 0
    };
    
    const char FBRUSBMSCString[] = {
        0x04, 3, 0x08, 0x09,					// Language Code (0x0809 is UK English)
        0x1c, 3, 'N',0,'X',0,'P',0,' ',0,'S',0,'E',0,'M',0,'I',0,'C',0,'O',0,'N',0,'D',0,' ',0,	// Manufacturer String
        0x28, 3, 'N',0,'X',0,'P',0,' ',0, 'L',0,'P',0,'C',0,'1',0,'3',0,'X',0,'X',0,' ',0,'M',0,'e',0,'m',0, 'o',0,'r',0, 'y',0, ' ',0,	// Product String
        0x1a, 3, 'F',0,'O',0,'R',0,'E',0,'B',0,'R',0,'A',0,'I',0,'N',0,	// Serial Number String
        0x0e, 3, 'M',0,'e',0,'m',0,'o',0,'r',0,'y',0,	// Interface 0, Setting 0
    };
    
    const char FBRMSCVolString[] = {
        'U','A','i','r','L','t','d',' ',        // Manufacturer
        'F','B','R','0','1',' ',' ',' ',        // Product
        'D','i','s','k',' ',' ',' ',' ',        // Description
        '1','.','0',' ',                        // Version
    };
    
    // *** Information structs
	HID_DEV_INFO FBRUSBHIDInfo;
    MSC_DEV_INFO FBRUSBMSCInfo;
    USB_DEV_INFO FBRUSBDevInfo;
    
	// *** USB HID ROM Initialisation function
	void HIDInit(unsigned char poll) {
		volatile unsigned int i;
		
        FBRUSBHIDInfo.USBVendorId = USB_VENDOR_ID;              // Vendor ID
		FBRUSBHIDInfo.USBProductId = USB_PRODUCT_ID;		    // Product ID
		FBRUSBHIDInfo.USBDeviceId = USB_DEVICE_ID; 			    // Device ID
		FBRUSBHIDInfo.USBHIDDescPtr = (unsigned int)&FBRUSBHIDString;  // Description string pointer
		FBRUSBHIDInfo.USBInReportCount = USB_IN_BYTES;	        // Bytes for in report (up to 64)
		FBRUSBHIDInfo.USBOutReportCount = USB_OUT_BYTES;	    // Bytes for out report (up to 64)
		FBRUSBHIDInfo.USBSampleInterval = poll;				    // Sample interval (in ms - up to 255, although it looks like 33ms is max)
		FBRUSBHIDInfo.USBInPtr = (unsigned int)&USBIn;		    // In Report function pointer
		FBRUSBHIDInfo.USBOutPtr = (unsigned int)&USBOut;	    // Out Report function pointer

        FBRUSBDevInfo.DevType = 0x03;
        FBRUSBDevInfo.DevDetailPtr = (unsigned int)&FBRUSBHIDInfo;
        
		LPC_SYSCON->SYSAHBCLKCTRL |= (1<<14) | (1<<10) | (1<<16);	// Enable clocks for USB_REG, Timer32B1, IOCON
        
		(*FBRUSBRom)->pUSBD->init_clk_pins();       // Initialise USB clock and pins (ROM function)
		for(i=0; i<1000; i++);					    // Brief delay for clock to stabilise
        (*FBRUSBRom)->pUSBD->init(&FBRUSBDevInfo);  // Initialise USB HID peripheral (ROM function)

		USBClockFix();  // Fix the clock!
        
        // Clear interrupts
        NVIC_ClearPendingIRQ(USB_FIQn);
        NVIC_ClearPendingIRQ(USB_IRQn);
        NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);
        
		USBConnect();							    // Connect USB (ROM function)
		for(i=0; i<1000; i++);					    // Brief wait before continuing to user code

		NVIC_SetPriority(USB_IRQn, USB_PRIORITY);   // USB interrupt is lowered slightly in priority since it occasionally interferes with serial data transfer like I2C, which is not very tolerant to being interrupted
		NVIC_SetPriority(USB_FIQn, USB_PRIORITY);
		NVIC_SetPriority(TIMER_32_1_IRQn, USB_PRIORITY);
	}
    
	void MSCInit(unsigned int size) {
		volatile unsigned int i;
        unsigned int blockcount, newsize;
        
        blockcount = size/MSC_BLOCK_SIZE;
        newsize = blockcount*MSC_BLOCK_SIZE;
        
        FBRUSBMSCInfo.USBVendorId = USB_VENDOR_ID;              // Vendor ID
		FBRUSBMSCInfo.USBProductId = USB_PRODUCT_ID;            // Product ID
		FBRUSBMSCInfo.USBDeviceId = USB_DEVICE_ID;              // Device ID
        FBRUSBMSCInfo.USBMSCDescPtr = (unsigned int)&FBRUSBMSCString;   // Description string pointer
		FBRUSBMSCInfo.MSCVolPtr = (unsigned int)&FBRMSCVolString;       // Volume string pointer
		FBRUSBMSCInfo.MSCBlockCount = blockcount;		        // Block count
		FBRUSBMSCInfo.MSCBlockSize = MSC_BLOCK_SIZE;            // Block size
		FBRUSBMSCInfo.MSCMemorySize = newsize;			        // Memory size
		FBRUSBMSCInfo.MSCWritePtr = (unsigned int)&MSCWrite;    // Write function pointer
		FBRUSBMSCInfo.MSCReadPtr = (unsigned int)&MSCRead;	    // Read function pointer

        FBRUSBDevInfo.DevType = 0x08;
        FBRUSBDevInfo.DevDetailPtr = (unsigned int)&FBRUSBMSCInfo;
        
        
		LPC_SYSCON->SYSAHBCLKCTRL |= (1<<14) | (1<<10) | (1<<16);	// Enable clocks for USB_REG, Timer32B1, IOCON
		
		(*FBRUSBRom)->pUSBD->init_clk_pins();       // Initialise USB clock and pins (ROM function)
		for(i=0; i<1000; i++);					    // Brief delay for clock to stabilise
        (*FBRUSBRom)->pUSBD->init(&FBRUSBDevInfo);  // Initialise USB HID peripheral (ROM function)
        
        USBClockFix();  // Fix the clock!
        
        // Clear interrupts
        NVIC_ClearPendingIRQ(USB_FIQn);
        NVIC_ClearPendingIRQ(USB_IRQn);
        NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);
        
		USBConnect();							    // Connect USB (ROM function)
		for(i=0; i<1000; i++);					    // Brief wait before continuing to user code

        *((unsigned int *)(0x10000054)) = 0x0;      // Initialise MSC state machine  
		NVIC_SetPriority(USB_IRQn, USB_PRIORITY);   // USB interrupt is lowered slightly in priority since it occasionally interferes with serial data transfer like I2C, which is not very tolerant to being interrupted
		NVIC_SetPriority(USB_FIQn, USB_PRIORITY);
		NVIC_SetPriority(TIMER_32_1_IRQn, USB_PRIORITY);
	}
    
    void USBClockFix(void) {
        // Bit of hax here, the ROM's built-in initialisation code sets the
		// main PLL to output the 48MHz required by USB, which leaves the
		// rest of the system running at 48MHz instead of the 72MHz we want.
		// So here we initialise the USB PLL to produce the 48MHz for the USB
		// peripheral (THAT'S WHAT IT WAS THERE FOR!), and reset the main PLL
		// back to 72MHz.
        unsigned int i;
        
        ClockMode(XTAL);
        LPC_SYSCON->PDRUNCFG &= ~(1 << 8);          	// Power up USB PLL
        for(i=0; i<1000; i++);                          // Brief delay
		LPC_SYSCON->USBPLLCLKSEL  = 0x01;   			// Select system oscillator as PLL source
		LPC_SYSCON->USBPLLCLKUEN  = 0x00;               // Update clock source
		LPC_SYSCON->USBPLLCLKUEN  = 0x01;
		while (!(LPC_SYSCON->USBPLLCLKUEN & 0x01));     // Wait for update
		LPC_SYSCON->USBPLLCTRL    = 0x03;				// Select PLL divider to 4 (12MHz - 48MHz)
		while (!(LPC_SYSCON->USBPLLSTAT & 0x01));		// Wait for PLL lock
        LPC_SYSCON->USBCLKSEL     = 0x00;               // Select PLL as main clock source
		LPC_SYSCON->USBCLKUEN = 0x00;               	// Update clock source
		LPC_SYSCON->USBCLKUEN = 0x01;
		while (!(LPC_SYSCON->USBCLKUEN & 0x01));       	// Wait for clock update
    }
    
	void USBStop(void) {
		USBDisconnect();    // Disconnect USB
		NVIC_DisableIRQ(USB_IRQn);  // Disable interrupts
		NVIC_DisableIRQ(USB_FIQn);
        NVIC_DisableIRQ(TIMER_32_1_IRQn);
		LPC_SYSCON->SYSAHBCLKCTRL &= ~((1<<14) | (1<<10));	// Disable clocks for USB and Timer32B1
	}

	void USB_IRQHandler(void) {
		(*FBRUSBRom)->pUSBD->isr(); // USB ISR (ROM function)
	}

	void USBConnect(void) {
		(*FBRUSBRom)->pUSBD->connect(1);    // USB ISR (ROM function)
	}

	void USBDisconnect(void) {
		(*FBRUSBRom)->pUSBD->connect(0);    // USB ISR (ROM function)
	}
#endif


// ****************************************************************************
// *** UART Functions
// ****************************************************************************

#if UART_EN
#if UART_GPS
#else
    unsigned char FBRUARTBuffer[UART_DATA_SIZE], FBRUARTIntChar;
    unsigned int FBRUARTIndex, FBRUARTPoint, FBRUARTOver, FBRUARTIntLength;

    void UARTInit(unsigned int baud) {
        unsigned int baudval;
        
        // Enable the pins
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16) | (1<<12);     // Enable clock to IOCON block and UART block
        LPC_IOCON->PIO1_6 = 0x11;                   // Rx
        LPC_IOCON->PIO1_7 = 0x11;                   // Tx
        
        // ...and flow control if enabled
        #if UART_FLOW
            LPC_IOCON->PIO0_7 = 0x11;               // #CTS
            LPC_IOCON->PIO1_5 = 0x11;               // #RTS
            LPC_UART->MCR |= (1<<6) | (1<<7);
        #endif
        
        // Set clock settings
        LPC_SYSCON->UARTCLKDIV = 0x01;              // Clock divider at 1
        
        LPC_UART->LCR = 0x80;                       // enable access to divisor (clock) latches
        if(baud == AUTO) {
            // In autobaud mode, wait for ascii 'A' or 'a' to be sent on the UART (don't send anything else, weirdness results
            LPC_UART->ACR = 0x003;                  // start Autobaud
            while((LPC_UART->ACR & 0x001) == 1);    // wait for Autobaud to finish
        }
        else {
            // Here are a massive list of pre-calculated fractional baud rates
            if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
                // assume 72MHz operation
                #if UART_USE_FBR
                    switch(baud) {                          // some predefined baud rates with pre-calculated fractional baud rates
                        case 110: LPC_UART->DLM = 0x92; LPC_UART->DLL = 0x7c; LPC_UART->FDR = 0xb1; break;
                        case 4800: LPC_UART->DLM = 0x03; LPC_UART->DLL = 0x6b; LPC_UART->FDR = 0xe1; break;
                        case 9600: LPC_UART->DLM = 0x01; LPC_UART->DLL = 0x77; LPC_UART->FDR = 0x41; break;
                        case 14400: LPC_UART->DLM = 0; LPC_UART->DLL = 0xfa; LPC_UART->FDR = 0x41; break;
                        case 19200: LPC_UART->DLM = 0; LPC_UART->DLL = 0x7d; LPC_UART->FDR = 0x87; break;
                        case 28800: LPC_UART->DLM = 0; LPC_UART->DLL = 0x7d; LPC_UART->FDR = 0x41; break;
                        case 38400: LPC_UART->DLM = 0; LPC_UART->DLL = 0x4a; LPC_UART->FDR = 0xc7; break;
                        case 56000: LPC_UART->DLM = 0; LPC_UART->DLL = 0x4b; LPC_UART->FDR = 0xe1; break;
                        case 57600: LPC_UART->DLM = 0; LPC_UART->DLL = 0x47; LPC_UART->FDR = 0xa1; break;
                        case 115200: LPC_UART->DLM = 0; LPC_UART->DLL = 0x17; LPC_UART->FDR = 0xa7; break;
                        case 128000: LPC_UART->DLM = 0; LPC_UART->DLL = 0x1f; LPC_UART->FDR = 0xf2; break;
                        case 153600: LPC_UART->DLM = 0; LPC_UART->DLL = 0x17; LPC_UART->FDR = 0xb3; break;
                        case 230400: LPC_UART->DLM = 0; LPC_UART->DLL = 0x10; LPC_UART->FDR = 0x92; break;
                        case 256000: LPC_UART->DLM = 0; LPC_UART->DLL = 0x10; LPC_UART->FDR = 0xa1; break;
                        case 460800: LPC_UART->DLM = 0; LPC_UART->DLL = 0x8; LPC_UART->FDR = 0x92; break;
                        case 921600: LPC_UART->DLM = 0; LPC_UART->DLL = 0x4; LPC_UART->FDR = 0x92; break;
                        case 1000000: LPC_UART->DLM = 0; LPC_UART->DLL = 0x4; LPC_UART->FDR = 0x81; break;
                        case 2000000: LPC_UART->DLM = 0; LPC_UART->DLL = 0x2; LPC_UART->FDR = 0x81; break;
                        case 3000000: LPC_UART->DLM = 0; LPC_UART->DLL = 0x1; LPC_UART->FDR = 0x21; break;
                        case 4000000: LPC_UART->DLM = 0; LPC_UART->DLL = 0x1; LPC_UART->FDR = 0x81; break;
                        default:
                            baudval = 4500000/baud;	                // baud rate
                            if(baudval > 0xffff) baudval = 0xffff;
                            else if(baudval == 0) baudval = 1;
                            
                            LPC_UART->DLM = (baudval >> 8) & 0xff;
                            LPC_UART->DLL = baudval & 0xff; 
                    }
                #else
                    baudval = 4500000/baud;	                // baud rate
                    if(baudval > 0xffff) baudval = 0xffff;
                    else if(baudval == 0) baudval = 1;
                    
                    LPC_UART->DLM = (baudval >> 8) & 0xff;
                    LPC_UART->DLL = baudval & 0xff; 
                #endif
                    
            }
            else {
                // assume 12MHz operation
                #if UART_USE_FBR
                    switch(baud) {                          // some predefined baud rates with pre-calculated fractional baud rates
                        case 110: LPC_UART->DLM = 0x18; LPC_UART->DLL = 0x6a; LPC_UART->FDR = 0xb1; break;
                        case 2400: LPC_UART->DLM = 0; LPC_UART->DLL = 0xfa; LPC_UART->FDR = 0x41; break;
                        case 4800: LPC_UART->DLM = 0; LPC_UART->DLL = 0x7d; LPC_UART->FDR = 0x41; break;
                        case 9600: LPC_UART->DLM = 0; LPC_UART->DLL = 0x47; LPC_UART->FDR = 0xa1; break;
                        case 14400: LPC_UART->DLM = 0; LPC_UART->DLL = 0x1b; LPC_UART->FDR = 0xed; break;
                        case 19200: LPC_UART->DLM = 0; LPC_UART->DLL = 0x17; LPC_UART->FDR = 0xa7; break;
                        case 28800: LPC_UART->DLM = 0; LPC_UART->DLL = 0x17; LPC_UART->FDR = 0xf2; break;
                        case 38400: LPC_UART->DLM = 0; LPC_UART->DLL = 0x10; LPC_UART->FDR = 0x92; break;
                        case 56000: LPC_UART->DLM = 0; LPC_UART->DLL = 0x7; LPC_UART->FDR = 0xcb; break;
                        case 57600: LPC_UART->DLM = 0; LPC_UART->DLL = 0xd; LPC_UART->FDR = 0x10; break;
                        case 115200: LPC_UART->DLM = 0; LPC_UART->DLL = 0x6; LPC_UART->FDR = 0xc1; break;
                        case 128000: LPC_UART->DLM = 0; LPC_UART->DLL = 0x4; LPC_UART->FDR = 0xf7; break;
                        case 153600: LPC_UART->DLM = 0; LPC_UART->DLL = 0x4; LPC_UART->FDR = 0x92; break;
                        case 230400: LPC_UART->DLM = 0; LPC_UART->DLL = 0x3; LPC_UART->FDR = 0xc1; break;
                        case 256000: LPC_UART->DLM = 0; LPC_UART->DLL = 0x2; LPC_UART->FDR = 0xf7; break;
                        case 460800: LPC_UART->DLM = 0; LPC_UART->DLL = 0x1; LPC_UART->FDR = 0x85; break;
                        default:
                            baudval = 750000/baud;	                // baud rate
                            if(baudval > 0xffff) baudval = 0xffff;
                            else if(baudval == 0) baudval = 1;
                            
                            LPC_UART->DLM = (baudval >> 8) & 0xff;
                            LPC_UART->DLL = baudval & 0xff; 
                    }
                #else
                    baudval = 750000/baud;	                // baud rate
                    if(baudval > 0xffff) baudval = 0xffff;
                    else if(baudval == 0) baudval = 1;
                    
                    LPC_UART->DLM = (baudval >> 8) & 0xff;
                    LPC_UART->DLL = baudval & 0xff; 
                #endif
            }
            
        }
        
        // Data format
        LPC_UART->LCR = ((UART_PARITY & 0x3) << 4) | ((UART_PARITY_EN & 0x1) << 3) | ((UART_STOP_BITS & 0x1) << 2) | (UART_BIT_LENGTH & 0x3);  // disable access to divisor latches, and set data format
        
        // Flus any residual data
        UARTFlush();
        
        // Set interrupt stuff
        FBRUARTIntLength = 1;
        FBRUARTIntChar = 0;
        LPC_UART->IER = 0x05;	// Enable UART receive interrupt and line interrupt
        NVIC_ClearPendingIRQ(UART_IRQn);
        NVIC_EnableIRQ(UART_IRQn);
        NVIC_SetPriority(UART_IRQn, UART_PRIORITY);
    }
    
    void UARTStop(void) {
        LPC_UART->IER = 0;
        NVIC_DisableIRQ(UART_IRQn);
        LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 12);     // Disable clock to UART block
    }

    void UARTFlush(void) {
        // Flush the software buffer
        for(FBRUARTIndex=0; FBRUARTIndex<UART_DATA_SIZE; FBRUARTIndex++) {
            FBRUARTBuffer[FBRUARTIndex] = 0;
        }
        
        FBRUARTIndex = 0;
        FBRUARTPoint = 0;
        
        // Flush the hardware FIFO
        LPC_UART->FCR = 0x00;
        LPC_UART->FCR = 0x07;                       // Reset buffer
        LPC_UART->SCR = LPC_UART->LSR;
       
        while (LPC_UART->LSR & 0x01)   LPC_UART->SCR = LPC_UART->RBR;  // Dump any data
    }
    
    void UARTWriteByte(unsigned char data) {
        //LPC_UART->IER &= ~0x001;                    // Disable receive interrupt while transmitting (causes problems in loop-back mode)
        while ((LPC_UART->LSR & 0x20) == 0);
        LPC_UART->THR = data;
        //LPC_UART->IER |= 0x001;                     // Re-enable receive interrupt
    }
    
    void UARTWrite(unsigned char data[], unsigned int length) {
        unsigned int i;
        //LPC_UART->IER &= ~0x001;                    // Disable receive interrupt while transmitting (causes problems in loop-back mode)
        for(i=0; i<length; i++) {
            while ((LPC_UART->LSR & 0x20) == 0);
            LPC_UART->THR = data[i];
        }
        //LPC_UART->IER |= 0x001;                     // Re-enable receive interrupt
    }

    unsigned char UARTReadByte(void) {
        #if UART_MODE
            // In interrupt mode, running this function triggers the user-supplied interrupt code.
            if(UARTInterrupt) {
                UARTInterrupt(FBRUARTBuffer, FBRUARTIndex);
                FBRUARTIndex = 0;
            }
            FBRUARTOver = 0;
            return 0;
        #else
            // Otherwise grab what data is available
            unsigned char temp;
            FBRUARTOver = 0;
            if(UARTAvailable()) {
                if(FBRUARTPoint >= UART_DATA_SIZE) FBRUARTPoint = 0;
                temp = FBRUARTBuffer[FBRUARTPoint++];
                return temp;
            }
            else return 0;
        #endif
    }
    
    void UART_IRQHandler(void) {
        unsigned char byte = 0;
        unsigned char iir, lsr;

        iir = (LPC_UART->IIR >> 1) & 0x07;    // interrupt identification register
        switch(iir) {
            case 0x3:   // 1 - Receive Line Status (RLS)
                lsr = LPC_UART->LSR;
                byte = LPC_UART->RBR;
                if(lsr & 0x2) {
                    if(FBRUARTOver < 0xffffffff) FBRUARTOver++; // prevent the overrun counter from overruning!
                }
                else if(lsr & 0x9c) {
                
                }
                else if(lsr & 0x01) {
                    while(LPC_UART->LSR & 0x01) {
                        #if UART_MODE
                            if(FBRUARTIndex < UART_DATA_SIZE) {
                                FBRUARTBuffer[FBRUARTIndex++] = byte;
                            }
                            else {
                                if(FBRUARTOver < 0xffffffff) FBRUARTOver++; // prevent the overrun counter from overruning!
                            }
                        #else
                            if(FBRUARTIndex >= FBRUARTPoint && ((FBRUARTPoint != 0) || (FBRUARTIndex != UART_DATA_SIZE-1))) {
                                FBRUARTBuffer[FBRUARTIndex++] = byte;
                            }
                            else if(FBRUARTIndex+1 < FBRUARTPoint) {
                                FBRUARTBuffer[FBRUARTIndex++] = byte;
                            }
                            else {
                                if(FBRUARTOver < 0xffffffff) FBRUARTOver++; // prevent the overrun counter from overruning!
                            }
                        #endif
                    }
                }
                break;
            case 0x2:   // 2a - Receive Data Available (RDA)
                while(LPC_UART->LSR & 0x01) {
                    byte = LPC_UART->RBR;
                    #if UART_MODE
                        if(FBRUARTIndex < UART_DATA_SIZE) {
                            FBRUARTBuffer[FBRUARTIndex++] = byte;
                        }
                        else {
                            if(FBRUARTOver < 0xffffffff) FBRUARTOver++; // prevent the overrun counter from overruning!
                        }
                    #else
                        if(FBRUARTIndex >= FBRUARTPoint && ((FBRUARTPoint != 0) || (FBRUARTIndex != UART_DATA_SIZE-1))) {
                            FBRUARTBuffer[FBRUARTIndex++] = byte;
                        }
                        else if(FBRUARTIndex+1 < FBRUARTPoint) {
                            FBRUARTBuffer[FBRUARTIndex++] = byte;
                        }
                        else {
                            if(FBRUARTOver < 0xffffffff) FBRUARTOver++; // prevent the overrun counter from overruning!
                        }
                    #endif
                }
                break;
            
            case 0x6:   // 2b - Character Time-out indicator (CTI)
                break;
            
            case 0x1:   // 3 - THRE interrupt
                lsr = LPC_UART->LSR;
                break;
            
            case 0x0:   // 4 - Modem interrupt
                break;
        }
      
        #if UART_MODE
            if((FBRUARTIntLength == 0 && byte == FBRUARTIntChar) || (FBRUARTIntChar == 0 && FBRUARTIndex >= FBRUARTIntLength) || (FBRUARTIntChar != 0 && FBRUARTIntLength != 0 && byte == FBRUARTIntChar && FBRUARTIndex >= FBRUARTIntLength) || FBRUARTIndex >= UART_DATA_SIZE) {
                if(UARTInterrupt) {
                    UARTInterrupt(FBRUARTBuffer, FBRUARTIndex);
                    FBRUARTIndex = 0;
                }
                FBRUARTOver = 0;
            }
        #endif
        
        if(FBRUARTIndex >= UART_DATA_SIZE) {
            FBRUARTIndex = 0;
        }
    }
#endif //UART_GPS
#endif


// ****************************************************************************
// *** I2C Functions
// ****************************************************************************

#if I2C_EN
    volatile unsigned char FBRI2CMasterState, FBRI2CMasterState2, FBRI2CSlaveState, FBRI2CSlaveState2, FBRI2CState;
    unsigned char FBRI2CBuffer[I2C_DATA_SIZE];
    volatile unsigned int FBRI2CRdLength, FBRI2CWrLength;
    volatile unsigned int FBRI2CRdIndex, FBRI2CWrIndex;

	// ******* Initialisation, set speed (in kHz)
	void I2CInit(unsigned short speed, unsigned char mode) {
        unsigned int i;
        
        for(i=0; i<I2C_DATA_SIZE; i++) {
            FBRI2CBuffer[i] = 0;
        }
        
        if(mode == SLAVE) {
            FBRI2CState = SLAVE;
        }
        else {
            FBRI2CState = MASTER;
        }
		FBRI2CMasterState=I2C_IDLE;
        FBRI2CSlaveState=I2C_IDLE;
		FBRI2CRdLength=0;
        FBRI2CWrLength=0;
		FBRI2CRdIndex=0;
        FBRI2CWrIndex=0;

		LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 5);	// Enable clock to I2C

		LPC_IOCON->PIO0_4 = 0x01;	// Set up pins PIO0_4 and PIO0_5 for I2C
		LPC_IOCON->PIO0_5 = 0x01;

		if(speed > 400) {
			// if speed is greater than 400k, set to Fast-mode Plus
			LPC_IOCON->PIO0_4 |= (1 << 9);
			LPC_IOCON->PIO0_5 |= (1 << 9);
		}
		
		LPC_SYSCON->PRESETCTRL |= (1 << 1);	// I2C reset de-asserted
		LPC_I2C->CONCLR = I2C_AA | I2C_SI | I2C_STA | I2C_ENA;	// Clear status fags

        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            // assume 72MHz operation
            LPC_I2C->SCLL = (36000 / speed) & 0xffff;	// Set speed
            LPC_I2C->SCLH = (36000 / speed) & 0xffff;

        }
        else {
            // assume 12MHz operation
            LPC_I2C->SCLL = (6000 / speed) & 0xffff;	// Set speed
            LPC_I2C->SCLH = (6000 / speed) & 0xffff;
        }
        
		if(FBRI2CState == SLAVE) {
			// set slave mode (I2C_MODE = 1)
			LPC_I2C->ADR0 = I2C_SLAVE_ADR0;	// slave mode addresses
			LPC_I2C->ADR1 = I2C_SLAVE_ADR1;
			LPC_I2C->ADR2 = I2C_SLAVE_ADR2;
			LPC_I2C->ADR3 = I2C_SLAVE_ADR3;
		}

        NVIC_ClearPendingIRQ(I2C_IRQn);	        // disable interrupt
		NVIC_EnableIRQ(I2C_IRQn);
        NVIC_SetPriority(I2C_IRQn, I2C_PRIORITY);
		LPC_I2C->CONSET = I2C_ENA | I2C_SI;
	}

	void I2CStop(void) {
		NVIC_DisableIRQ(I2C_IRQn);
		LPC_SYSCON->PRESETCTRL &= ~(1 << 1);	// I2C reset asserted
		LPC_SYSCON->SYSAHBCLKCTRL &= ~(1 << 5);	// Disable clock to I2C
	}

	// ****** The I2C Engine, does the I2C stuff
	void I2CMaster(unsigned char wrData[], unsigned int  wrLength, unsigned char rdData[], unsigned char rdLength) {
		unsigned int timeout = 0, i;
		FBRI2CMasterState = I2C_IDLE;
		FBRI2CMasterState2 = I2C_IDLE;
		FBRI2CRdIndex = 0;
		FBRI2CWrIndex = 0;
		FBRI2CRdLength = rdLength;
        FBRI2CWrLength = wrLength;
        
        if(rdLength > 0) wrLength++;
        
        for(i=0;i<wrLength; i++) {
            FBRI2CBuffer[i] = wrData[i];
        }
        
		LPC_I2C->CONSET = I2C_STA;	// set start condition
		while(1) {
			// loop until start condition transmit detected or timeout and send stop
			if (FBRI2CMasterState == I2C_STARTED) {
				while (1) {
					// once start state is transmitted, loop until NACK state then send stop
					if (FBRI2CMasterState2 == I2C_NACK){
						LPC_I2C->CONSET = I2C_STO;	// set stop condition
						LPC_I2C->CONCLR = I2C_SI;	// clear interrupt flag
                        
                        timeout = 0;
						while((LPC_I2C->CONSET & I2C_STO) && (timeout++ < I2C_TIMEOUT));	// wait until a stop condition
						break;
					}
				}
				break;	
			}
			if (timeout++ > I2C_TIMEOUT) {
				// timeout, send stop
				LPC_I2C->CONSET = I2C_STO;	// set stop condition
				LPC_I2C->CONCLR = I2C_SI;	// clear interrupt flag

				timeout = 0;
				while((LPC_I2C->CONSET & I2C_STO) && (timeout++ < I2C_TIMEOUT));	// wait until a stop condition
				break;
			}
		}
        
        for(i=0;i<rdLength; i++) {
            rdData[i] = FBRI2CBuffer[i];
        }
	}
	 
	// ****** Interrupt handler - I2C state is implemented using interrupts
	void I2C_IRQHandler(void) {
        unsigned char state;
        state = LPC_I2C->STAT & 0xff;
        
        if(FBRI2CState == MASTER) {
            switch (state) {
                case 0x08:	// A START condition has been transmitted
                    FBRI2CWrIndex = 0;
                    LPC_I2C->DAT = FBRI2CBuffer[FBRI2CWrIndex++];
                    LPC_I2C->CONCLR = I2C_STA;
                    FBRI2CMasterState = I2C_STARTED;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0x10:	// A Repeated START condition has been transmitted
                    FBRI2CRdIndex = 0;
                    LPC_I2C->DAT = FBRI2CBuffer[FBRI2CWrIndex++];
                    LPC_I2C->CONCLR = I2C_STA;
                    FBRI2CMasterState = I2C_RESTARTED;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0x18:	// SLA+W has been transmitted; ACK has been received
                    if (FBRI2CMasterState == I2C_STARTED) {
                        LPC_I2C->DAT = FBRI2CBuffer[FBRI2CWrIndex++];
                        FBRI2CMasterState = I2C_ACK;
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0x20:	// SLA+W has not been transmitted; NOT ACK has been received
                    FBRI2CMasterState = I2C_NACK;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                case 0x28:	// Data byte in I2DAT has been transmitted; ACK has been received
                    if (FBRI2CWrIndex < FBRI2CWrLength) {   
                        LPC_I2C->DAT = FBRI2CBuffer[FBRI2CWrIndex++];
                        FBRI2CMasterState = I2C_ACK;
                    }
                    else {
                        if (FBRI2CRdLength > 0) {
                            LPC_I2C->CONSET = I2C_STA;
                            FBRI2CMasterState = I2C_REPEATED_START;
                        }
                        else {
                            FBRI2CMasterState = I2C_ACK;
                            FBRI2CMasterState2 = I2C_NACK; // very very dirty hax, I2CMasterState used for ACK polling in EEPROM while I2CMasterState2 used for end of I2C operation detection!
                            LPC_I2C->CONSET = I2C_STO;
                        }
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                case 0x30:	// Data byte in I2DAT has been transmitted; NOT ACK has been received
                    FBRI2CMasterState = I2C_NACK;
                    FBRI2CMasterState2 = I2C_NACK;
                    LPC_I2C->CONSET = I2C_STO;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                case 0x38:	// Arbitration lost
                    FBRI2CMasterState = I2C_ERROR;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0x40:	// SLA+R has been trnasmitted; ACK has been received
                    if (FBRI2CRdLength == 1) {
                        LPC_I2C->CONCLR = I2C_AA;
                    }
                    else {
                        LPC_I2C->CONSET = I2C_AA;
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                case 0x48:	// SLA+R has not been transmitted; NOT ACK has been received
                    FBRI2CMasterState = I2C_NACK;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                case 0x50:	// Data byte has been receievd; ACK has been returned
                    FBRI2CBuffer[FBRI2CRdIndex++] = LPC_I2C->DAT;
                    if (FBRI2CRdIndex < FBRI2CRdLength) {   
                        FBRI2CMasterState = I2C_ACK;
                        LPC_I2C->CONSET = I2C_AA;
                    }
                    else {
                        FBRI2CMasterState = I2C_NACK;
                        LPC_I2C->CONCLR = I2C_AA;
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0x58:	// Data byte has been received; NOT ACK has been returned
                    FBRI2CBuffer[FBRI2CRdIndex++] = LPC_I2C->DAT;
                    FBRI2CMasterState = I2C_NACK;
                    FBRI2CMasterState2 = I2C_NACK;	// hax is needed (I2CMasterState changes too quickly to register in I2CEngine()
                    LPC_I2C->CONSET = I2C_STO;
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                
                default:	
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
            }
        }
        #if I2C_SLAVE_EN
        if(FBRI2CState != MASTER) {
            switch (state) {
                case 0x60:  // Own SLA+W has been received; ACK has been returned
                case 0x68:  // Arbitration lost in SLA+R/W as master; Own SLA+W has been received, ACK returned
                    FBRI2CWrIndex = 0;
                    FBRI2CRdIndex = 0;
                    LPC_I2C->CONSET = I2C_AA;
            		LPC_I2C->CONCLR = I2C_SI;
                    FBRI2CSlaveState = I2C_WR_STARTED;
                    FBRI2CSlaveState2 = I2C_WR_STARTED;
                    break;
                
                case 0x70:  // General call address (0x00) has been received; ACK has been returned
                case 0x78:  // Arbitration lost in SLA+R/W as master; General call address has been received, ACK has been returned
                    FBRI2CWrIndex = 0;
                    FBRI2CRdIndex = 0;
                    LPC_I2C->CONSET = I2C_AA;
            		LPC_I2C->CONCLR = I2C_SI;
                    FBRI2CSlaveState = I2C_GEN_STARTED;
                    FBRI2CSlaveState2 = I2C_GEN_STARTED;
                    break;
                
                case 0x80:  // Previously addressed with own SLV address; DATA has been received, ACK has been returned
                case 0x90:  // Previously addressed with General Call; DATA byte has been received; ACK has been returned
                    if (FBRI2CSlaveState == I2C_WR_STARTED) {
                        FBRI2CBuffer[FBRI2CWrIndex++] = LPC_I2C->DAT;
                        LPC_I2C->CONSET = I2C_AA;
                    }
                    else {
                        LPC_I2C->CONCLR = I2C_AA;
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;
                    
                
                case 0xA8:  // Own SLA+R has been received; ACK has been returned
                case 0xB0:  // Arbitration lost in SLA+R/W as master; Own SLA+R has been received, ACK has been returned
                    FBRI2CRdIndex = 0;
                    LPC_I2C->CONSET = I2C_AA;
            		LPC_I2C->CONCLR = I2C_SI;
                    LPC_I2C->DAT = FBRI2CBuffer[FBRI2CRdIndex++];
                    FBRI2CSlaveState = I2C_RD_STARTED;
                    FBRI2CSlaveState2 = I2C_RD_STARTED;
                    break;
                
                case 0xB8:  // Data byte in I2DAT has been transmitted; ACK has been received
                case 0xC8:  // Data byte in I2DAT has been transmitted; NOT ACK has been received.
                    if (FBRI2CSlaveState == I2C_RD_STARTED) {
                        LPC_I2C->DAT = FBRI2CBuffer[FBRI2CRdIndex++];
                        LPC_I2C->CONSET = I2C_AA;
                    }
                    else {
                        LPC_I2C->CONCLR = I2C_AA;
                    }
            		LPC_I2C->CONCLR = I2C_SI;
                    break;

                case 0xC0:  // Data byte in I2DAT has been transmitted; NOT ACK has been received
                    LPC_I2C->CONSET = I2C_AA;
            		LPC_I2C->CONCLR = I2C_SI;
                    FBRI2CSlaveState = I2C_NACK;
                break;
                
                case 0xA0:  // A STOP condition or Repeated START condition has been received while still addressed as SLV/REC or SLV/TRX
                    LPC_I2C->CONSET = I2C_AA;
                    LPC_I2C->CONCLR = I2C_SI;
                    FBRI2CSlaveState = I2C_IDLE;
                    if(I2CInterrupt) I2CInterrupt(FBRI2CBuffer, FBRI2CWrIndex);
                    break;
                
                default:
                    LPC_I2C->CONCLR = I2C_SI;
                    LPC_I2C->CONSET = I2C_ENA | I2C_SI;
                    break;
            }
        }
        #endif
	}
    
	// functions specifically for reading and writing a byte to Forebrain's EEPROM
	void EEPROMWriteByte(unsigned short address, unsigned char data) {
        unsigned int timeout = 0;
        
        FBRI2CBuffer[0] = I2C_EEPROM_ADR;
		FBRI2CBuffer[1] = (address >> 8) & 0x7f;	// ADDR high
		FBRI2CBuffer[2] = address & 0xff;	// ADDR low
		FBRI2CBuffer[3] = data;	// Data
        I2CMaster(FBRI2CBuffer, 4, 0, 0);
        
        // Poll until internal write cycle clears
        FBRI2CMasterState = I2C_NACK;
        while((FBRI2CMasterState == I2C_NACK) && (timeout++ < I2C_TIMEOUT)) {
            FBRI2CBuffer[0] = I2C_EEPROM_ADR;
            I2CMaster(FBRI2CBuffer, 1, 0, 0);
        }
	}

	unsigned char EEPROMReadByte(unsigned short address) {
		FBRI2CBuffer[0] = I2C_EEPROM_ADR;
		FBRI2CBuffer[1] = (address >> 8) & 0x7f;	// ADDR high
		FBRI2CBuffer[2] = address & 0xff;	// ADDR low
		FBRI2CBuffer[3] = I2C_EEPROM_ADR | 1;
        I2CMaster(FBRI2CBuffer, 3, FBRI2CBuffer, 1);
		return FBRI2CBuffer[0];
	}
	
	void EEPROMWrite(unsigned short address, unsigned char data[], unsigned int length) {
		unsigned int i;
		unsigned int timeout = 0;
        unsigned int datasize;
        
        if(length > I2C_DATA_SIZE-3) {
            datasize = I2C_DATA_SIZE-3;
        }
        else {
            datasize = length;
        }
        
        FBRI2CBuffer[0] = I2C_EEPROM_ADR;
		FBRI2CBuffer[1] = (address >> 8) & 0x7f;	// ADDR high
		FBRI2CBuffer[2] = address & 0xff;	// ADDR low
		for(i=0; i<datasize; i++) {
			FBRI2CBuffer[3+i] = data[i];	// Data
		}
        I2CMaster(FBRI2CBuffer, (3 + datasize), 0, 0);
        
        // Poll until internal write cycle clears
        FBRI2CMasterState = I2C_NACK;
        while((FBRI2CMasterState == I2C_NACK) && (timeout++ < I2C_TIMEOUT)) {
            FBRI2CBuffer[0] = I2C_EEPROM_ADR;
            I2CMaster(FBRI2CBuffer, 1, 0, 0);
        }
	}

	void EEPROMRead(unsigned short address, unsigned char data[], unsigned int length) {
		unsigned int i;
        unsigned int datasize;
        
        if(length > I2C_DATA_SIZE) {
            datasize = I2C_DATA_SIZE;
        }
        else {
            datasize = length;
        }
        
		FBRI2CBuffer[0] = I2C_EEPROM_ADR;
		FBRI2CBuffer[1] = (address >> 8) & 0x7f;	// ADDR high
		FBRI2CBuffer[2] = address & 0xff;	// ADDR low
		FBRI2CBuffer[3] = I2C_EEPROM_ADR | 1;
        I2CMaster(FBRI2CBuffer, 3, data, datasize);
		for(i=0; i<datasize; i++) {
			data[i] = FBRI2CBuffer[i];
		}
	}
	
	
#endif


// ****************************************************************************
// *** SSP Functions
// ****************************************************************************

#if SSP_EN
    void SSPInit(unsigned short speed, unsigned char mode) {
        unsigned char i, dummy=dummy;

        LPC_SYSCON->PRESETCTRL &= ~0x1;
        LPC_SYSCON->PRESETCTRL |= 0x1;              // deassert reset on SSP
        
        // Set up pins
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 11) | (1 << 16);
        LPC_IOCON->PIO0_8 = 0x01;
        LPC_IOCON->PIO0_9 = 0x01;

        // enable the correct SCK pin
        #if SSP_SCK == 0
            LPC_IOCON->SCKLOC = 0;
            LPC_IOCON->SWCLK_PIO0_10 = 0x02;
        #elif SSP_SCK == 1
            LPC_IOCON->SCKLOC = 1;
            LPC_IOCON->PIO2_11 = 0x02;
        #else
            LPC_IOCON->SCKLOC = 2;
            LPC_IOCON->PIO0_6 = 0x02;
        #endif
        

        #if SSP_SSEL
            LPC_IOCON->PIO0_2 = 0x01;	            // enable SSEL if used
        #endif

        // set up clock speed
        LPC_SYSCON->SSPCLKDIV = 1;
        SSPSetSpeed(speed);
    
        // SSP mode, currently not supported, set to 8-bit data frame, SPI, clock-low, transition away
        #if SSP_MODE == 0
            LPC_SSP->CR0 = 0x7;
        #endif
        

        for(i=0; i<16; i++ ) dummy = LPC_SSP->DR;   // Clear out FIFO buffer

        // set mode
        LPC_SSP->CR1 = 0;                       // Clear CR1, there are some strict transition requirements, so better to set things one at a time
        if(mode == SLAVE) LPC_SSP->CR1 = 0x4;   // Select slave mode if required (not supported)
        LPC_SSP->CR1 |= 0x2;                    // Enable SSP
    }
    
    void SSPStop(void) {
        LPC_SYSCON->SSPCLKDIV = 0;              // stop SSP clock
        LPC_SYSCON->PRESETCTRL &= ~0x1;         // assert SSP reset
    }
    
    void SSPSetSpeed(unsigned short speed) {
        unsigned int scale;
        if(speed == 0) speed = 1; // avoid divide by zero
        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            // assume 72MHz operation
            scale = (72000 + (speed/2))/speed; // 72000/speed but hax to round up using integer mathematics
        }
        else {
            // assume 12MHz operation
            scale = (12000 + (speed/2))/speed;
        }
        // work out ssp scale clock
        if(scale > 254) scale = 254;
        else if(scale < 2) scale = 2;
        LPC_SSP->CPSR = scale & 0xfe;
    }
    
    unsigned char SSPWriteByte(unsigned char data) {
        LPC_SSP->DR = data;
        while (LPC_SSP->SR & 0x10);       
        return (LPC_SSP->DR);                        
    }
    unsigned char SSPReadByte(void) {
        return SSPWriteByte(0xff); 
    }
#endif


// ****************************************************************************
// *** Timers Functions
// ****************************************************************************

void Timer0Init(unsigned short prescale) {
    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<7;
    LPC_TMR16B0->PR = prescale;
    LPC_TMR16B0->MCR = 0;
    LPC_TMR16B0->CCR = 0;
    LPC_TMR16B0->EMR = 0;
    LPC_TMR16B0->CTCR = 0;
    LPC_TMR16B0->PWMC = 0;
    LPC_TMR16B0->TCR = 1;
    NVIC_ClearPendingIRQ(TIMER_16_0_IRQn);
    NVIC_EnableIRQ(TIMER_16_0_IRQn);
	NVIC_SetPriority(TIMER_16_0_IRQn, TIMER0_PRIORITY);
}
void Timer1Init(unsigned short prescale) {
    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<8;
    LPC_TMR16B1->PR = prescale;
    LPC_TMR16B1->MCR = 0;
    LPC_TMR16B1->CCR = 0;
    LPC_TMR16B1->EMR = 0;
    LPC_TMR16B1->CTCR = 0;
    LPC_TMR16B1->PWMC = 0;
    LPC_TMR16B1->TCR = 1;
    NVIC_ClearPendingIRQ(TIMER_16_1_IRQn);
    NVIC_EnableIRQ(TIMER_16_1_IRQn);
	NVIC_SetPriority(TIMER_16_1_IRQn, TIMER1_PRIORITY);
}
void Timer2Init(unsigned int prescale) {
    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<9;
    LPC_TMR32B0->PR = prescale;
    LPC_TMR32B0->MCR = 0;
    LPC_TMR32B0->CCR = 0;
    LPC_TMR32B0->EMR = 0;
    LPC_TMR32B0->CTCR = 0;
    LPC_TMR32B0->PWMC = 0;
    LPC_TMR32B0->TCR = 1;
    NVIC_ClearPendingIRQ(TIMER_32_0_IRQn);
    NVIC_EnableIRQ(TIMER_32_0_IRQn);
	NVIC_SetPriority(TIMER_32_0_IRQn, TIMER2_PRIORITY);
}
void Timer3Init(unsigned int prescale) {
    LPC_SYSCON->SYSAHBCLKCTRL |= 1<<10;
    LPC_TMR32B1->PR = prescale;
    LPC_TMR32B1->MCR = 0;
    LPC_TMR32B1->CCR = 0;
    LPC_TMR32B1->EMR = 0;
    LPC_TMR32B1->CTCR = 0;
    LPC_TMR32B1->PWMC = 0;
    LPC_TMR32B1->TCR = 1;
    NVIC_ClearPendingIRQ(TIMER_32_1_IRQn);
    NVIC_EnableIRQ(TIMER_32_1_IRQn);
	NVIC_SetPriority(TIMER_32_1_IRQn, TIMER3_PRIORITY);
}

void Timer0Stop(void) {
    LPC_TMR16B0->TCR = 0x2;
	NVIC_DisableIRQ(TIMER_16_0_IRQn);
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<7);
    LPC_TMR16B0->TCR = 0x0;
}
void Timer1Stop(void) {
    LPC_TMR16B1->TCR = 0x2;
	NVIC_DisableIRQ(TIMER_16_1_IRQn);
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<8);
    LPC_TMR16B1->TCR = 0x0;
}
void Timer2Stop(void) {
    LPC_TMR32B0->TCR = 0x2;
	NVIC_DisableIRQ(TIMER_32_0_IRQn);
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<9);
    LPC_TMR32B0->TCR = 0x0;
}
void Timer3Stop(void) {
    LPC_TMR32B1->TCR = 0x2;
	NVIC_DisableIRQ(TIMER_32_1_IRQn);
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<10);
    LPC_TMR32B1->TCR = 0x0;
}

void Timer0Match0(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR16B0->MR0 = interval;
    LPC_TMR16B0->MCR &= ~0x7;
    LPC_TMR16B0->EMR &= ~0x30;
    LPC_TMR16B0->PWMC &= ~0x1;
    LPC_TMR16B0->MCR |= (mode & 0x7);
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR16B0->EMR |= (mode & 0x30);
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO0_8 = 0x02;
        if(mode & OUTPUTLOW) LPC_TMR16B0->EMR |= 1;
        else if(mode & OUTPUTHIGH) LPC_TMR16B0->EMR &= ~1;
    }
    if(mode & PWM) LPC_TMR16B0->PWMC |= 0x1;
}
void Timer0Match1(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR16B0->MR1 = interval;
    LPC_TMR16B0->MCR &= ~(0x7 << 3);
    LPC_TMR16B0->EMR &= ~(0x30 << 2);
    LPC_TMR16B0->PWMC &= ~(0x1 << 1);
    LPC_TMR16B0->MCR |= (mode & 0x7) << 3;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR16B0->EMR |= (mode & 0x30) << 2;
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO0_9 = 0x02;
        if(mode & OUTPUTLOW) LPC_TMR16B0->EMR |= 1<<1;
        else if(mode & OUTPUTHIGH) LPC_TMR16B0->EMR &= ~(1<<1);
    }
    if(mode & PWM) LPC_TMR16B0->PWMC |= 0x1 << 1;
}
void Timer0Match2(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR16B0->MR2 = interval;
    LPC_TMR16B0->MCR &= ~(0x7 << 6);
    LPC_TMR16B0->EMR &= ~(0x30 << 4);
    LPC_TMR16B0->PWMC &= ~(0x1 << 2);
    LPC_TMR16B0->MCR |= (mode & 0x7) << 6;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR16B0->EMR |= (mode & 0x30) << 4;
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->SWCLK_PIO0_10 = 0x03;
        if(mode & OUTPUTLOW) LPC_TMR16B0->EMR |= 1<<2;
        else if(mode & OUTPUTHIGH) LPC_TMR16B0->EMR &= ~(1<<2);
    }
    if(mode & PWM) LPC_TMR16B0->PWMC |= 0x1 << 2;
}
void Timer0Match3(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR16B0->MR3 = interval;
    LPC_TMR16B0->MCR &= ~(0x7 << 9);
    LPC_TMR16B0->EMR &= ~(0x30 << 6);
    LPC_TMR16B0->PWMC &= ~(0x1 << 3);;
    LPC_TMR16B0->MCR |= (mode & 0x7) << 9;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR16B0->EMR |= (mode & 0x30) << 6;
        if(mode & OUTPUTLOW) LPC_TMR16B0->EMR |= 1<<3;
        else if(mode & OUTPUTHIGH) LPC_TMR16B0->EMR &= ~(1<<3);
    }
    if(mode & PWM) LPC_TMR16B0->PWMC |= 0x1 << 3;
}
void Timer0Capture(unsigned char mode) {
    if(mode) {
        if(mode & COUNTER) {
            LPC_TMR16B0->CTCR = mode & 0x3;
            LPC_TMR16B0->TC = 0;
        }
        else {
            LPC_TMR16B0->CTCR = 0;
            if(mode & INTERRUPT) mode |= 0x4;
            LPC_TMR16B0->CCR = mode & 0x7;
        }
    
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO0_2 &= ~(0x07);
        LPC_IOCON->PIO0_2 |= 0x02;
    }
}
void Timer1Match0(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR16B1->MR0 = interval;
    LPC_TMR16B1->MCR &= ~0x7;
    LPC_TMR16B1->EMR &= ~0x30;
    LPC_TMR16B1->PWMC &= ~0x1;
    LPC_TMR16B1->MCR |= (mode & 0x7);
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR16B1->EMR |= (mode & 0x30);
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO1_9 = 0x01;
        if(mode & OUTPUTLOW) LPC_TMR16B1->EMR |= 1;
        else if(mode & OUTPUTHIGH) LPC_TMR16B1->EMR &= ~1;
    }
    if(mode & PWM) LPC_TMR16B1->PWMC |= 0x1;
}
void Timer1Match1(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR16B1->MR1 = interval;
    LPC_TMR16B1->MCR &= ~(0x7 << 3);
    LPC_TMR16B1->EMR &= ~(0x30 << 2);
    LPC_TMR16B1->PWMC &= ~(0x1 << 1);
    LPC_TMR16B1->MCR |= (mode & 0x7) << 3;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR16B1->EMR |= (mode & 0x30) << 2;
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO1_10 = 0x02;
        if(mode & OUTPUTLOW) LPC_TMR16B1->EMR |= 1<<1;
        else if(mode & OUTPUTHIGH) LPC_TMR16B1->EMR &= ~(1<<1);
    }
    if(mode & PWM) LPC_TMR16B1->PWMC |= 0x1 << 1;
}
void Timer1Match2(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR16B1->MR2 = interval;
    LPC_TMR16B1->MCR &= ~(0x7 << 6);
    LPC_TMR16B1->EMR &= ~(0x30 << 4);
    LPC_TMR16B1->PWMC &= ~(0x1 << 2);
    LPC_TMR16B1->MCR |= (mode & 0x7) << 6;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR16B1->EMR |= (mode & 0x30) << 4;
        if(mode & OUTPUTLOW) LPC_TMR16B1->EMR |= 1<<2;
        else if(mode & OUTPUTHIGH) LPC_TMR16B1->EMR &= ~(1<<2);
    }
    if(mode & PWM) LPC_TMR16B1->PWMC |= 0x1 << 2;
}
void Timer1Match3(unsigned short interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR16B1->MR3 = interval;
    LPC_TMR16B1->MCR &= ~(0x7 << 9);
    LPC_TMR16B1->EMR &= ~(0x30 << 6);
    LPC_TMR16B1->PWMC &= ~(0x1 << 3);;
    LPC_TMR16B1->MCR |= (mode & 0x7) << 9;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR16B1->EMR |= (mode & 0x30) << 6;
        if(mode & OUTPUTLOW) LPC_TMR16B1->EMR |= 1<<3;
        else if(mode & OUTPUTHIGH) LPC_TMR16B1->EMR &= ~(1<<3);
    }
    if(mode & PWM) LPC_TMR16B1->PWMC |= 0x1 << 3;
}
void Timer1Capture(unsigned char mode) {
    if(mode) {        
        if(mode & COUNTER) {
            LPC_TMR16B1->CTCR = mode & 0x3;
            LPC_TMR16B1->TC = 0;
        }
        else {
            LPC_TMR16B1->CTCR = 0;
            if(mode & INTERRUPT) mode |= 0x4;
            LPC_TMR16B1->CCR = mode & 0x7;
        }
    
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO1_8 &= ~(0x07);
        LPC_IOCON->PIO1_8 |= 0x01;
    }
}
void Timer2Match0(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR32B0->MR0 = interval;
    LPC_TMR32B0->MCR &= ~0x7;
    LPC_TMR32B0->EMR &= ~0x30;
    LPC_TMR32B0->PWMC &= ~0x1;
    LPC_TMR32B0->MCR |= (mode & 0x7);
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR32B0->EMR |= (mode & 0x30);
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO1_6 = 0x02;
        if(mode & OUTPUTLOW) LPC_TMR32B0->EMR |= 1;
        else if(mode & OUTPUTHIGH) LPC_TMR32B0->EMR &= ~1;
    }
    if(mode & PWM) LPC_TMR32B0->PWMC |= 0x1;
}
void Timer2Match1(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR32B0->MR1 = interval;
    LPC_TMR32B0->MCR &= ~(0x7 << 3);
    LPC_TMR32B0->EMR &= ~(0x30 << 2);
    LPC_TMR32B0->PWMC &= ~(0x1 << 1);
    LPC_TMR32B0->MCR |= (mode & 0x7) << 3;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR32B0->EMR |= (mode & 0x30) << 2;
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO1_7 = 0x02;
        if(mode & OUTPUTLOW) LPC_TMR32B0->EMR |= 1<<1;
        else if(mode & OUTPUTHIGH) LPC_TMR32B0->EMR &= ~(1<<1);
    }
    if(mode & PWM) LPC_TMR32B0->PWMC |= 0x1 << 1;
}
void Timer2Match2(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR32B0->MR2 = interval;
    LPC_TMR32B0->MCR &= ~(0x7 << 6);
    LPC_TMR32B0->EMR &= ~(0x30 << 4);
    LPC_TMR32B0->PWMC &= ~(0x1 << 2);
    LPC_TMR32B0->MCR |= (mode & 0x7) << 6;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR32B0->EMR |= (mode & 0x30) << 4;
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO0_1 = 0x02;
        if(mode & OUTPUTLOW) LPC_TMR32B0->EMR |= 1<<2;
        else if(mode & OUTPUTHIGH) LPC_TMR32B0->EMR &= ~(1<<2);
    }
    if(mode & PWM) LPC_TMR32B0->PWMC |= 0x1 << 2;
}
void Timer2Match3(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR32B0->MR3 = interval;
    LPC_TMR32B0->MCR &= ~(0x7 << 9);
    LPC_TMR32B0->EMR &= ~(0x30 << 6);
    LPC_TMR32B0->PWMC &= ~(0x1 << 3);
    LPC_TMR32B0->MCR |= (mode & 0x7) << 9;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR32B0->EMR |= (mode & 0x30) << 6;
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->R_PIO0_11 = 0x03;
        if(mode & OUTPUTLOW) LPC_TMR32B0->EMR |= 1<<3;
        else if(mode & OUTPUTHIGH) LPC_TMR32B0->EMR &= ~(1<<3);
    }
    if(mode & PWM) LPC_TMR32B0->PWMC |= 0x1 << 3;
}
void Timer2Capture(unsigned char mode) {
    if(mode) {
        if(mode & COUNTER) {
            LPC_TMR32B0->CTCR = mode & 0x3;
            LPC_TMR32B0->TC = 0;
        }
        else {
            LPC_TMR32B0->CTCR = 0;
            if(mode & INTERRUPT) mode |= 0x4;
            LPC_TMR32B0->CCR = mode & 0x7;
        }
    
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO1_5 &= ~(0x07);
        LPC_IOCON->PIO1_5 |= 0x02;
    }
}
void Timer3Match0(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR32B1->MR0 = interval;
    LPC_TMR32B1->MCR &= ~0x7;
    LPC_TMR32B1->EMR &= ~0x30;
    LPC_TMR32B1->PWMC &= ~0x1;
    LPC_TMR32B1->MCR |= (mode & 0x7);
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR32B1->EMR |= (mode & 0x30);
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->R_PIO1_1 = 0x03;
        if(mode & OUTPUTLOW) LPC_TMR32B1->EMR |= 1;
        else if(mode & OUTPUTHIGH) LPC_TMR32B1->EMR &= ~1;
    }
    if(mode & PWM) LPC_TMR32B1->PWMC |= 0x1;
}
void Timer3Match1(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR32B1->MR1 = interval;
    LPC_TMR32B1->MCR &= ~(0x7 << 3);
    LPC_TMR32B1->EMR &= ~(0x30 << 2);
    LPC_TMR32B1->PWMC &= ~(0x1 << 1);
    LPC_TMR32B1->MCR |= (mode & 0x7) << 3;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR32B1->EMR |= (mode & 0x30) << 2;
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->R_PIO1_2 = 0x03;
        if(mode & OUTPUTLOW) LPC_TMR32B1->EMR |= 1<<1;
        else if(mode & OUTPUTHIGH) LPC_TMR32B1->EMR &= ~(1<<1);
    }
    if(mode & PWM) LPC_TMR32B1->PWMC |= 0x1 << 1;
}
void Timer3Match2(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR32B1->MR2 = interval;
    LPC_TMR32B1->MCR &= ~(0x7 << 6);
    LPC_TMR32B1->EMR &= ~(0x30 << 4);
    LPC_TMR32B1->PWMC &= ~(0x1 << 2);
    LPC_TMR32B1->MCR |= (mode & 0x7) << 6;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR32B1->EMR |= (mode & 0x30) << 4;
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->SWDIO_PIO1_3 = 0x03;
        if(mode & OUTPUTLOW) LPC_TMR32B1->EMR |= 1<<2;
        else if(mode & OUTPUTHIGH) LPC_TMR32B1->EMR &= ~(1<<2);
    }
    if(mode & PWM) LPC_TMR32B1->PWMC |= 0x1 << 2;
}
void Timer3Match3(unsigned int interval, unsigned char mode) {
    if(mode & INTERRUPT) mode |= 0x1;
    LPC_TMR32B1->MR3 = interval;
    LPC_TMR32B1->MCR &= ~(0x7 << 9);
    LPC_TMR32B1->EMR &= ~(0x30 << 6);
    LPC_TMR32B1->PWMC &= ~(0x1 << 3);;
    LPC_TMR32B1->MCR |= (mode & 0x7) << 9;
    if(mode & OUTPUTTOGGLE) {
        LPC_TMR32B1->EMR |= (mode & 0x30) << 6;
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->PIO1_4 = 0x02;
        if(mode & OUTPUTLOW) LPC_TMR32B1->EMR |= 1<<3;
        else if(mode & OUTPUTHIGH) LPC_TMR32B1->EMR &= ~(1<<3);
    }
    if(mode & PWM) LPC_TMR32B1->PWMC |= 0x1 << 3;
}
void Timer3Capture(unsigned char mode) {
    if(mode) {
        if(mode & COUNTER) {
            LPC_TMR32B1->CTCR = mode & 0x3;
            LPC_TMR32B1->TC = 0;
        }
        else {
            LPC_TMR32B1->CTCR = 0;
            if(mode & INTERRUPT) mode |= 0x4;
            LPC_TMR32B1->CCR = mode & 0x7;
        }
    
        LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16);
        LPC_IOCON->R_PIO1_0 &= ~(0x07);
        LPC_IOCON->R_PIO1_0 |= 0x03;
    }
}
void TIMER16_0_IRQHandler(void) {
    if(Timer0Interrupt) Timer0Interrupt(LPC_TMR16B0->IR & 0x1f);
    if(LPC_TMR16B0->IR & 0x1 && Timer0Interrupt0) Timer0Interrupt0();
    if(LPC_TMR16B0->IR & 0x2 && Timer0Interrupt1) Timer0Interrupt1();
    if(LPC_TMR16B0->IR & 0x4 && Timer0Interrupt2) Timer0Interrupt2();
    if(LPC_TMR16B0->IR & 0x8 && Timer0Interrupt3) Timer0Interrupt3();
    if(LPC_TMR16B0->IR & 0x10 && Timer0InterruptC) Timer0InterruptC(LPC_TMR16B0->CR0);
    LPC_TMR16B0->IR = 0x1f;
    __NOP(); __NOP();
}
void TIMER16_1_IRQHandler(void) {
    if(Timer1Interrupt) Timer1Interrupt(LPC_TMR16B1->IR & 0x1f);
    if(LPC_TMR16B1->IR & 0x1 && Timer1Interrupt0) Timer1Interrupt0();
    if(LPC_TMR16B1->IR & 0x2 && Timer1Interrupt1) Timer1Interrupt1();
    if(LPC_TMR16B1->IR & 0x4 && Timer1Interrupt2) Timer1Interrupt2();
    if(LPC_TMR16B1->IR & 0x8 && Timer1Interrupt3) Timer1Interrupt3();
    if(LPC_TMR16B1->IR & 0x10 && Timer1InterruptC) Timer1InterruptC(LPC_TMR16B1->CR0);
    LPC_TMR16B1->IR = 0x1f;
    __NOP(); __NOP();
}
void TIMER32_0_IRQHandler(void) {
    if(Timer2Interrupt) Timer2Interrupt(LPC_TMR32B0->IR & 0x1f);
    if(LPC_TMR32B0->IR & 0x1 && Timer2Interrupt0) Timer2Interrupt0();
    if(LPC_TMR32B0->IR & 0x2 && Timer2Interrupt1) Timer2Interrupt1();
    if(LPC_TMR32B0->IR & 0x4 && Timer2Interrupt2) Timer2Interrupt2();
    if(LPC_TMR32B0->IR & 0x8 && Timer2Interrupt3) Timer2Interrupt3();
    if(LPC_TMR32B0->IR & 0x10 && Timer2InterruptC) Timer2InterruptC(LPC_TMR32B0->CR0);
    LPC_TMR32B0->IR = 0x1f;
    __NOP(); __NOP();
}
void TIMER32_1_IRQHandler(void) {
    if(Timer3Interrupt) Timer3Interrupt(LPC_TMR32B1->IR & 0x1f);
    if(LPC_TMR32B1->IR & 0x1 && Timer3Interrupt0) Timer3Interrupt0();
    if(LPC_TMR32B1->IR & 0x2 && Timer3Interrupt1) Timer3Interrupt1();
    if(LPC_TMR32B1->IR & 0x4 && Timer3Interrupt2) Timer3Interrupt2();
    if(LPC_TMR32B1->IR & 0x8 && Timer3Interrupt3) Timer3Interrupt3();
    if(LPC_TMR32B1->IR & 0x10 && Timer3InterruptC) Timer3InterruptC(LPC_TMR32B1->CR0);
    LPC_TMR32B1->IR = 0x1f;
    __NOP(); __NOP();
}

// ****************************************************************************
// *** ADC Functions
// ****************************************************************************

// *** Initialise the ADC
void ADCInit(unsigned short channels) {
	if(channels) {
		LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 16); // Enable IOCONFIG
		if(channels & CHN0) LPC_IOCON->R_PIO0_11 = 0x02;            // Sets AD0 to Analogue mode if specified
		if(channels & CHN1) LPC_IOCON->R_PIO1_0 = 0x02;             // Sets AD1 to Analogue mode if specified
		if(channels & CHN2) LPC_IOCON->R_PIO1_1 = 0x02;             // Sets AD2 to Analogue mode if specified
		if(channels & CHN3) LPC_IOCON->R_PIO1_2 = 0x02;             // Sets AD3 to Analogue mode if specified
		if(channels & CHN4) LPC_IOCON->SWDIO_PIO1_3 = 0x02;         // Sets AD4 to Analogue mode if specified
		if(channels & CHN5) LPC_IOCON->PIO1_4 = 0x01;               // Sets AD5 to Analogue mode if specified
		if(channels & CHN6) LPC_IOCON->PIO1_10 = 0x01;              // Sets AD6 to Analogue mode if specified
		if(channels & CHN7) LPC_IOCON->PIO1_11 = 0x01;              // Sets AD7 to Analogue mode if specified
		
		LPC_SYSCON->SYSAHBCLKCTRL |= (1<<13);	// Enable clock to the ADC peripheral
		LPC_SYSCON->PDRUNCFG &= ~(1<<4);		// Enable power to the ADC peripheral
		
        // Set sample rates depending on clock speed, attempt to get as close to the 4.5MHz maximum ADC clock rate as possible
        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            // assume 72MHz operation
            LPC_ADC->CR = 0x0F00;               // Sample at 4.5MHz clock, at 10bits
        }
        else {
            // assume 12MHz operation
            LPC_ADC->CR = 0x0200;               // Sample at 4.0MHz clock, at 10bits
        }
        #if ADC_MODE
			// in interrupt mode, set up burst mode and global interrupts
            LPC_ADC->CR |= 0x010000 | channels;						// Burst mode
			LPC_ADC->INTEN = 0x100;									// Enable global interrupt
			
            NVIC_ClearPendingIRQ(ADC_IRQn);
            NVIC_EnableIRQ(ADC_IRQn);
            NVIC_SetPriority(ADC_IRQn, ADC_PRIORITY);
		#else
            // in on-demand mode, disable burst mode and interrupts
            LPC_ADC->INTEN = 0x000;									// Disable global interrupt
		#endif
	}
}

// *** Stop the ADC
void ADCStop(void) {
	NVIC_DisableIRQ(ADC_IRQn);
	LPC_ADC->CR = 0x000F00;					// Disable burst mode
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<13);	// Disable clock to the ADC peripheral
	LPC_SYSCON->PDRUNCFG |= (1<<4);		    // Disable power to the ADC peripheral
}

// *** Read ADC value
unsigned short ADCRead(unsigned char channel) {
	#if ADC_MODE
        // in interrupt mode, select channel and return last data value for that channel
		if(channel & CHN0) return ((LPC_ADC->DR[0] >> 6) & 0x3FF);
		else if(channel & CHN1) return ((LPC_ADC->DR[1] >> 6) & 0x3FF);
		else if(channel & CHN2) return ((LPC_ADC->DR[2] >> 6) & 0x3FF);
		else if(channel & CHN3) return ((LPC_ADC->DR[3] >> 6) & 0x3FF);
		else if(channel & CHN4) return ((LPC_ADC->DR[4] >> 6) & 0x3FF);
		else if(channel & CHN5) return ((LPC_ADC->DR[5] >> 6) & 0x3FF);
		else if(channel & CHN6) return ((LPC_ADC->DR[6] >> 6) & 0x3FF);
		else if(channel & CHN7) return ((LPC_ADC->DR[7] >> 6) & 0x3FF);
		else return 0xffff;
	#else
        // in on-demand mode, initialise a reading
        if((LPC_SYSCON->MAINCLKSEL & 0x03) == 0x03) {
            // assume 72MHz operation
            LPC_ADC->CR = 0x1000F00 | channel;      // Start now!
        }
        else {
            // assume 12MHz operation
            LPC_ADC->CR = 0x1000200 | channel;      // Start now!
        }
		while(!(LPC_ADC->GDR & (1 << 31)));			// Wait for ADC to complete
		return ((LPC_ADC->GDR >> 6) & 0x3FF);       // Return the ADC value
	#endif
}

// *** ADC interrupt handler
#if ADC_MODE
	void ADC_IRQHandler(void) {
		unsigned int gdr;
		gdr = LPC_ADC->GDR;
		if(ADCInterrupt) ADCInterrupt(((gdr >> 24) & 0x7), ((gdr >> 6) & 0x3FF)); // If user-supplied handler is available, run it, passing data and channel
	}
#endif
