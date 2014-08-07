// ****************************************************************************
// *** Copyright (c) 2011, Universal Air Ltd. All rights reserved.
// *** Source and binaries are released under the BSD 3-Clause license
// *** See readme_forebrain.txt files for the text of the license
// ****************************************************************************
 
#ifndef __UAFUNC_H__
#define __UAFUNC_H__
#include "config.h"
#include "LPC13xx.h"

#define WEAK_ALIAS(f) __attribute__ ((weak, alias (#f)));
#define WEAK __attribute__ ((weak))
#define ALIAS(f) __attribute__ ((alias (#f)));

#ifndef TRUE
    #define TRUE        1
    #define FALSE       0
#endif

#ifndef NULL
    #ifdef __cplusplus              // EC++
        #define NULL          0
    #else
        #define NULL          ((void *) 0)
    #endif
#endif

#define ON            1
#define OFF            0


// ****************************************************************************
// *** Misc Functions
// ****************************************************************************

// *** In-application programming functions
// Note: currently only Reprogram() available, but other IAP commands can be supproted
#if IAP_EN
    typedef void (*FBRIAP)(unsigned int [], unsigned int []);
    extern unsigned int FBRIAPCommand[5], FBRIAPResult[4];
    void Reprogram(void);
#endif

// *** Reset functions
void Reset(void);
static inline void ResetInit(void) { LPC_IOCON->RESET_PIO0_0 = 0x50; }

// *** Byte handling functions
static inline unsigned char LowByte(unsigned short value)   { return (value & 0xff); }
static inline unsigned char HighByte(unsigned short value)  { return (value >> 8) & 0xff; }
static inline unsigned char FirstByte(unsigned int value)   { return (value & 0xff); }
static inline unsigned char SecondByte(unsigned int value)  { return (value >> 8) & 0xff; }
static inline unsigned char ThirdByte(unsigned int value)   { return (value >> 16) & 0xff; }
static inline unsigned char FourthByte(unsigned int value)  { return (value >> 24) & 0xff; }
static inline unsigned short MakeShort(unsigned char byte2, unsigned char byte1)  { return ((byte2 << 8) & byte1); }
static inline unsigned int MakeInt(unsigned char byte4, unsigned char byte3, unsigned char byte2, unsigned char byte1) { return ((byte4 << 24) | (byte3 << 16) | (byte2 << 8) | byte1); }

// *** Random number functions
extern unsigned int FBRRandomNumber;
static inline unsigned int Random(void) { FBRRandomNumber = FBRRandomNumber * RAND_A + RAND_C; return FBRRandomNumber; };
static inline void RandomSeed(unsigned int seed) { FBRRandomNumber = seed; };


// ****************************************************************************
// *** Clock Functions
// ****************************************************************************

// *** Clock mode functions
#define XTAL    0
#define IRC72   1
#define IRC12   2
void ClockMode(unsigned char mode);

// *** Delay functions
void Delay(unsigned int milliseconds);
void WaitDelay(unsigned int milliseconds);

// *** System tick timer functions
#if SYSTICK_EN
    void SysTickInit(void);
    void SysTickStop(void);
    volatile unsigned int FBRSysTicks, FBRSysTickDelayTicks;

    extern WEAK void TickInterrupt(void);
    extern WEAK void SDTick(void);
    void SysTickDelay(unsigned int milliseconds);
#endif

// *** Watchdog timer functions
#define INTERRUPT   0x40
#define RESET       0x2

void WDTInit(unsigned int milliseconds, unsigned char mode);
void WDTStop(void);
static inline void WDTFeed(void) {    LPC_WDT->FEED = 0xAA; LPC_WDT->FEED = 0x55;    }
extern WEAK void WDTInterrupt(void);

// *** Clockout functions
//#define OFF        0  // this defined as such elsewhere
#define IRCOSC      1
#define SYSOSC      2
#define WDTOSC      3
#define MAINCLK     4
#define USBSOF      5
void ClockOut(unsigned char mode, unsigned char divider);


// ****************************************************************************
// *** Power mode Functions
// ****************************************************************************

// *** Sleep mode
void Sleep(void);

// *** Deep sleep
#define PORT0PIN0   0x0000000000000001ULL
#define PORT0PIN1   0x0000000000000002ULL
#define PORT0PIN2   0x0000000000000004ULL
#define PORT0PIN3   0x0000000000000008ULL
#define PORT0PIN4   0x0000000000000010ULL
#define PORT0PIN5   0x0000000000000020ULL
#define PORT0PIN6   0x0000000000000040ULL
#define PORT0PIN7   0x0000000000000080ULL
#define PORT0PIN8   0x0000000000000100ULL
#define PORT0PIN9   0x0000000000000200ULL
#define PORT0PIN10  0x0000000000000400ULL
#define PORT0PIN11  0x0000000000000800ULL
#define PORT1PIN0   0x0000000000001000ULL
#define PORT1PIN1   0x0000000000002000ULL
#define PORT1PIN2   0x0000000000004000ULL
#define PORT1PIN3   0x0000000000008000ULL
#define PORT1PIN4   0x0000000000010000ULL
#define PORT1PIN5   0x0000000000020000ULL
#define PORT1PIN6   0x0000000000040000ULL
#define PORT1PIN7   0x0000000000080000ULL
#define PORT1PIN8   0x0000000000100000ULL
#define PORT1PIN9   0x0000000000200000ULL
#define PORT1PIN10  0x0000000000400000ULL
#define PORT1PIN11  0x0000000000800000ULL
#define PORT2PIN0   0x0000000001000000ULL
#define PORT2PIN1   0x0000000002000000ULL
#define PORT2PIN2   0x0000000004000000ULL
#define PORT2PIN3   0x0000000008000000ULL
#define PORT2PIN4   0x0000000010000000ULL
#define PORT2PIN5   0x0000000020000000ULL
#define PORT2PIN6   0x0000000040000000ULL
#define PORT2PIN7   0x0000000080000000ULL
#define PORT2PIN8   0x0000000100000000ULL
#define PORT2PIN9   0x0000000200000000ULL
#define PORT2PIN10  0x0000000400000000ULL
#define PORT2PIN11  0x0000000800000000ULL
#define PORT3PIN0   0x0000001000000000ULL
#define PORT3PIN1   0x0000002000000000ULL
#define PORT3PIN2   0x0000004000000000ULL
#define PORT3PIN3   0x0000008000000000ULL

void DeepSleep(unsigned long long startpins, unsigned long long startdirection, unsigned int timer);

// *** Deep power down mode
void PowerDown(void);
static inline void WriteBootData0(unsigned int data) { LPC_PMU->GPREG0 = data; }
static inline void WriteBootData1(unsigned int data) { LPC_PMU->GPREG1 = data; }
static inline void WriteBootData2(unsigned int data) { LPC_PMU->GPREG2 = data; }
static inline void WriteBootData3(unsigned int data) { LPC_PMU->GPREG3 = data; }
static inline void WriteBootData4(unsigned int data) { LPC_PMU->GPREG4 &= ~(0xfffff << 11); LPC_PMU->GPREG4 |= ((data & 0xfffff) << 11); }
static inline unsigned int ReadBootData0(void) { return LPC_PMU->GPREG0; }
static inline unsigned int ReadBootData1(void) { return LPC_PMU->GPREG1; }
static inline unsigned int ReadBootData2(void) { return LPC_PMU->GPREG2; }
static inline unsigned int ReadBootData3(void) { return LPC_PMU->GPREG3; }
static inline unsigned int ReadBootData4(void) { return (LPC_PMU->GPREG4 >> 11) & 0xfffff; }

// *** Brownout detect
#define BOD0    0 // 1.69V assert 1.84V deassert
#define BOD1    1 // 2.29V assert 2.44V deassert
#define BOD2    2 // 2.59V assert 2.74V deassert
#define BOD3    3 // 2.87V assert 2.98V deassert

void BODInit(unsigned char interruptlevel);
void BODStop(void);
extern WEAK void BODInterrupt(void);

// *** Reset status
#define POWERON     0x1
#define EXTERNAL    0x2
#define WATCHDOG    0x4
#define BROWNOUT    0x8
#define SOFTWARE    0x10
#define POWEREDDOWN 0x20
unsigned char ResetStatus(void);


// ****************************************************************************
// *** GPIO Functions
// ****************************************************************************

// *** Pin number (and LED number) definitions
#define PIN0        0x001
#define PIN1        0x002
#define PIN2        0x004
#define PIN3        0x008
#define PIN4        0x010
#define PIN5        0x020
#define PIN6        0x040
#define PIN7        0x080
#define PIN8        0x100
#define PIN9        0x200
#define PIN10        0x400
#define PIN11        0x800
#define ALL            0xfff

// *** Initialise pins as digital IO mode (note: default state is digital input)
void Port0Init(unsigned short pins);
void Port1Init(unsigned short pins);
void Port2Init(unsigned short pins);
void Port3Init(unsigned short pins);

// *** Sets pins as inputs or outputs
static inline void Port0SetIn(unsigned short pins) {    LPC_GPIO0->DIR &= ~pins;    }
static inline void Port1SetIn(unsigned short pins) {    LPC_GPIO1->DIR &= ~pins;    }
static inline void Port2SetIn(unsigned short pins) {    LPC_GPIO2->DIR &= ~pins;    }
static inline void Port3SetIn(unsigned short pins) {    LPC_GPIO3->DIR &= ~pins;    }
static inline void Port0SetOut(unsigned short pins) {    LPC_GPIO0->DIR |= pins;    }
static inline void Port1SetOut(unsigned short pins) {    LPC_GPIO1->DIR |= pins;    }
static inline void Port2SetOut(unsigned short pins) {    LPC_GPIO2->DIR |= pins;    }
static inline void Port3SetOut(unsigned short pins) {    LPC_GPIO3->DIR |= pins;    }

// *** Writes digital high or low to pins that are in Output mode
static inline void Port0Write(unsigned short pins, unsigned char value) {
    if(value) LPC_GPIO0->DATA |= pins;
    else LPC_GPIO0->DATA &= ~pins;
}
static inline void Port1Write(unsigned short pins, unsigned char value) {
    if(value) LPC_GPIO1->DATA |= pins;
    else LPC_GPIO1->DATA &= ~pins;
}
static inline void Port2Write(unsigned short pins, unsigned char value) {
    if(value) LPC_GPIO2->DATA |= pins;
    else LPC_GPIO2->DATA &= ~pins;
}
static inline void Port3Write(unsigned short pins, unsigned char value) {
    if(value) LPC_GPIO3->DATA |= pins;
    else LPC_GPIO3->DATA &= ~pins;
}

// *** Toggle the status of a pin
static inline void Port0Toggle(unsigned short pins) {    LPC_GPIO0->DATA ^= pins;    }
static inline void Port1Toggle(unsigned short pins) {    LPC_GPIO1->DATA ^= pins;    }
static inline void Port2Toggle(unsigned short pins) {    LPC_GPIO2->DATA ^= pins;    }
static inline void Port3Toggle(unsigned short pins) {    LPC_GPIO3->DATA ^= pins;    }

// *** Returns the digital value on a pin that are in Input mode
static inline unsigned char Port0Read(unsigned short pins) {
    if(LPC_GPIO0->DATA & pins) return 1;
    else return 0;
}
static inline unsigned char Port1Read(unsigned short pins) {
    if(LPC_GPIO1->DATA & pins) return 1;
    else return 0;
}
static inline unsigned char Port2Read(unsigned short pins) {
    if(LPC_GPIO2->DATA & pins) return 1;
    else return 0;
}
static inline unsigned char Port3Read(unsigned short pins) {
    if(LPC_GPIO3->DATA & pins) return 1;
    else return 0;
}

// *** Returns the data on a whole port
static inline unsigned short Port0Data(void) { return LPC_GPIO0->DATA; }
static inline unsigned short Port1Data(void) { return LPC_GPIO1->DATA; }
static inline unsigned short Port2Data(void) { return LPC_GPIO2->DATA; }
static inline unsigned short Port3Data(void) { return LPC_GPIO3->DATA; }

// *** Some LED functions
#define LED0        0x001
#define LED1        0x002
#define LED2        0x004
#define LED3        0x008

static inline void LEDOn(unsigned short leds) {    LPC_GPIO3->DATA &= ~leds;    }
static inline void LEDOff(unsigned short leds) { LPC_GPIO3->DATA |= leds;    }
static inline void LEDWrite(unsigned short leds) { LPC_GPIO3->DATA = ~leds;   }
static inline void LEDToggle(unsigned short leds) {    LPC_GPIO3->DATA ^= leds;    }
static inline void LEDInit(unsigned short leds) { Port3Init(leds);    Port3SetOut(leds);    LEDOff(leds);    }

// *** Configure hysteresis on a pin (note: default stat is hysteresis disabled)
//#define ON            1// this defined as such elsewhere
//#define OFF            0// this defined as such elsewhere
void Port0Hysteresis(unsigned short pins, unsigned char value);
void Port1Hysteresis(unsigned short pins, unsigned char value);
void Port2Hysteresis(unsigned short pins, unsigned char value);
void Port3Hysteresis(unsigned short pins, unsigned char value);

// *** Configure pull-up/-down resistors on a pin (note: default state is pull-up enabled)
#define PULLUP        0x2
#define PULLDOWN    0x1
#define REPEATER    0x3

void Port0Pull(unsigned short pins, unsigned char value);
void Port1Pull(unsigned short pins, unsigned char value);
void Port2Pull(unsigned short pins, unsigned char value);
void Port3Pull(unsigned short pins, unsigned char value);

// *** Configure interrupts on a pin (note: default state is interrupts disabled)
//#define OFF       0  // this defined as such elsewhere
#define FALLING     0x1
#define RISING      0x2
#define BOTH        0x3
#define LOW         0x10
#define HIGH        0x20
void Port0SetInterrupt(unsigned short pins, unsigned char mode);
void Port1SetInterrupt(unsigned short pins, unsigned char mode);
void Port2SetInterrupt(unsigned short pins, unsigned char mode);
void Port3SetInterrupt(unsigned short pins, unsigned char mode);

// *** Function prototypes for user-supplied port-wide interrupt functions
extern WEAK void Port0Interrupt(unsigned short GPIOPins);
extern WEAK void Port1Interrupt(unsigned short GPIOPins);
extern WEAK void Port2Interrupt(unsigned short GPIOPins);
extern WEAK void Port3Interrupt(unsigned short GPIOPins);

static inline unsigned short Port0GetInterrupt(void) { return LPC_GPIO0->MIS; }
static inline unsigned short Port1GetInterrupt(void) { return LPC_GPIO1->MIS; }
static inline unsigned short Port2GetInterrupt(void) { return LPC_GPIO2->MIS; }
static inline unsigned short Port3GetInterrupt(void) { return LPC_GPIO3->MIS; }



// *** Function prototypes for user-supplied pin-specific interrupt functions
extern WEAK void Port0Pin0Interrupt(void);
extern WEAK void Port0Pin1Interrupt(void);
extern WEAK void Port0Pin2Interrupt(void);
extern WEAK void Port0Pin3Interrupt(void);
extern WEAK void Port0Pin4Interrupt(void);
extern WEAK void Port0Pin5Interrupt(void);
extern WEAK void Port0Pin6Interrupt(void);
extern WEAK void Port0Pin7Interrupt(void);
extern WEAK void Port0Pin8Interrupt(void);
extern WEAK void Port0Pin9Interrupt(void);
extern WEAK void Port0Pin10Interrupt(void);
extern WEAK void Port0Pin11Interrupt(void);
extern WEAK void Port1Pin0Interrupt(void);
extern WEAK void Port1Pin1Interrupt(void);
extern WEAK void Port1Pin2Interrupt(void);
extern WEAK void Port1Pin3Interrupt(void);
extern WEAK void Port1Pin4Interrupt(void);
extern WEAK void Port1Pin5Interrupt(void);
extern WEAK void Port1Pin6Interrupt(void);
extern WEAK void Port1Pin7Interrupt(void);
extern WEAK void Port1Pin8Interrupt(void);
extern WEAK void Port1Pin9Interrupt(void);
extern WEAK void Port1Pin10Interrupt(void);
extern WEAK void Port1Pin11Interrupt(void);
extern WEAK void Port2Pin0Interrupt(void);
extern WEAK void Port2Pin1Interrupt(void);
extern WEAK void Port2Pin2Interrupt(void);
extern WEAK void Port2Pin3Interrupt(void);
extern WEAK void Port2Pin4Interrupt(void);
extern WEAK void Port2Pin5Interrupt(void);
extern WEAK void Port2Pin6Interrupt(void);
extern WEAK void Port2Pin7Interrupt(void);
extern WEAK void Port2Pin8Interrupt(void);
extern WEAK void Port2Pin9Interrupt(void);
extern WEAK void Port2Pin10Interrupt(void);
extern WEAK void Port2Pin11Interrupt(void);
extern WEAK void Port3Pin0Interrupt(void);
extern WEAK void Port3Pin1Interrupt(void);
extern WEAK void Port3Pin2Interrupt(void);
extern WEAK void Port3Pin3Interrupt(void);


// ****************************************************************************
// *** USB HID and MSC functions
// ****************************************************************************

#if USB_EN
    #define HID 0
    #define MSC 1
    
    // *** USB MSC size definitions
    #define KBYTE   1024UI
    #define MBYTE   1048576UI
    #define GBYTE   1073741824UI

    // *** Structures required by USB
    typedef    struct {
        unsigned short USBVendorId;
        unsigned short USBProductId;
        unsigned short USBDeviceId;
        unsigned int USBHIDDescPtr;
        unsigned char USBInReportCount;
        unsigned char USBOutReportCount;
        unsigned char USBSampleInterval;
        unsigned int USBInPtr;
        unsigned int USBOutPtr;
    } HID_DEV_INFO;
    
    typedef    struct {
        unsigned short USBVendorId;
        unsigned short USBProductId;
        unsigned short USBDeviceId;
        unsigned int USBMSCDescPtr;
        unsigned int MSCVolPtr;
        unsigned int MSCBlockCount;
        unsigned int MSCBlockSize;
        unsigned int MSCMemorySize;
        unsigned int MSCWritePtr;
        unsigned int MSCReadPtr;
    }  MSC_DEV_INFO;

    typedef struct {
        unsigned short DevType;
        unsigned int DevDetailPtr;
    } USB_DEV_INFO; 
    
    typedef    struct {
        void (*init_clk_pins)(void);
        void (*isr)(void);
        void (*init)(const USB_DEV_INFO * DevInfoPtr); 
        void (*connect)(unsigned int con);
    } USBD;

    typedef    struct {
        const USBD * pUSBD;
    } ROM;
    
    // *** USB functions, assorted
    void HIDInit(unsigned char poll);
    void MSCInit(unsigned int size);
    void USBClockFix(void);
    void USBStop(void);
    void USBConnect(void);
    void USBDisconnect(void);
    
    // *** User-supplied interrupt functions
    extern WEAK void USBIn(unsigned char USBData[], unsigned int USBLength);
    extern WEAK void USBOut(unsigned char USBData[], unsigned int USBLength);
    extern WEAK void MSCRead(unsigned int MSCOffset, unsigned char MSCData[], unsigned int MSCLength);
    extern WEAK void MSCWrite(unsigned int MSCOffset, unsigned char MSCData[], unsigned int MSCLength);
#endif


// ****************************************************************************
// *** UART Functions
// ****************************************************************************

#if UART_EN
#if UART_GPS
/*
		//Functions defined in gps_uart.h
		void UARTInit(unsigned int baud);
    void UARTStop(void);
    void UARTFlush(void);
    void UARTWrite(unsigned char data[], unsigned int length);
    void UARTWriteByte(unsigned char data);
*/
#else
    // *** UART global datas
    extern unsigned char FBRUARTBuffer[UART_DATA_SIZE], FBRUARTIntChar;
    extern unsigned int FBRUARTIndex, FBRUARTPoint, FBRUARTOver, FBRUARTIntLength;

    // *** UART start, stop functions functions
    #define AUTO                0
    
    void UARTInit(unsigned int baud);
    void UARTStop(void);
    void UARTFlush(void);
    
    // *** UART Write functions
    void UARTWrite(unsigned char data[], unsigned int length);
    void UARTWriteByte(unsigned char data);
    
    // *** UART Read functions
    #if UART_MODE
        static inline unsigned int UARTAvailable(void) { return FBRUARTIndex; }
        static inline void UARTSetInterrupt(unsigned  char intChar, unsigned int intLength) { FBRUARTIntChar = intChar; FBRUARTIntLength = intLength; }
    #else
        static inline unsigned int UARTAvailable(void) {
            if(FBRUARTIndex < FBRUARTPoint) return (FBRUARTIndex + UART_DATA_SIZE - FBRUARTPoint);
            else return (FBRUARTIndex - FBRUARTPoint);
        }
    #endif
    unsigned char UARTReadByte(void);
    static inline unsigned int UARTOverrun(void) { return FBRUARTOver;}
    
    extern WEAK void UARTInterrupt(unsigned char UARTData[], unsigned int UARTLength);
#endif //UART_GPS
#endif 


// ****************************************************************************
// *** I2C Functions
// ****************************************************************************

#if I2C_EN

    // *** Some software state machine definitions
    #define I2C_IDLE            0
    #define I2C_STARTED         1
    #define I2C_RESTARTED       2
    #define I2C_REPEATED_START  3
    #define I2C_ACK             4
    #define I2C_NACK            5
    #define I2C_NACK_END        6
    #define I2C_WR_STARTED      7
    #define I2C_RD_STARTED      8
    #define I2C_GEN_STARTED     9
    #define I2C_ERROR           255

    #define I2C_AA              0x04
    #define I2C_SI              0x08
    #define I2C_STO             0x10
    #define I2C_STA             0x20
    #define I2C_ENA             0x40
    
    #define MASTER              0
    #define SLAVE               1
    
    #define WRITE               0
    #define READ                1

    // *** I2C global datas
    extern volatile unsigned char FBRI2CMasterState, FBRI2CMasterState2, FBRI2CSlaveState, FBRI2CSlaveState2, FBRI2CState;
    extern unsigned char FBRI2CBuffer[I2C_DATA_SIZE];
    extern volatile unsigned int FBRI2CRdLength, FBRI2CWrLength;
    extern volatile unsigned int FBRI2CRdIndex, FBRI2CWrIndex;

    // *** I2C functions
    void I2CInit(unsigned short speed, unsigned char mode);
    void I2CStop(void);
    void I2CMaster(unsigned char wrData[], unsigned int  wrLength, unsigned char rdData[], unsigned char rdLength);

    // *** I2C user-provided interrupts (for slave mode only)
    extern WEAK void I2CInterrupt(unsigned char I2CData[], unsigned int  I2CWriteLength);
    
    // *** EEPROM functions
    void EEPROMWriteByte(unsigned short address, unsigned char data);
    unsigned char EEPROMReadByte(unsigned short address);
    void EEPROMWrite(unsigned short address, unsigned char data[], unsigned int length);
    void EEPROMRead(unsigned short address, unsigned char data[], unsigned int length);
#endif


// ****************************************************************************
// *** SSP Functions
// ****************************************************************************

#if SSP_EN
    // *** SSP Initialise and stop
    //#define MASTER              0 // this is defined as such elsewhere
    //#define SLAVE               1 // this is defined as such elsewhere
    void SSPInit(unsigned short speed, unsigned char mode);
    void SSPStop(void);
    
    // ** Change SSP speed (used for example for reading from SD cards)
    void SSPSetSpeed(unsigned short speed);
    
    // *** Read and write functions
    unsigned char SSPWriteByte(unsigned char data);
    unsigned char SSPReadByte(void);
#endif

// ****************************************************************************
// *** Timers Functions
// ****************************************************************************

// *** Initialisations and stops
void Timer0Init(unsigned short prescale);
void Timer1Init(unsigned short prescale);
void Timer2Init(unsigned int prescale);
void Timer3Init(unsigned int prescale);

void Timer0Stop(void);
void Timer1Stop(void);
void Timer2Stop(void);
void Timer3Stop(void);

// *** Timer controls
static inline void Timer0Go(void) { LPC_TMR16B0->TCR = 1; }
static inline void Timer1Go(void) { LPC_TMR16B1->TCR = 1; }
static inline void Timer2Go(void) { LPC_TMR32B0->TCR = 1; }
static inline void Timer3Go(void) { LPC_TMR32B1->TCR = 1; }
static inline void Timer0Pause(void) { LPC_TMR16B0->TCR = 0; }
static inline void Timer1Pause(void) { LPC_TMR16B1->TCR = 0; }
static inline void Timer2Pause(void) { LPC_TMR32B0->TCR = 0; }
static inline void Timer3Pause(void) { LPC_TMR32B1->TCR = 0; }
static inline void Timer0Reset(void) { LPC_TMR16B0->TCR = 0; LPC_TMR16B0->TC = 0; }
static inline void Timer1Reset(void) { LPC_TMR16B1->TCR = 0; LPC_TMR16B1->TC = 0; }
static inline void Timer2Reset(void) { LPC_TMR32B0->TCR = 0; LPC_TMR32B0->TC = 0; }
static inline void Timer3Reset(void) { LPC_TMR32B1->TCR = 0; LPC_TMR32B1->TC = 0; }

// *** Timer values
static inline unsigned short Timer0Read(void) { return LPC_TMR16B0->TC; }
static inline unsigned short Timer1Read(void) { return LPC_TMR16B1->TC; }
static inline unsigned int Timer2Read(void) { return LPC_TMR32B0->TC; }
static inline unsigned int Timer3Read(void) { return LPC_TMR32B1->TC; }
static inline void Timer0Write(unsigned short value) { LPC_TMR16B0->TC = value; }
static inline void Timer1Write(unsigned short value) { LPC_TMR16B1->TC = value; }
static inline void Timer2Write(unsigned int value) { LPC_TMR32B0->TC = value; }
static inline void Timer3Write(unsigned int value) { LPC_TMR32B1->TC = value; }
static inline unsigned short Timer0Prescaler(void) { return LPC_TMR16B0->PC; }
static inline unsigned short Timer1Prescaler(void) { return LPC_TMR16B1->PC; }
static inline unsigned int Timer2Prescaler(void) { return LPC_TMR32B0->PC; }
static inline unsigned int Timer3Prescaler(void) { return LPC_TMR32B1->PC; }

static inline void Timer0DisableInterrupt(void) { NVIC_DisableIRQ(TIMER_16_0_IRQn); }
static inline void Timer1DisableInterrupt(void) { NVIC_DisableIRQ(TIMER_16_1_IRQn); }
static inline void Timer2DisableInterrupt(void) { NVIC_DisableIRQ(TIMER_32_0_IRQn); }
static inline void Timer3DisableInterrupt(void) { NVIC_DisableIRQ(TIMER_32_1_IRQn); }
static inline void Timer0EnableInterrupt(void) { NVIC_EnableIRQ(TIMER_16_0_IRQn); }
static inline void Timer1EnableInterrupt(void) { NVIC_EnableIRQ(TIMER_16_1_IRQn); }
static inline void Timer2EnableInterrupt(void) { NVIC_EnableIRQ(TIMER_32_0_IRQn); }
static inline void Timer3EnableInterrupt(void) { NVIC_EnableIRQ(TIMER_32_1_IRQn); }

// *** Timer match control
//#define OFF       0       // already defined elsewhere
//#define INTERRUPT   0x40  // already defined elsewhere
//#define RESET       0x2   // already defined elsewhere
#define STOP        0x4
#define PWM         0x8
#define OUTPUTOFF   0       // this one doesn't need to be specified
#define OUTPUTLOW   0x10
#define OUTPUTHIGH  0x20
#define OUTPUTTOGGLE 0x30

void Timer0Match0(unsigned short interval, unsigned char mode);
void Timer0Match1(unsigned short interval, unsigned char mode);
void Timer0Match2(unsigned short interval, unsigned char mode);
void Timer0Match3(unsigned short interval, unsigned char mode);
void Timer1Match0(unsigned short interval, unsigned char mode);
void Timer1Match1(unsigned short interval, unsigned char mode);
void Timer1Match2(unsigned short interval, unsigned char mode);
void Timer1Match3(unsigned short interval, unsigned char mode);
void Timer2Match0(unsigned int interval, unsigned char mode);
void Timer2Match1(unsigned int interval, unsigned char mode);
void Timer2Match2(unsigned int interval, unsigned char mode);
void Timer2Match3(unsigned int interval, unsigned char mode);
void Timer3Match0(unsigned int interval, unsigned char mode);
void Timer3Match1(unsigned int interval, unsigned char mode);
void Timer3Match2(unsigned int interval, unsigned char mode);
void Timer3Match3(unsigned int interval, unsigned char mode);

// *** Timer match status
#define MATCH0      0x1
#define MATCH1      0x2
#define MATCH2      0x4
#define MATCH3      0x8

static inline unsigned char Timer0Status(void) { return LPC_TMR16B0->EMR & 0xf; }
static inline unsigned char Timer1Status(void) { return LPC_TMR16B1->EMR & 0xf; }
static inline unsigned char Timer2Status(void) { return LPC_TMR32B0->EMR & 0xf; }
static inline unsigned char Timer3Status(void) { return LPC_TMR32B1->EMR & 0xf; }

static inline void Timer0SetStatus(unsigned char status) { LPC_TMR16B0->EMR = status & 0xf; }
static inline void Timer1SetStatus(unsigned char status) { LPC_TMR16B1->EMR = status & 0xf; }
static inline void Timer2SetStatus(unsigned char status) { LPC_TMR32B0->EMR = status & 0xf; }
static inline void Timer3SetStatus(unsigned char status) { LPC_TMR32B1->EMR = status & 0xf; }

// *** Timer capture control
// #define OFF         0       // already defined elsewhere
// #define FALLING     0x1     // already defined elsewhere
// #define RISING      0x2     // already defined elsewhere
// #define BOTH        0x3     // already defined elsewhere
// #define INTERRUPT   0x40    // already defined elsewhere
#define COUNTER   0x10

void Timer0Capture(unsigned char mode);
void Timer1Capture(unsigned char mode);
void Timer2Capture(unsigned char mode);
void Timer3Capture(unsigned char mode);

static inline unsigned short Timer0CaptureValue(void) { return LPC_TMR16B0->CR0; }
static inline unsigned short Timer1CaptureValue(void) { return LPC_TMR16B1->CR0; }
static inline unsigned int Timer2CaptureValue(void) { return LPC_TMR32B0->CR0; }
static inline unsigned int Timer3CaptureValue(void) { return LPC_TMR32B1->CR0; }

// *** Timer interrupts
//#define MATCH0      0x1 // defined as such elsewhere
//#define MATCH1      0x2 // defined as such elsewhere
//#define MATCH2      0x4 // defined as such elsewhere
//#define MATCH3      0x8 // defined as such elsewhere
#define CAPTURE     0x10
extern WEAK void Timer0Interrupt(unsigned char TimerMatch);
extern WEAK void Timer0Interrupt0(void);
extern WEAK void Timer0Interrupt1(void);
extern WEAK void Timer0Interrupt2(void);
extern WEAK void Timer0Interrupt3(void);
extern WEAK void Timer0InterruptC(unsigned short TimerValue);
extern WEAK void Timer1Interrupt(unsigned char TimerMatch);
extern WEAK void Timer1Interrupt0(void);
extern WEAK void Timer1Interrupt1(void);
extern WEAK void Timer1Interrupt2(void);
extern WEAK void Timer1Interrupt3(void);
extern WEAK void Timer1InterruptC(unsigned short TimerValue);
extern WEAK void Timer2Interrupt(unsigned char TimerMatch);
extern WEAK void Timer2Interrupt0(void);
extern WEAK void Timer2Interrupt1(void);
extern WEAK void Timer2Interrupt2(void);
extern WEAK void Timer2Interrupt3(void);
extern WEAK void Timer2InterruptC(unsigned int TimerValue);
extern WEAK void Timer3Interrupt(unsigned char TimerMatch);
extern WEAK void Timer3Interrupt0(void);
extern WEAK void Timer3Interrupt1(void);
extern WEAK void Timer3Interrupt2(void);
extern WEAK void Timer3Interrupt3(void);
extern WEAK void Timer3InterruptC(unsigned int TimerValue);

// ****************************************************************************
// *** ADC Functions
// ****************************************************************************

// *** ADC channel number definitions
#define CHN0        0x01
#define CHN1        0x02
#define CHN2        0x04
#define CHN3        0x08
#define CHN4        0x10
#define CHN5        0x20
#define CHN6        0x40
#define CHN7        0x80

// *** Initialise or stop ADC
void ADCInit(unsigned short channels); 
void ADCStop(void);

// *** Read ADC value
unsigned short ADCRead(unsigned char channel);

// *** Prototype for user-supplied ADC interrupt function (only used when ADC is set to interrupt mode)
extern WEAK void ADCInterrupt(unsigned char ADCChannel, unsigned short ADCValue);


#endif /* __UAFUNC_H__ */
