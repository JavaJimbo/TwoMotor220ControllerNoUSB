/***************************************************************************************
 * Project:     Two Motor 220 NO USB - adapted from USB version
 *              For PIC 32MX220F032D and Two Motor Board Rev 2.0
 * FileName:    main.c 
 *   
 * 8-25-17 JBS: `A IO pin changes from Quad Board
 *               Both motors and D flips tested, both UARTS work, USB works.
 * 8-26-17 JBS:  Got AD and switch inputs working and interrupt on change
 *               Note that ConfigAd() must be initialized before digital inputs.
 * 8-28-17 JBS:  Everything has been tested except I2C. USB clobbers Atmel Memory input.
 *               ConfigAd() must be called before initializing IO.
 * 8-29-17 JBS:  Minor changes for testing purposes. Disable Atmel when USB is enabled.
 * 5-27-19 JBS:  Removed USB stuff and created new project.
 * 6-2-19  JBS:  Works great with AM 12V motor.
 * 6-3-19  JBS:  Added makeFloatString() routine. Accuracy is +/- 20 counts.
 *               Two Motor Controller Board Rev 2.0 with rotary sensor on Sense #1, pot on Sense #2 
 *               PWM MAX set to 1500
 ****************************************************************************************/

// #define TESTOUT LATBbits.LATB0
#define SW1 PORTBbits.RB1
#define SW2 PORTBbits.RB0

#define  MULTIPLIER 10000
#define _SUPPRESS_PLIB_WARNING

#define MAXSUM 600000
#define PWM_MAX 1500

#define LED         LATBbits.LATB13
#define DISABLE_OUT PORTBbits.RB4
#define FAULT1_IN PORTCbits.RC7
#define FAULT2_IN PORTCbits.RC8

#define FORWARD 1
#define REVERSE 0

#define EncoderOne TMR1
#define EncoderTwo TMR4

#define PWM1 OC4RS
#define PWM2 OC3RS

#define DIR1_OUT LATAbits.LATA7
#define DIR2_OUT LATCbits.LATC5

#define ENC1_DIR PORTBbits.RB15
#define ENC2_DIR PORTAbits.RA10

#define STX 36
#define ETX 13
#define DLE 16
#define MAXPACKET 80
#define MAXVELOCITY 500
#define DRIVEDIRECT 145
#define ROOMBA 0
#define RASPI 240
#define ROBOTNIK 19
#define SETPID 69
#define START 128
#define STOP 173
#define POWERDOWN 133
#define RESET 7
#define SAFE 131
#define FULL 132
#define QUIT 128
#define SHUTDOWN 160
#define NUMMOTORS 4


#define STANDBY 0
#define RUN 111

#define SYS_FREQ 60000000  // With 8 Mhz crystal and FPLLMUL = MUL_15
#define GetPeripheralClock() SYS_FREQ 

#include "Delay.h"
//#include "AT45DB161.h"
//#include "PCA9685.h"
#include <plib.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// #include <ctype.h>
// #include <XC.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier for 220 - yields 60 Mhz
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx3      // ICE/ICD Comm Channel Select
#pragma config JTAGEN   = OFF           // Use JTAG pins for normal IO
#pragma config DEBUG    = OFF            // Enable/disable debugging


/*** DEFINES *****************************************************************/
#define HOSTuart UART2
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR     
#define MAXBUFFER 64

#define false FALSE
#define true TRUE

#define START_ONE 80
#define START_TWO 80
#define START_THREE 20
#define START_FOUR 20
#define TIMEOUT 200
#define UART_TIMEOUT 400
#define MAXBITLENGTH 20

#define STX 36
#define ETX 13
#define DLE 16

/** I N C L U D E S **********************************************************/
#include "GenericTypeDefs.h"


#define PCA9685_ADDRESS 0b01000000   
/** V A R I A B L E S ********************************************************/
long LEFTREARENC, RIGHTREARENC;
#define MAXDATABYTES 64
unsigned char arrData[MAXDATABYTES];
unsigned char HOSTRxBuffer[MAXBUFFER + 1];
unsigned char HOSTTxBuffer[MAXBUFFER + 1];
unsigned char HOSTBufferFull = false;
unsigned int RxDataLength = 0;

unsigned char USBRxBuffer[MAXBUFFER];
unsigned char USBTxBuffer[MAXBUFFER];

unsigned long RxLength = 0;
unsigned long TxLength = 0;

unsigned long previousExpected = 0, numExpectedBytes = 0;
unsigned char RXstate = 0;
unsigned char timeoutFlag = false;
unsigned long numBytesReceived = 0;

/** V A R I A B L E S ******************************
 **************************/
#define FILTERSIZE 400
struct PIDtype
{
    short error[FILTERSIZE];
    long sumError;    
    float kP;
    float kI;
    float kD;
    unsigned long PWMoffset;
    long PWMvalue;
    long PreviousPWMvalue;
    long Rollovers;
    long ADActual;
    long ADPrevious;
    long ADCommand;
    unsigned char reset;    
} PID[NUMMOTORS];

#define MAXNUM 16
unsigned char NUMbuffer[MAXNUM + 1];
unsigned char KPbuffer[MAXNUM + 1];
unsigned char KIbuffer[MAXNUM + 1];
unsigned char KDbuffer[MAXNUM + 1];

/** P R I V A T E  P R O T O T Y P E S ***************************************/
// extern unsigned long CRCcalculate(unsigned char *message, unsigned char nBytes);
unsigned char intFlag = false, displayPWMFlag = false;
void InitializeSystem(void);
void ConfigAd(void);

unsigned char setMotorPWM(long side, long PWMvalue, unsigned char direction);
void putch(unsigned char ch);
void makeFloatString(float InValue, int numDecimalPlaces, unsigned char *arrAscii);
int fround (float x);


unsigned int decodePacket(unsigned char *ptrInPacket, unsigned int inPacketSize, unsigned char *ptrData);
long PIDcontrol(long servoID, struct PIDtype *PID);
union convertType 
{
    unsigned char byte[2];
    long integer;
} convert;


// #define MAXPWM 3000
#define MAXPWM 3000
unsigned long dataLength = 0;
unsigned char hostChar = 0, UART1char = 0;
#define NUM_AD_INPUTS 2
unsigned long arrADreading[NUM_AD_INPUTS];
unsigned long Timer5Counter = 0;
unsigned long PORTBreg = 0;

#define ATMEL_BUFFER 2

unsigned char displayFlag = true;

#define MAXSLEW 2

void ResetPID()
{
    int i, j;
    for (i = 0; i < NUMMOTORS; i++)
    {
        PID[i].sumError = 0;        
        PID[i].kP = 6.0;
        PID[i].kI = 0.010;
        PID[i].kD = 1.0;
        PID[i].PWMoffset = 100;
        PID[i].PreviousPWMvalue = PID[i].PWMvalue = 0;
        PID[i].Rollovers = 0;                
        PID[i].ADActual = 0;
        PID[i].ADPrevious = 0;
        PID[i].ADCommand = 0;
        PID[i].reset = true;
        for (j = 0; j < FILTERSIZE; j++) PID[i].error[j] = 0;
    }
}

int fround (float x)
{
  return (int)(x + 0.5f);
}


void makeFloatString(float InValue, int numDecimalPlaces, unsigned char *arrAscii)
{
    int i = 0, j = 0;        
    float floValue = InValue;
    unsigned char digit;
    
    if (floValue < 0)
    {
        floValue = 0 - floValue;
        arrAscii[j++] = '-';
    }
    while (floValue >= 1)
    {
        floValue = floValue / 10.0;
        i++;
    }
    if (i == 0) 
    {
        arrAscii[j++] = '0';
        arrAscii[j++] = '.';
    }
    else
    {
        while (i > 0)
        {
            floValue = floValue * 10;
            digit = (unsigned char)floValue;
            arrAscii[j++] = digit + '0';
            floValue = floValue - (float) digit;
            i--;
        }
        arrAscii[j++] = '.';
    }
    if (numDecimalPlaces > 0)
    {        
        i = numDecimalPlaces;
        while (i > 0)
        {  
            floValue = floValue * 10;
            digit = (unsigned char)floValue;
            arrAscii[j++] = digit + '0';
            floValue = floValue - (float) digit;
            i--;                         
        }
    }
    else arrAscii[j++] = '0';
    arrAscii[j++] = '\0';
}

/*
void makeFloatString(float InValue, int numDecimalPlaces, unsigned char *arrAscii)
{
    int i = 0, j = 0;        
    float floValue = InValue;
    float absValue;
    long powerOfTen = 1;
    unsigned char digit;
        
    if (InValue < 0)
    {
        absValue = 0 - InValue;
        arrAscii[j++] = '-';
    }
    else absValue = InValue;
    floValue = absValue;
    while (floValue >= 1)
    {
        floValue = floValue / 10.0;
        powerOfTen = powerOfTen * 10;
        i++;
    }
    floValue = absValue;
    if (i == 0) 
    {
        arrAscii[j++] = '0';
        arrAscii[j++] = '.';
    }
    else
    {
        while (i > 0)
        {            
            digit = (unsigned char)(floValue / powerOfTen);
            arrAscii[j++] = digit + '0';            
            floValue = floValue - (float) powerOfTen;
            powerOfTen = powerOfTen / 10;
            i--;
        }
        arrAscii[j++] = '.';
    }
    if (numDecimalPlaces > 0)
    {        
        i = numDecimalPlaces;
        while (i > 0)
        {  
            floValue = floValue * 10;
            digit = (unsigned char)(floValue);
            arrAscii[j++] = digit + '0';            
            floValue = floValue - (float) digit;;
            i--;            
        }
    }
    else arrAscii[j++] = '0';
    arrAscii[j++] = '\0';
}
*/

int main(void) 
{
    unsigned char i = 0, p = 0, q = 0;     ;
    long PWMvalue = 0; 
    
    unsigned char runMode = false;
    unsigned char standby = false;
    unsigned char ch, command = 0;
    int JogPWM = 0;
    float floValue;    
    unsigned char resetFlag = true;    
        
    PWM1 = PWM2 = 0;
    ResetPID();
    
    DelayMs(200);
    InitializeSystem();
    printf("\r\rTwo Motor Controller Board Rev 2.0\rStore on GitHub 6-3-19\rMotor #1 with rotary sensor on Sense #1, pot on Sense #2\rPWM MAX set to 1500");    
                
    while(1) 
    {    
        if (intFlag)
        {
            intFlag = false;
            if (runMode)
            {               
                LED = 1;
                for (i = 0; i < NUMMOTORS; i++)
                {
                    mAD1IntEnable(INT_ENABLED);                 
                    PID[0].ADCommand = (long) arrADreading[0];
                    PID[0].ADActual = (long) arrADreading[1];
                    PIDcontrol(0, PID);             
                
                    if (JogPWM != 0) PWMvalue = JogPWM;
                    else PWMvalue = PID[0].PWMvalue;                
                    if (PWMvalue < 0)          
                    {            
                        DIR1_OUT = REVERSE;                    
                        PWMvalue = 0 - PWMvalue;
                    }
                    else DIR1_OUT = FORWARD;                    
                    if (runMode) 
                    {
                        if (i == 0) PWM1 = PWMvalue;
                        else if (i == 1) PWM2 = PWMvalue;
                    }
                    else PWM2 = PWM1 = 0;                
                }
                LED = 0;
            }
            else PWM1 = PWM2 = 0;                
        }        
        
        if (HOSTBufferFull)
        {
            HOSTBufferFull = false; 
            printf("\rReceived: ");
            q = 0;
            command = 0;
            for (p = 0; p < MAXBUFFER; p++) 
            {
                ch = HOSTRxBuffer[p];
                if (ch >= 'a' && ch <= 'z') ch = ch - 'a' + 'A';
                
                if (ch >= 'A' && ch <= 'Z') command = ch;
                else if (ch == ' ') command = ' ';
                
                putch(ch);
                if (ch == '\r' || ch == ' ')break;
                if ( ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-') && q < MAXNUM) NUMbuffer[q++] = ch;
            }
            if (q) 
            {
                NUMbuffer[q] = '\0';
                floValue = atof(NUMbuffer);
            }
            if (command) 
            {
                switch (command) 
                {
                    case 'J':
                        JogPWM = (long) floValue;
                        printf("\rJog: %d", JogPWM);
                        break;
                    case 'F':
                        DIR1_OUT = FORWARD;
                        printf("\rFORWARD");
                        break;
                    case 'R':
                        DIR1_OUT = REVERSE;
                        printf("\rREVERSE");
                        break;                        
                    case 'P':
                        if (q) PID[0].kP = floValue;
                        break;
                    case 'I':
                        if (q) PID[0].kI = floValue;
                        break;
                    case 'D':
                        if (q) PID[0].kD = floValue;
                        break;
                    case 'O':
                        if (q) PID[0].PWMoffset = (long) floValue;
                        break;
                    case ' ':
                        if (standby) 
                        {                            
                            runMode = false;
                            printf("\rHALT");
                        }
                        else if (runMode)
                        {
                            runMode = false;
                            resetFlag = true;
                            for (i = 0; i < NUMMOTORS; i++)
                                PID[i].PreviousPWMvalue = PID[i].PWMvalue = 0;                            
                            printf("\rHALT");
                        }
                        else 
                        {
                            runMode = true;
                            printf("\rRUN");
                        }
                        standby = false;                        
                        break;
                    case 'M':
                        if (displayFlag)
                        {
                            displayFlag = false;
                            printf("\rDisplay OFF");
                        }
                        else {
                            displayFlag = true;
                            printf("\rDisplay ON");
                        }   
                        break;
                    case 'S':
                        if (standby) 
                        {
                            standby = false;
                            printf("\rStandby OFF");
                        }
                        else
                        {
                            standby = true;
                            printf("\rStandby ON");
                        }     
                        break;
                    case 'Z':
                        ResetPID();
                        printf("\rPID reset = true");
                        break;
                    default:
                        printf("\rCommand: %c", command);
                        break;
                } // end switch command
                
                makeFloatString(PID[0].kP, 3, KPbuffer);
                makeFloatString(PID[0].kI, 3, KIbuffer);
                makeFloatString(PID[0].kD, 3, KDbuffer);
                
                printf("\rkP = %s, kI = %s, kD = %s, Offset: %d\r", KPbuffer, KIbuffer, KDbuffer, PID[0].PWMoffset);
            } // End if command             
        } // End if HOSTBufferFull
    } // End while(1))
} // End main())


void putch(unsigned char ch) {
    while (!IFS1bits.U2TXIF); // set when register is empty 
    U2TXREG = ch;
}

void InitializeSystem(void) 
{
    unsigned char ch;
    unsigned long dummyRead;

    ConfigAd();
    
    mJTAGPortEnable(false);           

    // DIGITAL OUTPUTS: 
    PORTSetPinsDigitalOut(IOPORT_A, BIT_7);
    
    PORTSetPinsDigitalOut(IOPORT_B, BIT_4 | BIT_5 | BIT_7 | BIT_13 | BIT_14); // Added BIT 13 for LED out and BIT 7 for Atmel CS 
    
    PORTSetPinsDigitalOut(IOPORT_C, BIT_5 | BIT_9); // Added BIT 9 for Atmel WP write protect    
    
    PPSOutput(2, RPB5, SDO1);    
    PPSInput(2, SDI1, RPA9);     
    
    //ATMEL_WRITE_PROTECT = 1; // Enable EEPROM write protection at startup, and disable chip select
    //ATMEL_CS = 1;

    DIR1_OUT = DIR2_OUT = 0;

    // Enable analog inputs
    ANSELCbits.ANSC0 = 1; // AN6  
    ANSELCbits.ANSC1 = 1; // AN7     

    // Disable analog on digital inputs:    
    ANSELBbits.ANSB15 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;
    ANSELBbits.ANSB2 = 0;
    ANSELCbits.ANSC3 = 0;


    PORTSetPinsDigitalIn(IOPORT_A, BIT_10); // For Encoder 2 Direction input `A        
    PORTSetPinsDigitalIn(IOPORT_B, BIT_0 | BIT_1 | BIT_15); // Removed BIT_13   `A    
    PORTSetPinsDigitalIn(IOPORT_C, BIT_7 | BIT_8); // Changed to input  `A          
    

    LED = 0;
    DISABLE_OUT = 0;

        // Set up main UART    
    PPSOutput(4, RPC2, U2TX); // REV 1.0 
    PPSInput(2, U2RX, RPA8);    
    
    // Configure UART #2 (HOST UART))
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 921600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_U2RX, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);    

    do {
        ch = UARTGetDataByte(HOSTuart);
    } while (ch);

    // Set counter inputs    
    PPSInput(3, T4CK, RPB2);

    // Set up timers as counters
    T1CON = 0x00;
    T1CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T1CONbits.TCKPS1 = 0; // 1:1 Prescaler
    T1CONbits.TCKPS0 = 0;
    T1CONbits.TSYNC = 1;
    PR1 = 0xFFFF;
    T1CONbits.TON = 1; // Let her rip 

    T4CON = 0x00;
    T4CONbits.TCS = 1; // Use external counter as input source (motor encoder)
    T4CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T4CONbits.TCKPS1 = 0;
    T4CONbits.TCKPS0 = 0;
    T4CONbits.T32 = 0; // TMRx and TMRy form separate 16-bit timers
    PR4 = 0xFFFF;
    T4CONbits.TON = 1; // Let her rip     

    // Set up Timer 5 interrupt with a priority of 2
    ConfigIntTimer5(T5_INT_ON | T5_INT_PRIOR_2);
    OpenTimer5(T5_ON | T5_SOURCE_INT | T5_PS_1_8, 7500);

    // Set up Timer 2 for PWM time base    
    T2CON = 0x00;
    T2CONbits.TCKPS2 = 0; // 1:1 Prescaler
    T2CONbits.TCKPS1 = 0;
    T2CONbits.TCKPS0 = 0;
    PR2 = 3000; // Use 50 microsecond rollover for 20 khz
    T2CONbits.TON = 1; // Let her rip       
    
    // Set up Timer 2 interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    // OpenTimer2(T2_ON | T2_SOURCE_INT);
    

    // Set up PWM OC3
    PPSOutput(4, RPC4, OC3);    
    OC3CON = 0x00;
    OC3CONbits.OC32 = 0; // 16 bit PWM
    OC3CONbits.ON = 1; // Turn on PWM
    OC3CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC3CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC3CONbits.OCM1 = 1;
    OC3CONbits.OCM0 = 0;
    OC3RS = 0;

    // Set up PWM OC4 on D6 on the Olimex 220 board:
    PPSOutput(3, RPC6, OC4);
    OC4CON = 0x00;
    OC4CONbits.OC32 = 0; // 16 bit PWM
    OC4CONbits.ON = 1; // Turn on PWM
    OC4CONbits.OCTSEL = 0; // Use Timer 2 as PWM time base
    OC4CONbits.OCM2 = 1; // PWM mode enabled, no fault pin
    OC4CONbits.OCM1 = 1;
    OC4CONbits.OCM0 = 0;
    OC4RS = 0;
    

/*    
    // Set up interrupt on change for the PORT B input pins RB0 and RB1
    CNCONBbits.ON = 1; // CN is enabled
    CNCONBbits.SIDL = 0; // CPU Idle does not affect CN operation
    CNENBbits.CNIEB0 = 1; // Enable RB0 change notice    
    CNENBbits.CNIEB1 = 1; // Enable RB1 change notice
*/
    // Read port B to clear mismatch condition
    dummyRead = PORTB;
    
    /*
    mCNSetIntPriority(2);
    mCNSetIntSubPriority(2);
    mCNBClearIntFlag();
    mCNBIntEnable(BIT_0 | BIT_1);
    */
    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();


}//end UserInit


#define UART1bits U1STAbits

void __ISR(_UART_1_VECTOR, IPL2AUTO) IntUart1Handler(void) {
    unsigned char dummy;

    if (INTGetFlag(INT_SOURCE_UART_RX(UART1))) {
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
        if (UART1bits.OERR || UART1bits.FERR) {
            if (UARTReceivedDataIsAvailable(UART1))
                dummy = UARTGetDataByte(UART1);
            UART1bits.OERR = 0;
        }

        if (UARTReceivedDataIsAvailable(UART1)) {
            UART1char = UARTGetDataByte(UART1);
        }
    }
}

void ConfigAd(void) 
{
    //mPORTCSetPinsAnalogIn(BIT_0 | BIT_1);
    //mPORTBSetPinsDigitalOut(BIT_0 | BIT_1);
    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

    //  set AM7 (A1 on Olimex 220 board) input to analog
    // #define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA


// USE AN6 and AN7    
#define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 |SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 |\
    SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 |\
    SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
    
#define PARAM4    ENABLE_AN6_ANA | ENABLE_AN7_ANA     

    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}

void __ISR(_ADC_VECTOR, IPL2AUTO) ADHandler(void) {
    unsigned long offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < NUM_AD_INPUTS; i++)
        arrADreading[i] = (unsigned long) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
}

void __ISR(_TIMER_5_VECTOR, IPL2AUTO) Timer5Handler(void) {
    mT5ClearIntFlag(); // Clear interrupt flag
    if (Timer5Counter) Timer5Counter--;
}

void __ISR(_CHANGE_NOTICE_VECTOR, IPL2AUTO) ChangeNotice_Handler(void) {

    // Step #1 - always clear the mismatch condition first
    PORTBreg = PORTB & 0x0003;

    // Step #2 - then clear the interrupt flag
    mCNBClearIntFlag();

}

// Timer 2 generates an interrupt every 50 microseconds approximately
void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void) 
{
    static int intCounter = 0;    
    mT2ClearIntFlag(); // Clear interrupt flag       
    
    intCounter++;
    if (intCounter >= 50)
    {
        intCounter = 0;
        intFlag = true;
    }
}

long PIDcontrol(long servoID, struct PIDtype *PID)
{
    static long DisplayCounter = 20;    
    long lngError;     
    static lngPreviousError = 0;
    long actualPosition;    
    long commandPosition; 
    long diffPosition;
    long pastError;
    long derError;
    static short errIndex = 0;
    float PCorr = 0, ICorr = 0, DCorr = 0;
    
    if (PID[servoID].reset) diffPosition = 0;
    else diffPosition = PID[servoID].ADActual - PID[servoID].ADPrevious;        
    PID[servoID].ADPrevious = PID[servoID].ADActual;
    
    if (diffPosition < -333) PID[servoID].Rollovers++;
    if (diffPosition > 333) PID[servoID].Rollovers--;
    actualPosition = (PID[servoID].Rollovers * 758) + PID[servoID].ADActual;    
    commandPosition = PID[servoID].ADCommand;
    if (commandPosition < 128) commandPosition = 128;
    if (commandPosition > 700) commandPosition = 700;
    lngError = actualPosition - commandPosition;    

    if (lngError > 0x7FFF) lngError = 0x7FFF;
    if (lngError < -0x7FFF) lngError = -0x7FFF;
    
    pastError = PID[servoID].error[errIndex];    
    PID[servoID].sumError = PID[servoID].sumError + lngError - pastError;
    PID[servoID].error[errIndex] = (short) lngError;
    
    if (PID[servoID].sumError > MAXSUM) PID[servoID].sumError = MAXSUM;
    if (PID[servoID].sumError < -MAXSUM) PID[servoID].sumError = -MAXSUM;
        
    errIndex++;
    if (errIndex >= FILTERSIZE) errIndex = 0;             
    
    derError = lngError - pastError;     
    
    PCorr = ((float) lngError) * -PID[servoID].kP;    
    ICorr = ((float) PID[servoID].sumError)  * -PID[servoID].kI;
    DCorr = ((float) derError) * -PID[servoID].kD;

    float PIDcorrection = PCorr + ICorr + DCorr;
    
    if (PIDcorrection < 0) PID[servoID].PWMvalue = (long) (PIDcorrection - PID[servoID].PWMoffset);            
    else PID[servoID].PWMvalue = (long) (PIDcorrection + PID[servoID].PWMoffset);                    
               
    if (PID[servoID].PWMvalue > PWM_MAX) 
        PID[servoID].PWMvalue = PWM_MAX;
    else if (PID[servoID].PWMvalue < -PWM_MAX) 
        PID[servoID].PWMvalue = -PWM_MAX;
                
    if (displayFlag)
    {
        long lngDiffError = abs(lngError - lngPreviousError);
        if (lngDiffError > 2)
        {
            printf("\rCOM: %d, ACT: %d, ERR: %d, SUM: %d, P: %d, I: %d, D: %d, PWM: %d", commandPosition, actualPosition, lngError, PID[servoID].sumError, (int)PCorr, (int)ICorr, (int)DCorr, PID[servoID].PWMvalue); 
            DisplayCounter = 80;
            displayPWMFlag = true;
            lngPreviousError = lngError;
        }  
    }
    PID[servoID].reset = false;
    return 0;
}
    
#define ESC 27
#define CR 13
#define BACKSPACE 8
void __ISR(HOST_VECTOR, IPL2AUTO) IntHostUartHandler(void) 
{
unsigned char ch;
static unsigned long i = 0;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) 
    {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));                 
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;            
        }
        if (UARTReceivedDataIsAvailable(HOSTuart)) 
        {
            ch = UARTGetDataByte(HOSTuart);            
            if (ch != 0 && ch != '\n') {            
                if (ch == BACKSPACE) 
                {
                    if (i != 0) i--;
                    HOSTRxBuffer[i] = '\0';
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, ' ');
                    while(!UARTTransmitterIsReady(HOSTuart));
                    UARTSendDataByte (HOSTuart, BACKSPACE);                
                } 
                else if (i < MAXBUFFER) 
                {
                    HOSTRxBuffer[i] = ch;
                    i++;
                }            
                if ('\r' == ch || ' ' == ch) 
                {
                    HOSTBufferFull = true;
                    HOSTRxBuffer[i] = '\0';
                    i = 0;
                }
            }
        }
    }         
    
    if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) 
        INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));            
}