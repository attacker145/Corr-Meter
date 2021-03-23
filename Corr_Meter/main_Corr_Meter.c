/*
 * ======== main Rev011.c ========
    UART connections:
                                        P3.3/UCA0TXD|------------>
                                        P3.4/UCA0RXD|<------------
        Baud Rate 19200 8E1
    ADC Connections:
        Temperature across resistor     ----> P6.2/CB2/A2
        Vbat -->|P6.1/CB1/A1
    LCD:
        DB4 LCD PIN 11                  ----> P2.0
        DB5 LCD PIN 12                  ----> P2.1
        DB6 P2.2    LCD PIN 13

        Data Input String: DDMMYYHHMMSSTTUVCON or DDMMYYHHMMSSTTUVCOFF
        TT - work time 2 digit
*/

#include <string.h>
#include "driverlib.h"
#include <stdint.h>

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                     //USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"

#include <msp430x552x.h>
/*
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"
#include "TypeDef.h"
#include "api.h"

extern int Report(const char *pcFormat, ...);
extern void uart_print(const char *uart0_buf, unsigned char size);
extern void Message(const char *str);

// Function declarations
void convertTimeBinToASCII(uint8_t* str);
void convertDateBinToASCII(uint8_t* str);
void writeLCD(const unsigned char *strL1, const unsigned char *strL2);
void initRTC(void);
// Application globals
volatile Calendar newTime;
volatile uint8_t hour = 4, min = 30, sec = 00;  // Real-time clock (RTC) values.  4:30:00
volatile uint8_t day, month;
volatile uint8_t Rx_buf[10];
volatile uint8_t Rx16_buf[5];

yeartype year;
monthtype Stmonth;
daytype Stday;
Styeartype Styear;

uint16_t year1 = 0;
uint16_t year2 = 0;
uint16_t year3 = 0;
uint16_t year4 = 0;
volatile uint16_t BatStat = 0;
volatile uint16_t VSET = 0;
volatile uint8_t batch;
//Time
uint8_t SecMSB, SecLSB;
uint8_t MinMSB, MinLSB;
uint8_t HrMSB, HrLSB;
uint8_t Run = 0;//09152017
uint8_t USB_present = 0;

volatile uint8_t bSendTimeToHost = FALSE;       // RTC-->main():  "send the time over USB"
uint8_t bSendRec = FALSE;
uint8_t timeStr[10];                            // Stores the time as an ASCII string
uint8_t dateStr[12];
uint8_t t_lapse_min_str[16];//Time lapse in minutes string

//Temperature
uint8_t cntrStr[13];                            // Temperature counter string: convertCountBinToASCII(cntrStr);
uint8_t PulseStr[13];                           // Temperature counter binary

//Voltage
uint8_t Bat_voltage[13];
uint8_t Set_voltage[13];
uint8_t Set_thck[6];

//Temperature Limits Temperature Limits
uint8_t HiLimit[13];
uint8_t LoLimit[13];
uint8_t HiLimitMSB, HiLimitLSB;
uint8_t LoLimitMSB, LoLimitLSB;

uint32_t LoPlsCnt, HiPlsCnt;


uint8_t RecVal[28];
uint8_t RecStr[56];
uint8_t RecCntrl[28];
uint8_t RecCntrlStr[56];

unsigned char cmdcntr;

// Global flags set by events
volatile uint8_t bCDCDataReceived_event = FALSE;  // Flag set by event handler to
                                               // indicate data has been
                                               // received into USB buffer

#define BUFFER_SIZE 256
char dataBuffer[BUFFER_SIZE] = "";
uint8_t message[] = "\nAlarm\n";
uint8_t message1[] = "\nRTC Interrupts are enabled\n";
uint8_t messageSPI[] = "\nSPI Interrupt \n";
uint8_t ver[] = "\nBallast UVC v00\n";
uint8_t adc_message[] = "\nNo ADC sample...\n";
uint8_t P30[] = "\nP3.0 not Active\n";
uint8_t uart0_buf[4];
uint8_t UV;

volatile uint32_t PulseCount = 0;
volatile uint32_t t_lapse_min = 0;
volatile uint16_t t_lapse_hour = 0;
volatile uint8_t SecCount = 0;
volatile uint8_t SecCount1 = 0;
//volatile uint8_t SecCountLCD = 0; //09152017
volatile uint8_t MinCount = 0;
volatile uint8_t MinAlarmSet = 59;
volatile uint8_t flag = 0;
volatile uint8_t ADCflag = 0;
volatile uint8_t rus = 0;
volatile uint8_t buffer_full = 0;

volatile uint8_t HourStart = 0;
volatile uint8_t MinStart = 0;
volatile uint8_t SecStart = 0;

volatile uint8_t uart_cntr = 0;
volatile unsigned int i;

uint16_t work_time;

const unsigned char Msg1[] = "UVDI UVC SYSTEM ";
const unsigned char Msg2[] = "  Badger Robot  ";

const unsigned char Time[] = "Time:   ";
const unsigned char epty[] = "                ";
const unsigned char usb_msg1[] = "Not Connected...";
const unsigned char usb_msg2[] = "     USB Is     ";
const unsigned char usb_msg3[] = "Connected to a  ";
const unsigned char usb_msg4[] = "     USB        ";
const unsigned char usb_msg5[] = "     USB Is     ";
const unsigned char usb_msg6[] = "  Enumerating   ";
const unsigned char usb_msg7[] = "No ADC sample...";
const unsigned char usb_msg8[] = "  Low Battery   ";
const unsigned char usb_msg9[] = "Falling asleep..";


/*
 *  11 01 0013 0025 0E84
    11: The Slave Address (11 hex = address17 )
    01: The Function Code 1 (read Coil Status)
    0013: The Data Address of the first coil to read.
         ( 0013 hex = 19 , + 1 offset = coil #20 )
    0025: The total number of coils requested.  (25 hex = 37,  inputs 20 to 56 )
    0E84: The CRC (cyclic redundancy check) for error checking.

    Metronic sensor
    Read temperature example: 0x40 0x45 0xF183
    Read UV irradiance: 0x40 0x46 0xB182
    Read UV measurement range [W/m²]: 0x40 0x43 0x7181
    0x40 0x46
 */
//const char plc_on[] = {0x11, 0x05,   0x00,0x00,   0xFF,0x00,    0x8E,0xAA};//Slave 17, Coil Write, Q0000 address 0000, set FF00
uint8_t r_temp[] = {0x40, 0x45, 0xF1, 0x83, 0x00};//Read temperature - example
uint8_t r_irradiance[] = {0x40, 0x46, 0xB1, 0x82, 0x00};

//const unsigned char usb_msg9[] = "     ";
const unsigned char usb_msg10[] = "Bat.";
const unsigned char usb_msg11[] = {0xA0, 0x41, 0x54, 0xD0};// BAT.
const unsigned char usb_msg12[] = {0xA0,0x41,0x54,0x41,0x50,0x45,0xB1,0x20,0xA0,0x41,0x54,0x41,0x50,0x45,0xB1,0x20};// "BATAREYA RAZRYAZHENA "
const unsigned char usb_msg13[] = {0x20,0x20,0x20,0xA0,0x41,0x54,0x41,0x50,0x45,0xB1,0x20,0x20,0x20,0x20,0x20,0x20};// "   RAZRYAZHENA   "
const unsigned char usb_msg14[] = {0x20,0x20,0xA8,0x4F,0xE0,0x4B,0xA7,0xB0,0xAB,0x45,0x48,0x20,0x4B,0x20,0x20,0x20};// PODKLYUCHEN K
const unsigned char usb_msg15[] = {0x48,0x45,0x20,0xA8,0x4F,0xE0,0x4B,0xA7,0xB0,0xAB,0x45,0x48,0x20,0x4B,0x20,0x20};// HE PODKLYUCHEN K
//const unsigned char usb_msg8[] = "Rev-03\n";
const unsigned char Hi[] ={0x20, 0xA8, 0x4B, 0x20, 0xA7,0x48,0x4B,0x20,0x43,0x45,0x50,0x42,0xA5,0x43,0x20,0x20};
const unsigned char Hi1[] ={0x20,0x20,0x20,0x20,0x20, 0x4D, 0x4F,0x43,0x4B,0x42,0x41, 0x20,0x20,0x20,0x20,0x20};
const unsigned char DFLT[] = " Default State  ";
//#### SPI Variables ###########################################################
//unsigned char MST_Data, SLV_Data;
unsigned char temp;

// WatchDog as WatchDog
#define WatchDog    (WDTPW | WDTCNTCL | WDTSSEL__ACLK | WDTIS__512K)    // 011b = Watchdog clock source /(2^19) (00h:00m:16s at 32.768 kHz) (depends on clock configuration)

void main(void)
{

    Calendar currentTime;
    char *stateUVC = NULL;
    uint8_t usbstr[64];     //USB string
#ifdef Nitche_UVDI360
    uint16_t state_snsr1 = 0;
#endif
    uint8_t VCH1[14];       //CH1 buffer
    uint8_t VCH2[14];       //CH2 buffer
    uint8_t Cycles[10];
    uint8_t hour_run[10];
    uint16_t count = 0;
    uint32_t CycleCount;

    WDTCTL = WDTPW + WDTHOLD;                               // Stop WatchDog during initialization

    pin_mux();                                              //P1 interrupt is enabled here
    //adc_int_Vref_3_3();                                     //ADC init
    adc_int_3_3V_4ch();

    count = 0;                                              //Reset count, used in USB received buffer
    SecCount = 0;                                           //Initialize seconds counter

   /*
    * Real Time Clock: Initialize Calendar Mode of RTC
    * Select XT1: port5 pin4 and pin5 are connected to 32768 Hz external oscillator on the schematics.
    * GPIO_setAsPeripheralModuleFunctionInputPin function sets them to their periferal designated function - XIN, XOUT.
    */
   GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN4 + GPIO_PIN5);

   //Initialize LFXT1: UCS_turnOnLFXT1 function turns on the 32768 Hz external oscillatior
   UCS_turnOnLFXT1(UCS_XT1_DRIVE_3, UCS_XCAP_3);

//--------------------------------------------------------------------------------------------------------------------RTC
#define debug
   //Setup some Date and Time for Calendar. The calendar will be updated in the sec ISR
   currentTime.Seconds = 0x00;
   currentTime.Minutes = 0x00;
   currentTime.Hours = 0x00;
   currentTime.DayOfWeek = 0x00;
   currentTime.DayOfMonth = 00;
   currentTime.Month = 00;
   currentTime.Year = 0000;

   /*
    * Base Address of the RTC_A_A
    * Pass in current time, intialized above
    * Use BCD as Calendar Register Format
    */
   //RTC_A_initCalendar(RTC_A_BASE, &currentTime, RTC_A_FORMAT_BCD);
   RTC_A_initCalendar(RTC_A_BASE, &currentTime, RTC_A_FORMAT_BINARY);

   //Setup Calendar Alarm for 5:00pm on the 5th day of the week.
   //Note: Does not specify day of the week.
   RTC_A_configureCalendarAlarmParam param = {0};
   param.minutesAlarm = 0x00;
   param.hoursAlarm = 0x17;
   param.dayOfWeekAlarm = RTC_A_ALARMCONDITION_OFF;
   param.dayOfMonthAlarm = 0x05;
   RTC_A_configureCalendarAlarm(RTC_A_BASE, &param);
   //Initialize the Comparator B module
   /*
    * Base Address of Comparator B,
    * Pin CB0 to Positive(+) Terminal,
    * Reference Voltage to Negative(-) Terminal,
    * Normal Power Mode,
    * Output Filter On with minimal delay,
    * Non-Inverted Output Polarity
    */

   //Set comparator B

   //Select CBOUT function on P1.6/CBOUT and set P1.6 to output direction
   //CBOUT will be "high" or "low" depending on the state of the comparator and CBEX setting
   GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN6);


   Comp_B_initParam param1 = {0};
   param1.positiveTerminalInput = COMP_B_INPUT0;    //P6.0/CB0/A0 pulse temperature sensor
   //param1.positiveTerminalInput = COMP_B_INPUT1;
   param1.negativeTerminalInput = COMP_B_VREF;
   param1.powerModeSelect = COMP_B_POWERMODE_NORMALMODE;
   param1.outputFilterEnableAndDelayLevel = COMP_B_FILTEROUTPUT_DLYLVL1;
   param1.invertedOutputPolarity = COMP_B_NORMALOUTPUTPOLARITY;
   Comp_B_init(COMP_B_BASE, &param1);

   //Set the reference voltage that is being supplied to the (-) terminal
   /* Base Address of Comparator B,
    * Reference Voltage of 2.0 V,
    * Lower Limit of 2.0*(8/32) = 0.5V,
    * Upper Limit of 2.0*(8/32) = 0.5V,
    * Static Mode Accuracy
    */
    Comp_B_configureReferenceVoltageParam refVoltageParam = {0};
    refVoltageParam.supplyVoltageReferenceBase = COMP_B_VREFBASE2_0V;
    refVoltageParam.lowerLimitSupplyVoltageFractionOf32 = 8;
    refVoltageParam.upperLimitSupplyVoltageFractionOf32 = 8;
    refVoltageParam.referenceAccuracy = COMP_B_ACCURACY_STATIC;
    Comp_B_configureReferenceVoltage(COMP_B_BASE, &refVoltageParam);

    //delay for the reference to settle
    __delay_cycles(75);

    //USB initialize
    // Minimum Vcore setting required for the USB API is PMM_CORE_LEVEL_2
    PMM_setVCore(PMM_CORE_LEVEL_2);
    USBHAL_initPorts();             // Config GPIOS for low-power (output low)
    USBHAL_initClocks(8000000);     // Config clocks. MCLK = SMCLK = FLL = 8MHz; ACLK=REFO=32kHz
    USB_setup(TRUE,TRUE);           // Init USB & events; if a host is present, connect

    uart_init_19200_8MHz();
   //uart_init_19200_8MHz_8E1();

    DelayMs1 (2000);

    //Specify an interrupt to assert every minute: http://dev.ti.com/tirex/content/mspware/mspware__3.30.00.18/driverlib/doc/MSP430F5xx_6xx/html/group__rtc__a__api.html#gae76eccca4f8175296a2d1bbd0a15d8a7
    RTC_A_setCalendarEvent(RTC_A_BASE, RTC_A_CALENDAREVENT_MINUTECHANGE);//This function sets a specified event to assert the RTCTEVIFG interrupt. This interrupt is independent from the Calendar alarm interrupt.
    //RTC_A_setCalendarEvent(RTC_A_BASE, RTC_A_CALENDAREVENT_MIDNIGHT);//Specify an interrupt to assert midnight
    //void RTC_A_clearInterrupt: This function clears the RTC interrupt flag is cleared, so that it no longer asserts.
    RTC_A_clearInterrupt(RTC_A_BASE, RTCRDYIFG + RTCTEVIFG + RTCAIFG); // RTCRDYIFG - interrupt every second. The time is updated for display inside the interrupt
    //Enable interrupt for RTC Ready Status, which asserts when the RTC Calendar registers are ready to read.
    //Also, enable interrupts for the Calendar alarm and Calendar event.
    RTC_A_enableInterrupt(RTC_A_BASE, RTCRDYIE + RTCTEVIE + RTCAIE); //RTC Ready IE, RTC Event IE, RTC A IE
    //Enable RTC interrupt on every minute<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-<-Interrupt
    //RTC_A_clearInterrupt(RTC_A_BASE, RTCTEVIFG); // RTCEVIFG - clear interrupt flag
    //RTC_A_enableInterrupt(RTC_A_BASE, RTCTEVIE); // The above listed, RTC_A_CALENDAREVENT_MINUTECHANGE, minute interrupt enabled here

    RTC_A_startClock(RTC_A_BASE);   //Start RTC Clock

    initRTC();                      // Start the real-time clock (counter). Used to upload current time from RTClock

    Comp_B_enable(COMP_B_BASE);     //Allow power to Comparator module
    __no_operation();
    __no_operation();
    //Comp_B_disable(COMP_B_BASE);
    //__no_operation();
    __delay_cycles(75);         // delay for the reference to settle
    __enable_interrupt();       // Enable interrupts globally

    DelayMs1(2000);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0);     // Turn Off red LED P1.0
    GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7);     // Turn Off green LED P4.7

    buffer_full = 0;
    uart_cntr = 0;
    CycleCount = 0;
    t_lapse_min = 0;
    t_lapse_hour = 0;
    MinAlarmSet = 59;
    UV = 1;         // Enable UV lights
    work_time = 9;  //Hour reset for daytime counter
#ifdef Nitche_UVDI360
    state_snsr1 = GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3);       //Load current state of P2,3 either 0 or 1
#endif

    //WDTCTL = WatchDog;
    //_enable_interrupts();

    while (1)
    {

        WDTCTL = WatchDog;

        ADC12CTL0 |= ADC12SC;                       // Start sampling/conversion
        __no_operation();                           // For debugger

#ifdef uart0debug
        if (buffer_full == 1){
            for (cntr = 0; cntr < (sizeof(uart0_buf)/sizeof(uart0_buf[0])); cntr++){
                    while (!(UCA0IFG&UCTXIFG));     // USCI_A0 TX buffer ready?
                    UCA0TXBUF = uart0_buf[cntr];
            }
            buffer_full = 0;
        }
#endif

        switch (USB_getConnectionState())           // Returns the state of the USB connection, whether USB is connected or not. Check the USB state and directl main loop accordingly
        {//The USB module is powered from PC USB

            case ST_ENUM_ACTIVE:                    // Enumeration is done USB is active

                USB_present = TRUE;

                if (!USBCDC_getBytesInUSBBuffer(CDC0_INTFNUM)) {    // Returns how many bytes are in the buffer are received and ready to be read.
                    __bis_SR_register(LPM0_bits + GIE);             // Enter LPM0 sleep mode with global interrupt enabled. If no serial data is received fall asleep and wait for data. Any interrupt will wake up the system.

                }
                __enable_interrupt();                               // Enable global interrupt. That is why Minute interrupt does not enable GIE.
                ADC12CTL0 |= ADC12SC;
                __no_operation();                                   // For debugger

/*
                if (flag == 0){                                                                         //LCD print only once
                    if (!rus){
                        writeLCD((const unsigned char *) usb_msg3, (const unsigned char *) usb_msg4);   //"Connected to a  " "     USB        "
                        //DelayMs1(2000);
                    }
                    else{
                        writeLCD((const unsigned char *) usb_msg14, (const unsigned char *) usb_msg4);
                        //DelayMs1(2000);
                    }
                    DelayMs1(3000);
                }//above is printed only once
*/

                if (!USBCDC_handleReceiveCompleted(CDC0_INTFNUM)){ // This event indicates that a receive operation on interface intfNum has just been completed. Returns False

                    /*
                     *         Data Input String: DDMMYYHHMMSSTTUVCON or DDMMYYHHMMSSTTUVCOFF
                     *         TT - work time 2 digit
                     */
                    count = USBCDC_receiveDataInBuffer((uint8_t*)dataBuffer, BUFFER_SIZE, CDC0_INTFNUM);// Get data from USB buffer

                    if (count != 0){
                        // Initialize date
#ifndef debug
                        dateStr[0]  = '\n'; dateStr[1] = dataBuffer[0]; dateStr[2] = dataBuffer[1]; //month string
                        dateStr[3]  = ':';  dateStr[4] = dataBuffer[2]; dateStr[5] = dataBuffer[3]; //day string
                        dateStr[6]  = ':';  dateStr[7] = dataBuffer[4]; dateStr[8] = dataBuffer[5]; //year string

                        dateStr[9]  = dataBuffer[6]; dateStr[10] = dataBuffer[7]; //year string
                        dateStr[11] = '\n';
#endif

#ifdef debug
                        currentTime.DayOfWeek = 0x03;

                        //Read current date from the received buffer.  Day
                        Stday.MSB = dataBuffer[0] - 48;     Stday.LSB = dataBuffer[1] - 48;
                        Stday.MSB = Stday.MSB * 10;         Stday.MSB = Stday.MSB + Stday.LSB;
                        currentTime.DayOfMonth = Stday.MSB; day = Stday.MSB;

                        //Read current month from the received buffer. Month
                        Stmonth.MSB = dataBuffer[2] - 48;   Stmonth.LSB = dataBuffer[3] - 48;
                        Stmonth.MSB = Stmonth.MSB * 10;     Stmonth.MSB = Stmonth.MSB + Stmonth.LSB;
                        month = Stmonth.MSB;                currentTime.Month = Stmonth.MSB;

                        //Read current year from the received buffer. Year
                        Styear.MSB  = dataBuffer[4] - 48;   year1 = Styear.MSB;
                        year1 = year1 * 1000;               Styear.NSB1 = dataBuffer[5] - 48;
                        year2 = Styear.NSB1;                year2 = year2 * 100;
                        Styear.NSB2 = dataBuffer[6] - 48;   year3 = Styear.NSB2;
                        year3 = year3 * 10;                 Styear.LSB  = dataBuffer[7] - 48;
                        year4 = Styear.LSB;                 year4 = year1 + year2 + year3 + year4;
                        currentTime.Year = year4;           year.currentyear = year4;

                        //Read current hour from the received buffer. Hour
                        HrMSB   = dataBuffer[8] - 48;       HrLSB   = dataBuffer[9] - 48;
                        HrMSB   = HrMSB * 10;               HrMSB   = HrMSB + HrLSB;
                        currentTime.Hours = HrMSB;
                        //Read current min from the received buffer. Min
                        MinMSB  = dataBuffer[10] - 48;      MinLSB  = dataBuffer[11] - 48;
                        MinMSB = MinMSB * 10;               MinMSB = MinMSB + MinLSB;
                        currentTime.Minutes = MinMSB;
                        //Read current sec from the received buffer. Sec
                        SecMSB  = dataBuffer[12] - 48;      SecLSB  = dataBuffer[13] - 48;
                        SecMSB = SecMSB * 10;               SecMSB = SecMSB + SecLSB;
                        currentTime.Seconds = SecMSB;

                        HiLimitMSB  = dataBuffer[14] - 48;// Work time MSB dataBuffer[14] is used in to set daytime counter max value. One character
                        HiLimitLSB  = dataBuffer[15] - 48;// Work time LSB One character
                        work_time = HiLimitMSB;
                        work_time = work_time << 8;
                        work_time = work_time + HiLimitLSB;
                        
                        //Used for calibration HV VSET
                        LoLimitMSB  = dataBuffer[16] - 48;//Use as shift
                        LoLimitLSB  = dataBuffer[17] - 48;
                        //UVC ON/OFF message from PC
                        *stateUVC = NULL;
                        stateUVC = strstr (dataBuffer,"UVCON");//Returns a pointer to the first occurrence of str2 in str1, or a null pointer if str2 is not part of str1.
                        if (stateUVC){//if (statobj != NULL)
                            GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7); // Turn ON green LED P4.7
                            GPIO_setOutputHighOnPin(GPIO_PORT_P1,GPIO_PIN5);
                            UV = 1;
                        }
                        stateUVC = strstr (dataBuffer,"UVCOFF");
                        if (stateUVC){//if (statobj != NULL)
                            GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7); // Turn OFF green LED P4.7
                            GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN5);
                            UV = 0;
                        }
                        stateUVC = strstr (dataBuffer,"reset");
                        if (stateUVC){//if (statobj != NULL)
                            //GPIO_setOutputLowOnPin(GPIO_PORT_P3,GPIO_PIN5); // Set RESET pin Low
                            WDTCTL = 0xDEAD;
                        }
                         //RTC_A_initCalendar(RTC_A_BASE, &currentTime, RTC_A_FORMAT_BCD);
                        if (!stateUVC){//If command from the bot is not received update the time
                            RTC_A_initCalendar(RTC_A_BASE, &currentTime, RTC_A_FORMAT_BINARY);//Load current date and time into the calendar
                        }
                         //Setup Calendar Alarm for 5:00pm on the 5th day of the week.
                         //Note: Does not specify day of the week.
                         RTC_A_configureCalendarAlarmParam param = {0};
                         param.minutesAlarm = 0x00;
                         param.hoursAlarm = 0x17;
                         param.dayOfWeekAlarm = RTC_A_ALARMCONDITION_OFF;
                         param.dayOfMonthAlarm = 0x05;
                         RTC_A_configureCalendarAlarm(RTC_A_BASE, &param);

                         //Start RTC Clock
                         RTC_A_startClock(RTC_A_BASE);

                         //MinAlarmSet = dataBuffer[14]- 48;  //Set minute alarm text print time interval
#endif

                        count = 0; //Reset number of bytes received from the USB
                        //bCDCDataReceived_event = TRUE;
                    }

                }//End of Get data from USB. Data is received

                if (bSendTimeToHost)//OK to send data USB. Sent the current time to the host. Flag is set every second in RTC
                {

                   if (flag == 0){//To print only once
                        USBCDC_sendDataInBackground(ver, 17, CDC0_INTFNUM, 1000); // Print "\nBallast UVC v00\n"
                        {
                            _NOP();     // If it fails, it'll end up here.  Could happen if
                                        // the cable was detached after the connectionState()
                        }

                        flag = 1;       // To print the above once only
                    }
                   convertTimeBinToASCII(timeStr);          //uint8_t timeStr[10]; timeStr gets filled up in convertTimeBinToASCII
                   convertDateBinToASCII(dateStr);          // Fills up dateStr in convertDateBinToASCII function

                   usbstr [42] = ' ';
                   if (!GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN0)){ //Sensor1
                       usbstr [43] = '1';
                   }else{
                       usbstr [43] = '0';
                   }
                   usbstr [44] = ' ';
                   if (!GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5)){ //Sensor2
                       usbstr [45] = '1';
                   }else{
                       usbstr [45] = '0';
                   }
                   usbstr [46] = ' ';
                   if (!GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN2)){ //Sensor3
                       usbstr [47] = '1';
                   }else{
                       usbstr [47] = '0';
                   }
                   usbstr [48] = ' ';
                   if (!GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3)){ //Sensor4
                       usbstr [49] = '1';
                   }else{
                       usbstr [49] = '0';
                   }
                   usbstr [50] = ' ';
                   if (!GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN0)){ //Sensor5
                       usbstr [51] = '1';
                   }else{
                       usbstr [51] = '0';
                   }
                   usbstr [52] = ' ';
                   hexdec_to_str((uint32_t) t_lapse_hour, hour_run);
                   usbstr [53] = hour_run[8];//Hour singles
                   usbstr [54] = hour_run[9];//Hour decimal

                   usbstr [55] = ' ';

                   if (batch == 0){
                       usbstr [56] = 'B';
                       usbstr [57] = '0';
                   }else{
                       usbstr [56] = 'B';
                       usbstr [57] = '1';
                   }

                   if (ADCflag == 1){// Set the flag when sample is ready
                        //VoltageBinToASCII(VCH1, BatStat);
                        VoltageBinToASCII(VCH2, VSET);
                        temperature(VCH1);        //Compute temperature and
                        DelayMs1(1);
                        //CycleCount = t_lapse_min / 10;

#ifdef Nitche_UVDI360
                        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3) && (state_snsr1 == 0))
                            CycleCount ++;
                        state_snsr1 = GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3);       //Load current state of P2,3 either 0 or 1
#else
                        CycleCount = t_lapse_min / 10;
#endif
                        hexdec_to_str(CycleCount, Cycles);

                        if((ADC12IE &= ADC12IE2) == 0)
                            ADC12IE = ADC12IE2;     // Set bit3 in ADC12IE
                        if ((SecCount % 10) == 0){
                            //strcpy((char *) usbstr, (const char *)VCH1);                  // Print temperature
                            // Print temperature
                            usbstr[0] = VCH1[0];                                            //The new line
                            usbstr[1] = VCH1[3];//VCH1[5]
                            usbstr[2] = VCH1[4];//VCH1[6]
                            usbstr[3] = VCH1[5];//VCH1[7]
                            usbstr[4] = VCH1[6];//VCH1[8]
                            usbstr[5] = VCH1[7];//VCH1[9]
                            //usbstr[6] = VCH1[8];
                            //usbstr[7] = VCH1[9];
                            //usbstr[8] = VCH1[12];
                            //usbstr[9] = VCH1[13];

                            usbstr [6] = ' ';
                            //Print voltage from CH2
                            usbstr [7] = VCH2[5];
                            usbstr [8] = VCH2[6];
                            usbstr [9] = VCH2[7];
                            usbstr [10] = VCH2[8];
                            usbstr [11] = VCH2[9];
                            //usbstr [15] = VCH2[10];
                            //usbstr [16] = VCH2[11];
                            //usbstr [17] = VCH2[12];
                            //usbstr [19] = VCH2[13];

                            usbstr [12] = ' ';

                            //usbstr [27] = timeStr[0];
                            usbstr [13] = timeStr[1];
                            usbstr [14] = timeStr[2];
                            usbstr [15] = timeStr[3];
                            usbstr [16] = timeStr[4];
                            usbstr [17] = timeStr[5];
                            usbstr [18] = timeStr[6];
                            usbstr [19] = timeStr[7];
                            usbstr [20] = timeStr[8];

                            usbstr [21] = ' ';

                            usbstr [22] = dateStr[1];
                            usbstr [23] = dateStr[2];
                            usbstr [24] = dateStr[3];
                            usbstr [25] = dateStr[4];
                            usbstr [26] = dateStr[5];
                            usbstr [27] = dateStr[6];
                            usbstr [28] = dateStr[7];
                            usbstr [29] = dateStr[8];
                            usbstr [30] = dateStr[9];
                            usbstr [31] = dateStr[10];

                            usbstr [32] = ' ';

                            if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5)){//This pin will be set low after 10 hours of operation in RTC routine
                                usbstr [33] = 'E';
                            }else{
                                usbstr [33] = 'D';
                            }

                            usbstr [34] = ' ';

                            //usbstr [30] = Cycles[0];
                            //usbstr [31] = Cycles[1];
                            usbstr [35] = Cycles[2];
                            usbstr [36] = Cycles[3];
                            usbstr [37] = Cycles[4];
                            usbstr [38] = Cycles[6];
                            usbstr [39] = Cycles[7];
                            usbstr [40] = Cycles[8];
                            usbstr [41] = Cycles[9];
                            USBCDC_sendDataInBackground(usbstr, 58, CDC0_INTFNUM, 1000);    // Send Temperature
                            {
                                _NOP();     // If it fails, it'll end up here.  Could happen if
                                        // the cable was detached after the connectionState()
                            }
                        }

                   }
                   else{

                    }
                   //Report("\n UART Test");
                   //uart_print("UART test  ", 11);
                   Message("\n UART Test");
                   if (!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1)){//Simulate dead IC
                        DelayMs1(17000);//17sec delay
                   }
//Every 10 seconds reset variables, with USB
                    if ((SecCount == 10) && (flag == 1)){ //let the unit run for 10 sec. Flag is set to 1 above to indicate a single message has been printed already
                        flag = 2;
                        CBINT &= ~(CBIFG + CBIIFG);     // Clear any erroneous interrupts
                        CBINT  |= CBIE;                 // Enable comparator B interrupt
                        PulseCount = 0;
                    }
                    else if ((SecCount == 10) && (flag == 2)){  // Every 10th second print on Termite USB
                        PulseCount = 0;
                        SecCount = 0;
                    }//end of every 10 seconds
                    else{

                    }
                    bSendTimeToHost = FALSE;
                }
               break;
//NOUSB-NOUSB-NOUSB-NOUSB-NOUSB-NOUSB-NOUSB-NOUSB USB NOT connected #######################################################
               // These cases are executed while your device is disconnected from
               // the host (meaning, not enumerated); enumerated but suspended
               // by the host, or connected to a powered hub without a USB host
               // present. If unit is not connected to USB it has to go to sleep
               case ST_PHYS_DISCONNECTED:               // No USB connection
                   //Sleep in here
                   __bis_SR_register(LPM0_bits + GIE);

                   break;
               case ST_ENUM_SUSPENDED:
               case ST_PHYS_CONNECTED_NOENUM_SUSP:
                   //Sleep in here
                   __bis_SR_register(LPM0_bits + GIE);
                   _NOP();
               break;

                    // The default is executed for the momentary state
                    // ST_ENUM_IN_PROGRESS.  Usually, this state only last a few
                    // seconds.  Be sure not to enter LPM3 in this state; USB
                    // communication is taking place here, and therefore the mode must
                    // be LPM0 or active-CPU.
               case ST_ENUM_IN_PROGRESS:

               default:
                   if (flag == 0){                      // Print the following only once
                       if (!rus){
                            //writeLCD((const unsigned char *) usb_msg3, (const unsigned char *) usb_msg4);//"Connected to a  " "     USB        "
                        }
                        else{
                            //writeLCD((const unsigned char *) usb_msg15, (const unsigned char *) usb_msg4);// HE PODKLYUCHEN K
                        }
                        DelayMs1(3000);
                       flag = 1;
                   }

                   USB_present = FALSE;
                   __enable_interrupt();//Enable global interrupt. That is why Minute interrupt does not enable GIE.

//No USB Loop - Continuous part NoUSB ----------------------------------------------------------------------------
                   //Continuous part USB DISconnected
                   if ((SecCount == 2) && (flag == 1)){
                       flag = 2;
                       CBINT &= ~(CBIFG + CBIIFG);   // Clear any errant interrupts
                       CBINT  |= CBIE;
                       PulseCount = 0;
                   }
                   else if ((SecCount == 10) && (flag == 2)){
                       PulseCount = 0;
                   }
                   else if (flag == 2){
                       if (ADCflag == 1){// Set the flag when sample is ready

                       }
                       else{//No ADC sample

                       }
                       if((ADC12IE &= ADC12IE2) == 0)
                           ADC12IE = ADC12IE2;
                   }
                   else{
                       //Sleep in here
                       __bis_SR_register(LPM0_bits + GIE);
                   }
                   break;
                }
            __bis_SR_register(LPM0_bits + GIE + LPM4_bits);

    }  //while(1)

}  //main()


// Starts a real-time clock on TimerA_0.  Earlier we assigned ACLK to be driven
// by the REFO, at 32768Hz.  So below we set the timer to count up to 32768 and
// roll over; and generate an interrupt when it rolls over.
void initRTC(void)
{
    TA0CCR0 = 32768;
    TA0CTL = TASSEL_1+MC_1+TACLR; // ACLK, count to CCR0 then roll, clear TAR
    TA0CCTL0 = CCIE;              // Gen int at rollover (TIMER0_A0 vector)
}


void writeLCD(const unsigned char *strL1, const unsigned char *strL2){
    writecom4_NHD_C0216CU_FSW_GBW_3V3(0x80);
    DelayMs1(10);
    for(i=0;i<16;i++){
        writedata4_NHD_C0216CU_FSW_GBW_3V3(strL1[i]);// Print: "Line1"
        DelayMs1(30);
    }
    writecom4_NHD_C0216CU_FSW_GBW_3V3(0xc0);
    DelayMs1(10);
    for(i=0;i<16;i++){
        writedata4_NHD_C0216CU_FSW_GBW_3V3(strL2[i]);// Print: "Line2"
        DelayMs1(30);
    }
}
//*********************************** INTERRUPTS ********************** INTERRUPTS ****************** INTERRUPTS *******
//**********************************************************************************************************************
// Timer0 A0 interrupt service routine.  Generated when TimerA_0 (real-time clock)
// rolls over from 32768 to 0, every second.
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not found!
#endif
{
   // if (sec++ == 60)
   // {
   //     sec = 0;
   //     if (min++ == 60)
   //     {
   //         min = 0;
   //         if (hour++ == 24)
   //         {
   //             hour = 0;
   //         }
   //     }
   // }

    //bSendTimeToHost = TRUE;                 // Time to update
    //__bic_SR_register_on_exit(LPM3_bits);   // Exit LPM
}

/*
 * ======== UNMI_ISR ========
 */
#if defined(__TI_COMPILER_VERSION__) || (__IAR_SYSTEMS_ICC__)
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR (void)
#elif defined(__GNUC__) && (__MSP430__)
void __attribute__ ((interrupt(UNMI_VECTOR))) UNMI_ISR (void)
#else
#error Compiler not found!
#endif
{
        switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG )) {
        case SYSUNIV_NONE:
                __no_operation();
                break;
        case SYSUNIV_NMIIFG:
                __no_operation();
                break;
        case SYSUNIV_OFIFG:

                UCS_clearFaultFlag(UCS_XT2OFFG);
                UCS_clearFaultFlag(UCS_DCOFFG);
                SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
                break;
        case SYSUNIV_ACCVIFG:
                __no_operation();
                break;
        case SYSUNIV_BUSIFG:
                // If the CPU accesses USB memory while the USB module is
                // suspended, a "bus error" can occur.  This generates an NMI.  If
                // USB is automatically disconnecting in your software, set a
                // breakpoint here and see if execution hits it.  See the
                // Programmer's Guide for more information.
                SYSBERRIV = 0;  // Clear bus error flag
                USB_disable();  // Disable
        }
}

// Comp_B ISR Interrupt --------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=COMP_B_VECTOR
__interrupt void Comp_B_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(COMP_B_VECTOR))) Comp_B_ISR (void)
#else
#error Compiler not supported!
#endif
{
  //CBCTL1 ^= CBIES;            // Toggles interrupt edge
  CBINT &= ~CBIFG;              // Clear Interrupt flag
  //GPIO_toggleOutputOnPin(GPIO_PORT_P2,GPIO_PIN0);
  PulseCount++;             // Every time comparator output
      // __bic_SR_register_on_exit
      //__bis_SR_register_on_exit(LPM4_bits + GIE);
      //bSendTimeToHost = TRUE;                 // Time to update
      //__bic_SR_register_on_exit(LPM3_bits);   // Exit LPM
      //__bic_SR_register_on_exit(LPM0_bits + GIE );   // Exit LPM0
      //__bic_SR_register_on_exit(CPUOFF);
}

// RTC (real time clock) Interrupt interrupt on every minute/second
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(RTC_VECTOR)))
#endif
void RTC_A_ISR(void)    // Happens every second
{
    newTime = RTC_A_getCalendarTime(RTC_A_BASE); //Update the newTime variable. All time data is here

    switch(__even_in_range(RTCIV,16))
    {
    case 0: break;      //No interrupts
    case 2:             //RTCRDYIFG - RTC ready; Interrupt Flag: RTCRDYIFG
        /*
         * LPM0 – CPU and MCLK are shutoff. SMCLK and ACLK remain active.
         * LPM1 – CPU and MCLK are off, as in LPM1, but DCO and DC generator are disabled if the DCO is not used for SMCLK. ACLK is active.
         * LPM2 – CPU, MCLK, SMCLK and DCO are disabled, while DC generator is still enabled. ACLK is active.
         * LPM3 – CPU, MCLK, SMCLK, DCO and DC generator are disabled. ACLK is active.
         * LPM4 – CPU and all clocks disabled
         * An easy way to safely read the real-time clock registers is to use the RTCRDYIFG interrupt flag. Setting
         * RTCRDYIE enables the RTCRDYIFG interrupt. Once enabled, an interrupt is generated based on the
         * rising edge of the RTCRDY bit, causing the RTCRDYIFG to be set. At this point, the application has
         * nearly a complete second to safely read any or all of the real-time clock registers. This synchronization
         * process prevents reading the time value during transition. The RTCRDYIFG flag is reset automatically
         * when the interrupt is serviced, or can be reset with software.
         */
        //Toggle P1.0 every second USB_present == FALSE Run == FALSE
        GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);//Toggle red LED P1.0
        sec = newTime.Seconds;
        hour = newTime.Hours;
        min = newTime.Minutes;
        day = newTime.DayOfMonth;
        month = newTime.Month;
        year.currentyear = newTime.Year;

        bSendTimeToHost = TRUE;                 // Time to update, enable USB transfer
        SecCount++;     //Used in USB print
        SecCount1++;    //Used here
        if (SecCount1 == 35){ // every 30 seconds:0-29 go to sleep and wake up on the next minute
            RTC_A_clearInterrupt(RTC_A_BASE, RTCRDYIFG + RTCTEVIFG + RTCAIFG); // Clear interrupt flags
            /*
             * RTCRDYIE - Real-time clock read ready interrupt enable
             * RTCAIE - Real-time clock alarm interrupt enable. This bit remains cleared when in counter mode (RTCMODE = 0)
             * An easy way to safely read the real-time clock registers is to use the RTCRDYIFG interrupt flag. Setting
             * RTCRDYIE enables the RTCRDYIFG interrupt. Once enabled, an interrupt is generated based on the
             * rising edge of the RTCRDY bit, causing the RTCRDYIFG to be set.
             */

            //Puts unit to sleep
            //RTC_A_disableInterrupt(RTC_A_BASE, RTCRDYIE + RTCAIE);    //Disable RTC ready (RTCRDYIE) 1 sec interrupt. Disable RTCA interrupt

            CBINT = 0;                          // Disable CompB Interrupt on rising edge of CBIFG (CBIES=0)
            SecCount1 = 0;//Reset secnds count
            SecCount = 0;
            GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0); // Turn OFF red LED P1.0
        }

        __bic_SR_register_on_exit(LPM0_bits + GIE);     //
        break;
    case 4: //Minute Interrupt. RTCEVIFG - 04h = Interrupt Source: RTC interval timer; Interrupt Flag: RTCTEVIFG. Interrupts every minute - used to wake up system with an alarm

        if ((t_lapse_hour <=  work_time) && (UV == 1)){//Daytime counter and remote UV enable
            ++t_lapse_min;                                          //time lapse in minutes to keep track of working time
            if ((t_lapse_min%5) == 0){
                GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN5);    // Ballast control PIN
                GPIO_toggleOutputOnPin(GPIO_PORT_P4,GPIO_PIN7);     // Toggle green LED P4.7
                if (!GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN5)){ //Read status of the Ballast control PIN
                    batch = 0;
                }else{
                    batch = 1;
                }
            }
        }else{//if t_lapse_hour >= work_time greater
            GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);        // Ballast control PIN
            GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7);         // Toggle green LED P4.7
        }

        if (++MinCount > 59)
        {
            SecCount1 = 0;
            MinCount = 0;
            t_lapse_hour++;         // Increment hour count - up to 10 hours a day
            if (t_lapse_hour > 23)  // Reset t_lapse_hour after 24 hours
                t_lapse_hour = 0;

            //Comparator B interrupt
            CBINT &= ~(CBIFG + CBIIFG);   // Clear any errant interrupts
            CBINT  |= CBIE;               // Enable CompB Interrupt on rising edge of CBIFG (CBIES=0)

            RTC_A_clearInterrupt(RTC_A_BASE, RTCRDYIFG + RTCTEVIFG + RTCAIFG);
            RTC_A_enableInterrupt(RTC_A_BASE, RTCRDYIE + RTCTEVIE + RTCAIE);//Enable RTCRDYIE - RTC ready Interrupt. Occurs every second
        }
        //__no_operation();
        __bic_SR_register_on_exit(LPM0_bits);   // The next minute alarm will wake the system.
        break;
    case 6:             //RTCAIFG
        //Interrupts 5:00pm on 5th day of week
        __no_operation();
        break;
    case 8: break;      //RT0PSIFG
    case 10: break;     //RT1PSIFG
    case 12: break;     //Reserved
    case 14: break;     //Reserved
    case 16: break;     //Reserved
    default: break;
    }
}


// Port 1 ISR Interrupt (push button) ------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{
  P1IFG &= ~BIT1;                         // Clear P1.1 IFG
  P1IE &= ~ BIT1;                         // Clear P1.1 IE

  P1IFG &= ~BIT3;                         // Clear P1.3 IFG
  P1IE &= ~ BIT3;                         // Clear P1.3 IE

  RTC_A_clearInterrupt(RTC_A_BASE, RTCRDYIFG + RTCTEVIFG + RTCAIFG); // RTCRDYIFG - int every second. Clear interrupt flags
  //The RTCTEV bits select the respective trigger event. RTCTEV event can trigger an interrupt by setting the RTCTEVIE bit
  RTC_A_enableInterrupt(RTC_A_BASE, RTCRDYIE + RTCTEVIE + RTCAIE);// Enable interrupts

  //USBCDC_sendDataInBackground(message1, 28, CDC0_INTFNUM, 1000);//"\nRTC Interrupts are enabled\n"
    //flag = 99;
    //rus = 1;
  //Comparator B interrupt
  //CBINT &= ~(CBIFG + CBIIFG);   // Clear any errant interrupts
  //CBINT  |= CBIE;               // Enable CompB Interrupt on rising edge of CBIFG (CBIES=0)

  //Puts unit to sleep and keeps RTCTEVIE RTC event interrupt enabled
  //RTC_A_disableInterrupt(RTC_A_BASE, RTCRDYIE + RTCTEVIE);    //Disable RTC ready (RTCRDYIE) 1 sec interrupt. Disable RTCA interrupt
  //RTC_A_disableInterrupt(RTC_A_BASE, RTCRDYIE + RTCTEVIE + RTCAIE);// Disable RTC interrupt and RTC event interrupt
  //RTC_A_disableInterrupt(RTC_A_BASE, RTCTEVIE);   //Disable Event interrupt
  //UCA0IE |= UCRXIE;             // Enable USCI_A0 RX interrupt

  //__bic_SR_register_on_exit(LPM4_bits);   // Exit LPM4
  __bic_SR_register_on_exit(LPM0_bits + GIE );   // Bit clear Special Register: Exit LPM0 with GIE cleared (disabled)
}



#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(ADC12IV,34))
  {//Reading the ADC12MEMx register clears the respective interrupt flag
  case  0:
      ADCflag = 1;//To separate ADC result interrupts
      //__bic_SR_register_on_exit(LPM0_bits + GIE);
      break;                           // Vector  0:  No interrupt
  case  2:                              // ADC12MEMx overflow; Interrupt Flag: –; Interrupt
      ADCflag = 1;
      //__bic_SR_register_on_exit(LPM0_bits + GIE);
      break;                           // Vector  2:  ADC overflow
  case  4:                              // Conversion time overflow; Interrupt Flag
      ADCflag = 1;
      //__bic_SR_register_on_exit(LPM0_bits + GIE); //01152018
      break;
  case  6:                                          // 06h = Interrupt Source: ADC12MEM0 interrupt flag; Interrupt Flag: ADC12IFG0
    ADCflag = 1;                                    // Set the flag when sample is ready
    ADC12IFG &= ~ADC12IFG0;
    ADC12IE &= ~ADC12IE0;
    //BatStat = ADC12MEM0;
    //VSET = ADC12MEM2;

    // if (ADC12MEM0 >= 0x7ff)                      // ADC12MEM = A0 > 0.5AVcc? Battery voltage
      //P1OUT |= BIT0;                              // P1.0 = 1
    //else
      //P1OUT &= ~BIT0;                             // P1.0 = 0
    __bic_SR_register_on_exit(LPM0_bits + GIE);     // Exit LPM0 with GIE set
    break;
  case  8:                                          // 08h = Interrupt Source: ADC12MEM1 interrupt flag; Interrupt Flag: ADC12IFG1
      ADCflag = 1;                                  // Set the flag when sample is ready
      //BatStat = ADC12MEM1;
      ADC12IFG &= ~ADC12IFG1;
      ADC12IE &= ~ADC12IE1;                         // Have to disable ADC it inhibits USB
      __bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set
      break;                                        // Vector  8:  ADC12IFG1
  case 10:                                          // 0Ah = Interrupt Source: ADC12MEM2 interrupt flag; Interrupt Flag: ADC12IFG2
      ADCflag = 1;
      BatStat   = ADC12MEM1;
      VSET      = ADC12MEM2;
      ADC12IFG &= ~ADC12IFG2;
      ADC12IE &= ~ADC12IE2;
      __bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set
      break;
  case 12:
      ADC12IFG &= ~ADC12IFG3;
      ADC12IE &= ~ADC12IE3;
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set
      break;                           // Vector 12:  ADC12IFG3
  case 14:
      ADC12IFG &= ~BIT4;
      //USBCDC_sendDataInBackground(messageADC8, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set 01152018
      break;                           // Vector 14:  ADC12IFG4
  case 16:
      ADC12IFG &= ~BIT5;
      //USBCDC_sendDataInBackground(messageADC9, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set 01152018
      break;                           // Vector 16:  ADC12IFG5
  case 18:
      //ADCflag = 1;                        // Set the flag when sample is ready
      //__bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
      ADC12IFG &= ~BIT6;
      //USBCDC_sendDataInBackground(messageADC10, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set 01152018
      break;                           // Vector 18:  ADC12IFG6
  case 20:
      ADC12IFG &= ~BIT7;
      //USBCDC_sendDataInBackground(messageADC11, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set
      break;                           // Vector 20:  ADC12IFG7
  case 22:
      ADC12IFG &= ~BIT8;
      //USBCDC_sendDataInBackground(messageADC12, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set 01152018
      break;                           // Vector 22:  ADC12IFG8
  case 24:
      ADC12IFG &= ~BIT9;
      //USBCDC_sendDataInBackground(messageADC13, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set 01152018
      break;                           // Vector 24:  ADC12IFG9
  case 26:
      ADC12IFG &= ~BITA;
      //USBCDC_sendDataInBackground(messageADC14, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE setbreak; 01152018
  case 28:
      ADC12IFG &= ~BITB;
      //USBCDC_sendDataInBackground(messageADC15, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set 01152018
      break;                           // Vector 28:  ADC12IFG11
  case 30:
      ADC12IFG &= ~BITC;
      //USBCDC_sendDataInBackground(messageADC16, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set 01152018
      break;                           // Vector 30:  ADC12IFG12
  case 32:
      ADC12IFG &= ~BITD;
      //USBCDC_sendDataInBackground(messageADC17, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set 01152018
      break;                           // Vector 32:  ADC12IFG13
  case 34:
      ADC12IFG &= ~BITE;
      //USBCDC_sendDataInBackground(messageADC18, 15, CDC0_INTFNUM, 1000);
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE set 01152018
      break;                           // Vector 34:  ADC12IFG14
  default:
      ADCflag = 1;
      //USBCDC_sendDataInBackground(messageADC, 15, CDC0_INTFNUM, 1000);//"\nADC Interrupt\n"
      //__bic_SR_register_on_exit(LPM0_bits + GIE);   // Exit LPM0 with GIE cleared (disabled) 01152018
      break;
  }
}


// Echo back RXed character, confirm TX buffer is ready first
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                                 // Vector 0 - no interrupt uart_cntr
  case 2:                                       // Vector 2 - RXIFG
#ifdef uart0debug
      if (uart_cntr < 7){
          uart0_buf[uart_cntr] = UCA0RXBUF;     //Get character UART1
          uart_cntr++;
      }else{
          uart_cntr = 0;                        //Buffer full, reset counter
          buffer_full = 1;                      //Set buffer full flag
      }
      while (!(UCA0IFG&UCTXIFG));               //USCI_A0 TX buffer ready?
      UCA0TXBUF = UCA0RXBUF;
#endif
      if (buffer_full == 2){                        // Command to the sensor has been issued.
          if (uart_cntr < 4){                       // Start receiving response from the sensor
              uart0_buf[uart_cntr] = UCA0RXBUF;     // Read current character to uart0_buf
              uart_cntr++;
          }else{
              uart_cntr = 0;                        // Buffer full, reset counter
              buffer_full = 1;                      // Set buffer full flag
          }
      }else{
          _NOP();
      }
      break;
  case 4:break;                                 // Vector 4 - TXIFG
  default: break;
  }
}

