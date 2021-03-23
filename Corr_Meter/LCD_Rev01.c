/*
 * LCD.c
 *
 *  Created on: Jan 13, 2017
 *      Author: rchak
 *      https://www.pantechsolutions.net/how-to-interface-lcd-with-msp430f5529-msp430-development-board
 *		http://www.digikey.com/product-detail/en/newhaven-display-intl/NHD-C0216CU-FSW-GBW-3V3/NHD-C0216CU-FSW-GBW-3V3-ND/2165870
 *		http://www.digikey.com/product-detail/en/newhaven-display-intl/NHD-0216HZ-FSW-FBW-33V3C/NHD-0216HZ-FSW-FBW-33V3C-ND/2773591
 *		http://www.digikey.com/product-detail/en/newhaven-display-intl/NHD-0216SZW-BG5/NHD-0216SZW-BG5-ND/2626458
 *
 */

#include <msp430x552x.h>
#include <string.h>
#include "driverlib.h"
#include "hal.h"

extern void DelayMs(int Ms);
extern void DelayMs1(int Ms);

void writecom_NHD_C0216CU_FSW_GBW_3V3(unsigned char cmd);
void writedata_NHD_C0216CU_FSW_GBW_3V3(unsigned char byte);
unsigned char LCDbusy_NHD_C0216CU_FSW_GBW_3V3();

// 4 - bit
void writecom4_NHD_C0216CU_FSW_GBW_3V3(char cmd);
void writedata4_NHD_C0216CU_FSW_GBW_3V3(char byte);
unsigned char LCDbusy4_NHD_C0216CU_FSW_GBW_3V3();

extern unsigned char cmdcntr;

extern uint8_t RecVal[28];
extern uint8_t RecStr[56];
extern uint8_t RecCntrl[28];
extern uint8_t RecCntrlStr[56];

extern void convertTwoDigBinToASCII(uint8_t bin, uint8_t* str);

//void convertTwoDigBinToASCII(RecVal[0], RecStr[0]);

void latch(){                           // command to latch E for all displays
	GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 1;			SETB E

	DelayMs1(1);

	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 0;			RESETB E
}

void init_NHD_C0216CU_FSW_GBW_3V3() //initialize the LCD 8 bit
{

	//P3 = 1;

	//P1 = 1;
	//P7OUT = 0x01;

	//RST = 0; //RESET
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN2); //RESET

	//delay(2);
	DelayMs (2);

	//RST = 1; //end reset
	GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6); //end reset

	//delay(20);
	DelayMs (40);

	writecom_NHD_C0216CU_FSW_GBW_3V3(0x38); //wake up 	38H 8 bit,N=1,5*7dot
	DelayMs (2);
	writecom_NHD_C0216CU_FSW_GBW_3V3(0x38); //wake up	38H 8 bit,N=1,5*7dot
	DelayMs (2);
	writecom_NHD_C0216CU_FSW_GBW_3V3(0x38); //wake up	38H 8 bit,N=1,5*7dot
	DelayMs (2);
	writecom_NHD_C0216CU_FSW_GBW_3V3(0x39); //function set
	DelayMs (2);
	writecom_NHD_C0216CU_FSW_GBW_3V3(0x14); //internal osc frequency
	DelayMs (2);
	writecom_NHD_C0216CU_FSW_GBW_3V3(0x56); //power control
	DelayMs (2);
	writecom_NHD_C0216CU_FSW_GBW_3V3(0x6D); //follower control
	DelayMs (2);
	writecom_NHD_C0216CU_FSW_GBW_3V3(0x70); //contrast
	DelayMs (2);
	writecom_NHD_C0216CU_FSW_GBW_3V3(0x0C); //display on
	DelayMs (2);
	writecom_NHD_C0216CU_FSW_GBW_3V3(0x06); //entry mode
	DelayMs (2);
	writecom_NHD_C0216CU_FSW_GBW_3V3(0x01); //clear
	DelayMs (10);
}


//-------------------------------------------------------------------------------------------
/*
 *
 * GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN5); // RS
 * GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4); // RW
 * GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN3); // En
 * GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN2); // XRESET
 *
*/
void writecom_NHD_C0216CU_FSW_GBW_3V3(unsigned char cmd) //8 bit
{
	//RW = 0; //Write
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN4); 	// RW = 0 - Write  		CLR RW
	//RS = 0; //Command
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN5); 	// RS = 0 - Command 	CLR RS
	//E = 1;
	GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 1;				SETB E
	//P1 = cmd;
	P7OUT = cmd;										//						MOV P1,A
	//E = 0;
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 0;				CLR E
}

void writedata_NHD_C0216CU_FSW_GBW_3V3(unsigned char byte)//8 bit
{
	//RW = 0; //Write
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN4); 	// RW = 0 - Write
	//RS = 1; //Data
	GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN5); 	// RS = 1 - Data
	//E = 1;
	GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 1;
	//P1 = byte;
	P7OUT = byte;
	//E = 0;
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 0;
}

unsigned char LCDbusy_NHD_C0216CU_FSW_GBW_3V3(){
	P7DIR = 0x7F;
	DelayMs (2);
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN5); 	// RS = 0 - Command
	GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN4); 	// RW = 1 - Read
	return (P7IN & 0x80);
}


extern uint8_t bSendRec;
/*
 * Write command
 */
void writecom4_NHD_C0216CU_FSW_GBW_3V3(char cmd)//4 bit
{
	char temp;
	temp = cmd;
	bSendRec = 1;

	// Send High Nybble
	temp = ((temp >> 4) & 0x0F);		//Shift H-Nybble to L-Nybble

	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 0;				RESETB E ok
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN4); 	// RW = 0 - Write  		CLR RW ok
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN5); 	// RS = 0 - Command 	CLR RS ok
	DelayMs1(10);

	//GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 1;				SETB E

	DelayMs1(1);

	P2OUT = (P2OUT & 0xF0) | temp; 						// Apply H-Nybble to 2.0 - 2.3 MOV P1,A ok
	//The following 2 instructions read current Port 2 data (P2OUT) and prints it to a USB terminal
	RecVal[cmdcntr] = P2OUT;							//Record sent command to RecVal buffer
	convertTwoDigBinToASCII(RecVal[cmdcntr], &RecStr[(2*cmdcntr)]); // cmdcntr = 0 to 12. Convert command to printable characters

	DelayMs1(1);
	latch();		//ok

	//GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 0;				RESETB E

	//DelayMs1(1000);//----------------------------------------------------------------------------------------

	// Send Low Nybble
	//GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 1;				SETB E
	DelayMs1(1);

	P2OUT = (cmd & 0x0F);				// Apply L-Nybble to 2.0 - 2.3				MOV P1,A
	//The following 2 instructions read current Port 2 data (P2OUT) and prints it to a USB terminal
	RecVal[(cmdcntr + 13)] = P2OUT;//Record sent command to RecVal buffer. Convert command to printable characters
	convertTwoDigBinToASCII(RecVal[(cmdcntr + 13)], &RecStr[(cmdcntr+13)*2]); //25 + 12 = 37 27 + 24 = 51

	DelayMs1(1);

	//RecStr[1] = (cmd & 0x0F);
	//convertTwoDigBinToASCII(RecVal[1], RecStr[1]);

	latch();

	//GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 0;				RESETB E


	DelayMs1 (2);

}

void writedata4_NHD_C0216CU_FSW_GBW_3V3(char byte)
{

	char temp;

	temp = byte;

	temp = ((temp >> 4) & 0x0F);			//Shift variable 'byte' MS nibble to LS nibble

	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 0;				RESETB E
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN4); 	// RW = 0 - Write
	GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN5); 	// RS = 1 - Data

	DelayMs1(1);

	//latch();

	//GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 1;				SETB E
	//delay(1);
	//DelayMs1(1);

	P2OUT = (P2OUT & 0xF0) | temp;			//Output HN (P2OUT & 0xF0)|((cmd>>4) & 0x0F)
	latch();

	//GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 0;				RESETB E
	//delay(1);
	//DelayMs1(1000);

	//------------------------------------------------------------------------------------------

	//GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 1;				SETB E
	//delay(1);
	//DelayMs1(1);

	P2OUT = (byte & 0x0F);							//Output LN (P2OUT & 0xF0)|(byte & 0x0F)
	latch();

	//GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 0;				RESETB E
	//delay(1);
	//DelayMs1(1);

	//latch();

	//DelayMs1(1);
}

unsigned char LCDbusy4_NHD_C0216CU_FSW_GBW_3V3(){

	unsigned char busy;

	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN3);		//In 2.3

	DelayMs (2);

	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN5); 	// RS = 0 - Command

	GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN4); 	// RW = 1 - Read

	busy = (P2IN & 0x04);

	/*
	 * P2.3
	 * 0 1 2 3   4 5 6 7
	 * 0 0 0 1   0 0 0 0
	 *
	 */

	GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);		//Out 2.3

	DelayMs (2);

	return busy;
}

void init4_NHD_C0216CU_FSW_GBW_3V3(void) //initialize the LCD. Was - extern void init4_NHD_C0216CU_FSW_GBW_3V3();
{


	//RST = 0; //RESET
	//GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN2); //RESET

	//delay(2);
	//DelayMs1 (100);

	//RST = 1; //end reset
	//GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN2); 	//end reset

	//P2.0 - DB4
	//P2.1 - DB5
	//P2.2 - DB6
	//P2.3 - DB7

	DelayMs1 (1000);
	P2OUT = 0x03;		//1
	latch();
	DelayMs1 (100);
	latch();			//2
	DelayMs1 (100);
	latch();			//3
	DelayMs1 (100);


	P2OUT = 0x02;
	latch();
	DelayMs1 (100);

	//GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn ON green LED P4.7
	//GPIO_setOutputHignOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn ON red LED P1.0

	//cmdcntr = 0;	//Used to print data on PC
	//writecom4_NHD_C0216CU_FSW_GBW_3V3(0x30); //wake up 	38H 8 bit,N=1,5*7dot
	//DelayMs1 (100);

	//GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn OFF green LED P4.7
	//GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn OFF red LED P1.0

	//cmdcntr = 1;	//Used to print data on PC
	//writecom4_NHD_C0216CU_FSW_GBW_3V3(0x38); //wake up	38H 8 bit,N=1,5*7dot
	//DelayMs1 (100);

	//GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn ON green LED P4.7
	//GPIO_setOutputHignOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn ON red LED P1.0

	//cmdcntr = 2;	//Used to print data on PC
	//writecom4_NHD_C0216CU_FSW_GBW_3V3(0x38); //wake up	38H 8 bit,N=1,5*7dot
	//DelayMs1 (100);

	//GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn OFF green LED P4.7
	//GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn OFF red LED P1.0

	//cmdcntr = 3;	//Used to print data on PC
	//writecom4_NHD_C0216CU_FSW_GBW_3V3(0x38); //wake up	38H 8 bit,N=1,5*7dot
	//DelayMs1 (100);

	//GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn ON green LED P4.7
	//GPIO_setOutputHignOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn ON red LED P1.0

	cmdcntr = 0;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x28); //wake up	28H 4 bit,N=1,5*7dot
	DelayMs1 (100);

	//GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn OFF green LED P4.7
	//GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn OFF red LED P1.0

	cmdcntr = 1;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x10); //wake up	29H 4 bit,N=1,5*7dot
	DelayMs1 (100);

	//GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn ON green LED P4.7
	//GPIO_setOutputHignOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn ON red LED P1.0

	cmdcntr = 2;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x0F); //Internal OSC
	DelayMs1 (100);

	//GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn OFF green LED P4.7
	//GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn OFF red LED P1.0

	cmdcntr = 3;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x06); //Contrast set 78H
	DelayMs1 (100);

	//GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn ON green LED P4.7
	//GPIO_setOutputHignOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn ON red LED P1.0

	//cmdcntr = 8;	//Used to print data on PC
	//writecom4_NHD_C0216CU_FSW_GBW_3V3(0x5E); //Power/ICON/Contrast 5E
	//DelayMs1 (100);

	//GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn OFF green LED P4.7
	//GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn OFF red LED P1.0

	//cmdcntr = 9;	//Used to print data on PC
	//writecom4_NHD_C0216CU_FSW_GBW_3V3(0x6A); //Follower control 6A
	//DelayMs1 (500);//<--------------------------------------------------250ms min

	//GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn ON green LED P4.7
	//GPIO_setOutputHignOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn ON red LED P1.0

	//cmdcntr = 10;	//Used to print data on PC
	//writecom4_NHD_C0216CU_FSW_GBW_3V3(0x0C); //DISPLAY ON 0C
	//DelayMs1 (100);

	//GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn OFF green LED P4.7
	//GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn OFF red LED P1.0

	//cmdcntr = 11;	//Used to print data on PC
	//writecom4_NHD_C0216CU_FSW_GBW_3V3(0x01); //CLEAR DISPLAY
	//DelayMs1 (100);

	//GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn ON green LED P4.7
	//GPIO_setOutputHignOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn ON red LED P1.0

	//cmdcntr = 12;	//Used to print data on PC
	//writecom4_NHD_C0216CU_FSW_GBW_3V3(0x06); //ENTRY MODE SET
	//DelayMs1 (100);

	//GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7); 	// Turn OFF green LED P4.7
	//GPIO_setOutputLowOnPin(GPIO_PORT_P1,GPIO_PIN0); 	// Turn OFF red LED P1.0


	//GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7); // Turn ON green LED P4.7

}



// On power up, the display is initilaized as:
// 1. Display clear
// 2. Function set:
//    DL="1": 8-bit interface data
//    N="0": 1-line display
//    F="0": 5 x 8 dot character font
// 3. Power turn off
//    PWR=”0”
// 4. Display on/off control: D="0": Display off C="0": Cursor off B="0": Blinking off
// 5. Entry mode set
//    I/D="1": Increment by 1
//    S="0": No shift
// 6. Cursor/Display shift/Mode / Pwr
//    S/C=”0”, R/L=”1”: Shifts cursor position to the right
//    G/C=”0”: Character mode
//    Pwr=”1”: Internal DCDC power on
/*
 * http://dokuwiki.ehajo.de/artikel:displays:weh001602_spi_democode
 * sende_befehl(0b00111001);	// Function set: 8bit, 2 Zeilen, 5x8 Punkte, Westeurop. Charset
	_delay_us(500);
	sende_befehl(0b00001100);	// Display on. Display an, Cursor aus, Blinken aus.
	_delay_us(500);
	sende_befehl(0b00000001);	// Display clear
	_delay_us(500);
	sende_befehl(0b00000010);	// Display home
	_delay_us(500);
	sende_befehl(0b00000110);	// Entry mode: Dekrement, no shift.
	_delay_us(500);
 */


void WriteIns(unsigned char instruction) //For WEH1602 4bit
{
	unsigned char temp;

	temp = instruction;//0xHL

	//temp = ((temp << 4) & 0xF0);//0xL0

	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN3); 	// E = 0;				RESETB E ok
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN4); 	// RW = 0 - Write  		CLR RW ok
	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN5); 	// RS = 0 - Command 	CLR RS ok

	//Data_BUS = instruction & 0xf0;
	/*
	 * P2.0 - DB4
	 * P2.1 - DB5
	 * P2.2 - DB6
	 * P2.3 - DB7
	 */
	P2OUT = (P2OUT & 0xF0) | temp;//Write 0xHL, writing H to P2OUT

	latch();		//E = 1 DelayMs1(1) E = 0

}



void init4_WS_0010_5V(void) //For WEH1602 4bit
{

	//P2.0 - DB4
	//P2.1 - DB5
	//P2.2 - DB6
	//P2.3 - DB7

	DelayMs1 (1000);// wait for 1 sec

	WriteIns(0x00);
	DelayMs1 (200);

	WriteIns(0x00);
	DelayMs1 (200);

	WriteIns(0x00);
	DelayMs1 (200);

	WriteIns(0x00);
	DelayMs1 (200);

	WriteIns(0x00);
	DelayMs1 (200);




	WriteIns(0x20);//function set //do it only once change to 0x22 for russian
	DelayMs1 (200);

	/*
	   Switch to graphics
	   OLED1.command(0x08);//turn off display
	   OLED1.command(0x1F);//switch to graphics
	   OLED1.command(0x01);//clear display
	   OLED1.command(0x08|0x04);//turn on display
	   Switch to text
	   OLED1.command(0x08);//turn off display
   	   OLED1.command(0x17);//switch to text
   	   OLED1.command(0x01);//clear display
   	   OLED1.command(0x04 | 0x08);//turn on display
	 */

	cmdcntr = 0;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x28); //function set.
	DelayMs1 (200);

	cmdcntr = 1;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x0C);//display on
	DelayMs1 (200);

	cmdcntr = 2;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x06);//entry mode set
	DelayMs1 (200);

	cmdcntr = 3;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x02);
	DelayMs1 (200);

	cmdcntr = 3;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x01);//clear display
	DelayMs1 (200);


}


void init4_RUS_WS_0010_5V(void) //For WEH1602 4bit
{

	//P2.0 - DB4
	//P2.1 - DB5
	//P2.2 - DB6
	//P2.3 - DB7

	DelayMs1 (1000);// wait for 1 sec

	WriteIns(0x00);
	DelayMs1 (200);

	WriteIns(0x00);
	DelayMs1 (200);

	WriteIns(0x00);
	DelayMs1 (200);

	WriteIns(0x00);
	DelayMs1 (200);

	WriteIns(0x00);
	DelayMs1 (200);

	WriteIns(0x20);//function set //do it only once change to 0x22 for Russian
	DelayMs1 (200);

	/*
	   Switch to graphics
	   OLED1.command(0x08);//turn off display
	   OLED1.command(0x1F);//switch to graphics
	   OLED1.command(0x01);//clear display
	   OLED1.command(0x08|0x04);//turn on display
	   Switch to text
	   OLED1.command(0x08);//turn off display
   	   OLED1.command(0x17);//switch to text
   	   OLED1.command(0x01);//clear display
   	   OLED1.command(0x04 | 0x08);//turn on display
	 */

	cmdcntr = 0;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x2A); //function set FT1 FT0 = 10 for rus
	DelayMs1 (200);

	cmdcntr = 1;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x0C);//display on
	DelayMs1 (200);

	cmdcntr = 2;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x06);//entry mode set
	DelayMs1 (200);

	cmdcntr = 3;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x02);//LCD return Home
	DelayMs1 (200);

	cmdcntr = 3;	//Used to print data on PC
	writecom4_NHD_C0216CU_FSW_GBW_3V3(0x01);//clear display
	DelayMs1 (200);


}



void reset_textmode(){

/*
  	   Switch to text
	   OLED1.command(0x08);//turn off display
   	   OLED1.command(0x17);//switch to text
   	   OLED1.command(0x01);//clear display
   	   OLED1.command(0x04 | 0x08);//turn on display
 */


}
