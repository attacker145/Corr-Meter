/*
 * PulseToTemp.c
 *
 *  Created on: Jan 13, 2017
 *      Author: rchak
 */

#include <string.h>
#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                     //USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"


/*
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"

extern void hexdec_long( uint32_t count );
//extern volatile uint32_t ADC12MEM0;
extern volatile uint8_t Rx_buf[10];

extern volatile uint16_t BatStat;
extern volatile uint16_t VSET;
extern volatile uint8_t rus;
extern uint8_t LoLimitMSB;
extern uint8_t HiLimitLSB;


void VoltageBinToASCII(uint8_t* str, uint16_t VIN) // str is the output string to which result is loaded - 11 characters
{

    uint32_t VCH1;

    //3.3V/2^12 = 0.000805V/step;
    VCH1 = ((uint32_t)VIN * 805);       // 0.000805V per step. 805uV per step
    hexdec_long(VCH1);                      // Convert to a string

    str[0]  = '\n';
    str[1] = ' ';
    str[2]  = Rx_buf[0];
    str[3]  = Rx_buf[1];
    str[4]  = Rx_buf[2];
    str[5]  = Rx_buf[3];
    str[6]  = '.';
    str[7]  = Rx_buf[4];
    str[8]  = Rx_buf[5];
    str[9]  = Rx_buf[6];
    str[10]  = Rx_buf[7];
    str[11]  = Rx_buf[8];
    str[12]  = Rx_buf[9];
    str[13]  = 'V';

    return;
}



// Battery Voltage
void convertVoltageBinToASCII(uint8_t* str) // str is the output string to which result is loaded
{

	uint32_t Vbat, BAT;

	//3.3V/2^12 = 0.000805V/step;
	//temp = ((uint32_t)BatStat * 805); //V
	Vbat = ((uint32_t)BatStat * 805);   //805 counts per step
	BAT = ((14286 * Vbat) - 36884)/4;
	hexdec_long(BAT);			        // Conver PulseCount to a string

	//3.3 V - 100%
	//2.6 V - 0%
	//(3.3, 100), (2.6, 0); m = 100/(3.3-2.6) = 142.86; y = m(x - xo) + yo = 142.86 * (x - 2.6) + 2.6 = 142.86x - 371.44 + 2.6;
	//y = 142.86x - 368.84. BAT% = 142.86 * Vbat - 368.84 = ((14286 * Vbat)- 36884)/100

//	str[0] 	= '\n';
//	str[1] = ' ';
//	str[2] 	= Rx_buf[0];
//	str[3] 	= Rx_buf[1];
//	str[4] 	= Rx_buf[2];
//	str[5] 	= '.';
//	str[6]  = Rx_buf[3];
//	str[7] 	= Rx_buf[4];
//	str[8]  = '%';
//	str[9]  = ' ';
//	str[10] = ' ';
//	str[11] = ' ';
	str[0] 	= '\n';
	str[1] 	= Rx_buf[0];
	str[2] 	= Rx_buf[1];
	str[3] 	= Rx_buf[2];
	str[4]  = '%';


//	str[12] = '\n';

}

//High Voltage set
void convertSetVoltageBinToASCII(uint8_t* str) // str is the output string
{

	uint32_t temp;
	//3.3V/2^12 = 0.000805V/step
	//2.5V/2^12 = 0.000805V/step

	//temp = ((uint32_t)VSET * 805)/1000000;//uV
	//temp = (((uint32_t)VSET * 805) * HiLimitLSB) + LoLimitMSB;
	temp = (uint32_t)VSET * 805;
	hexdec_long(temp);			// Conver PulseCount to a string

	str[0] 	= '\n';//Don't print to LCD, USB only
	str[1] = ' ';//Don't print to LCD, USB only
	str[2] 	= Rx_buf[0];
	str[3] 	= Rx_buf[1];
	str[4] 	= Rx_buf[2];
	str[5] 	= Rx_buf[3];
	str[6]  = '.';
	str[7] 	= Rx_buf[4];
	str[8]  = Rx_buf[5];
	str[9]  = Rx_buf[6];
	str[10] = ' ';
	if (!rus){
		str[11] = 'V';
	}
	else{
		str[11] = 0x42;
	}
	str[12] = '\n';//Don't print to LCD, USB only

}

//High Voltage set
void convertThckBinToASCII(uint8_t* str) // str is the output string
{

	uint32_t temp;
	//3.3V/2^12 = 0.000805V/step
	//temp = ((uint32_t)VSET * 805)/1000000;//uV

	temp = (((((uint32_t)VSET * 805) * HiLimitLSB) + LoLimitMSB))/5;
	hexdec_long(temp);			// Conver PulseCount to a string

	str[0] 	= Rx_buf[2];//junk val
	str[1] 	= Rx_buf[3];
	str[2]  = '.';
	str[3] 	= Rx_buf[4];

	if (!rus){
		str[4] = 'm';
		str[5] = 'm';
	}
	else{
		str[4] = 0xBC;
		str[5] = 0xBC;
	}
	str[6] = ' ';
	str[7] = ' ';
	Rx_buf[0] = '\n';
	//str[12] = '\n';//Don't print to LCD, USB only
	//USBCDC_sendDataInBackground((uint8_t *)Rx_buf, 10, CDC0_INTFNUM, 1000); // Print "\nGalas-NDT Holiday Detector Rev01\n"
	{
		_NOP();  	// If it fails, it'll end up here.  Could happen if
	    	    	// the cable was detached after the connectionState()
	}
}



void convertSetVoltageBinToASCII_NoUSB(uint8_t* str) // str is the output string
{

	uint32_t temp;
	/*
	 * 3.1V - 40000V
	 * 40000/3.1 = 12903.22580
	 */
	temp = ((uint32_t)VSET * 10120);
	hexdec_long(temp);			// Conver PulseCount to a string

	str[0] 	= '\n';//Don't print to LCD, USB only
	str[1] = ' ';//Don't print to LCD, USB only
	str[2] 	= Rx_buf[0];
	str[3] 	= Rx_buf[1];
	str[4] 	= Rx_buf[2];
	str[5] 	= Rx_buf[3];
	str[6]  = '.';
	str[7] 	= Rx_buf[4];
	str[8]  = Rx_buf[5];
	str[9]  = Rx_buf[6];
	str[10] = 'k';
	if (!rus){
		str[11] = 'V';
	}
	else{
		str[11] = 0x42;
	}
	str[12] = '\n';//Don't print to LCD, USB only
}

void convertThckBinToASCII_NoUSB(uint8_t* str) // str is the output string
{

	uint32_t temp;
	temp = ((uint32_t)VSET * 10120)/5;
	hexdec_long(temp);			// Conver PulseCount to a string

//	str[0] 	= '\n';//Don't print to LCD, USB only
//	str[1] = ' ';//Don't print to LCD, USB only
	str[0] 	= Rx_buf[2];//junk val
	str[1] 	= Rx_buf[3];
	str[2]  = '.';
	str[3] 	= Rx_buf[4];

	if (!rus){
		str[4] = 'm';
		str[5] = 'm';
	}
	else{
		str[4] = 0xBC;
		str[5] = 0xBC;
	}
	str[6] = ' ';
	str[7] = ' ';
	Rx_buf[0] = '\n';

	//USBCDC_sendDataInBackground((uint8_t *)Rx_buf, 10, CDC0_INTFNUM, 1000); // Print "\nGalas-NDT Holiday Detector Rev01\n"
	{
		_NOP();  	// If it fails, it'll end up here.  Could happen if
	    	    	// the cable was detached after the connectionState()
	}


}

/*
 * Temperature sensor P6.1
 * 1 uA/K
 * With 5kOhm resistor. V (per Kelvin) = 1 uA * 5000 = 5mV / per Kelvin
 * C = K - 273.15 = Voltage on (P6.1) * 161 * 1000 - 273150000
 * K = C + 273.15
 * 5mV - 1K
 * xmV - 1C = 1K - 273.15
 */
void temperature(uint8_t* str) // str is the output string
{

    uint32_t temp;
    //3.3V/2^12 = 0.000805V/step
    //2.5V/2^12 = 0.000610V/step
    //1.5V/2^12 = 0.000366/step

    //temp = ((uint32_t)VSET * 805)/1000000;//uV
    //temp = (((uint32_t)VSET * 805) * HiLimitLSB) + LoLimitMSB;
    //temp = (uint32_t)VSET * 805 * 1000 - 273150000;
    //temp = (uint32_t)BatStat * 805 * 1000 - 273150000; //1k-kOhm resistor
    temp = (uint32_t)BatStat * 161 * 1000 - 273150000; //5k-kOhm resistor
    hexdec_long(temp);          // Conver PulseCount to a string

    str[0]  = '\n';//Don't print to LCD, USB only
    str[1] = ' ';//Don't print to LCD, USB only
    str[2]  = Rx_buf[0];
    str[3]  = Rx_buf[1];
    str[4]  = Rx_buf[2];
    str[5]  = Rx_buf[3];
    str[6]  = '.';
    str[7]  = Rx_buf[4];
    str[8]  = Rx_buf[5];
    str[9]  = Rx_buf[6];
    if (!rus){
        str[10] = 'C';
    }
    else{
        str[10] = 0x42;
    }
    str[11] = ' ';
    str[12] = ' ';//Don't print to LCD, USB only

}
