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
extern volatile uint32_t PulseCount;
extern volatile uint8_t Rx_buf[10];

// Convert temperature count to character string
void convertPulseCountBinToASCII(uint8_t* str) // str is the output string
{

	hexdec_long((uint32_t) PulseCount);			// Conver PulseCount to a string

	str[0] 	= '\n';
	str[1] = ' ';
	str[2] 	= Rx_buf[0];
	str[3] 	= Rx_buf[1];;

	str[4] 	= Rx_buf[2];
	str[5] 	= Rx_buf[3];
	str[6] 	= Rx_buf[4];
	str[7]  = Rx_buf[5];
	str[8]  = Rx_buf[6];
	str[9]  = Rx_buf[7];
	str[10] = Rx_buf[8];
	str[11] = Rx_buf[9];

	str[12] = '\n';

}


// Convert temperature count to character string
void convertCountBinToASCII(uint8_t* str)
{
	uint32_t temp;
	temp = PulseCount / 103;			//Temperature

	/*
	 * y = m(x-xo) + yo
	 * m = (temp2 - temp1)/(pulse2 - pulse1)
	 */

	if ((temp < 1602) && (temp >= 1443)){	// 50 C - 40 C
											// 1602 - 1443
		temp = 629 * (temp - 1443) + 400000;

		hexdec_long((uint32_t) temp);
		str[0] 	= '\n';
		str[1] 	= '+';
		str[2] 	= ' ';

		str[3] 	= Rx_buf[4];
		str[4] 	= Rx_buf[5];
		str[5] 	= '.';
		str[6] = Rx_buf[6];
		str[7] = Rx_buf[7];
		str[8] = Rx_buf[8];
		str[9] = Rx_buf[9];
		str[10] = ' ';
		str[11] = 'C';
		str[12] = '\n';
	}
	else if ((temp < 1443) && (temp >= 1284)){ 	// 40 C - 30 C
												// 1443 - 1284
		temp = 629 * (temp - 1284) + 300000;

		hexdec_long((uint32_t) temp);
		str[0] 	= '\n';
		str[1] 	= '+';
		str[2] 	= ' ';

		str[3] 	= Rx_buf[4];
		str[4] 	= Rx_buf[5];
		str[5] 	= '.';
		str[6] = Rx_buf[6];
		str[7] = Rx_buf[7];
		str[8] = Rx_buf[8];
		str[9] = Rx_buf[9];
		str[10] = ' ';
		str[11] = 'C';
		str[12] = '\n';
	}
	else if ((temp < 1284) && (temp >= 1125)){	// 30 C - 20 C
		temp = 629 * (temp - 1125) + 200000;	// 1284 - 1125

		hexdec_long((uint32_t) temp);
		str[0] 	= '\n';
		str[1] 	= '+';
		str[2] 	= ' ';

		str[3] 	= Rx_buf[4];
		str[4] 	= Rx_buf[5];
		str[5] 	= '.';
		str[6] = Rx_buf[6];
		str[7] = Rx_buf[7];
		str[8] = Rx_buf[8];
		str[9] = Rx_buf[9];
		str[10] = ' ';
		str[11] = 'C';
		str[12] = '\n';
	}
	else if ((temp < 1125) && (temp >= 966)){	// 20 C - 10 C
		temp = 629 * (temp - 966) + 100000;		// 1125 - 966

		hexdec_long((uint32_t) temp);
		str[0] 	= '\n';
		str[1] 	= '+';
		str[2] 	= ' ';

		str[3] 	= Rx_buf[4];
		str[4] 	= Rx_buf[5];
		str[5] 	= '.';
		str[6] = Rx_buf[6];
		str[7] = Rx_buf[7];
		str[8] = Rx_buf[8];
		str[9] = Rx_buf[9];
		str[10] = ' ';
		str[11] = 'C';
		str[12] = '\n';
	}
	else if ((temp < 966) && (temp >= 808)){	// 10 C - 0C
		temp = 633 * (temp - 808);				// 966 - 808

		hexdec_long((uint32_t) temp);
		str[0] 	= '\n';
		str[1] 	= '+';
		str[2] 	= ' ';

		str[3] 	= Rx_buf[4];
		str[4] 	= Rx_buf[5];
		str[5] 	= '.';
		str[6] = Rx_buf[6];
		str[7] = Rx_buf[7];
		str[8] = Rx_buf[8];
		str[9] = Rx_buf[9];
		str[10] = ' ';
		str[11] = 'C';
		str[12] = '\n';
	}
	else if ((temp < 808) && (temp >= 651)){ 	// 0 C - -10 C
		temp = 637 * (temp - 651) - 10000;		// 808 - 651

		hexdec_long((uint32_t) temp);
		str[0] 	= '\n';
		str[1] 	= '-';
		str[2] 	= ' ';

		str[3] 	= Rx_buf[4];
		str[4] 	= Rx_buf[5];
		str[5] 	= '.';
		str[6] = Rx_buf[6];
		str[7] = Rx_buf[7];
		str[8] = Rx_buf[8];
		str[9] = Rx_buf[9];
		str[10] = ' ';
		str[11] = 'C';
		str[12] = '\n';
	}
	/*
	 * y = m(x-xo) + yo
	 * m = (temp2 - temp1)/(pulse2 - pulse1) = (-10 + 20)/(651-494)= 0.06369
	 */
	else if ((temp < 651) && (temp >= 494)){ 	// -10 C - -20 C
			temp = 637 * (temp - 494) - 20000;	//  651  -  494

			hexdec_long((uint32_t) temp);
			str[0] 	= '\n';
			str[1] 	= '-';
			str[2] 	= ' ';

			str[3] 	= Rx_buf[4];
			str[4] 	= Rx_buf[5];
			str[5] 	= '.';
			str[6] = Rx_buf[6];
			str[7] = Rx_buf[7];
			str[8] = Rx_buf[8];
			str[9] = Rx_buf[9];
			str[10] = ' ';
			str[11] = 'C';
			str[12] = '\n';
		}
	else{
		temp = PulseCount;
		str[1] 	= 'x';
		hexdec_long((uint32_t) temp);
		str[0] 	= '\n';
		str[2] 	= Rx_buf[0];
		str[3] 	= Rx_buf[1];
		str[4] 	= Rx_buf[1];
		str[4] 	= Rx_buf[2];
		str[5] 	= Rx_buf[3];
		str[6] 	= Rx_buf[4];
		str[7] 	= Rx_buf[5];

		str[8] 	= Rx_buf[6];
		str[9] = Rx_buf[7];
		str[10] = Rx_buf[8];
		str[11] = Rx_buf[9];
		str[12] = ' ';
		str[13] = 'C';
		str[14] = '\n';
		str[15] = '\n';

	}

}
