/*
 * PulseToTemp.c
 *
 *  Created on: 2017
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
 * uint8_t t_lapse_min_str[13];//Time lapse in minutes string
 */
#include "hal.h"

extern void hexdec_long( uint32_t count );
extern volatile uint32_t t_lapse_min;
extern volatile uint8_t Rx_buf[10];

// Convert time lapse count to character string
void convertTimeLapseBinToASCII(uint8_t* str) // str is the output string
{

	hexdec_long(t_lapse_min);			// Conver PulseCount to a string

	str[0] 	= '\n';
	str[1] = ' ';
	str[2] 	= Rx_buf[0];
	str[3] 	= Rx_buf[1];
	str[4] 	= Rx_buf[2];
	str[5] 	= Rx_buf[3];
	str[6]  = Rx_buf[4];
	str[7] 	= Rx_buf[5];
	str[8]  = Rx_buf[6];
	str[9]  = Rx_buf[7];
	str[10] = Rx_buf[8];
	str[11] = Rx_buf[9];
	str[12] = 'm';
	str[13] = 'i';
	str[14] = 'n';
	str[15] = '\n';

}
/*
 * t_lapse_minToString.c
 *
 *  Created on: Sep 1, 2017
 *      Author: RChak
 */




