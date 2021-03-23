/*
 * HexToLong.c
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

extern volatile uint32_t PulseCount;
extern volatile uint8_t Rx16_buf[5];

/*
 * Convert uint32_t hex value to an uint8_t array. Use this function to convert year to character decimal string.
 */
void Bit16_to_char( uint16_t count )
{
    uint8_t ones;
    uint8_t tens;
    uint8_t hundreds;
    uint8_t thousands;
    uint8_t thousand10s;

	thousand10s 	= 0;
	thousands 		= 0;
	hundreds 		= 0;
	tens  			= 0;
	ones 			= 0;

	while ( count >= 10000 )
	{
		count -= 10000;             // subtract 10000
		thousand10s++;				// increment 10th thousands
	}
	while ( count >= 1000 )
	{
		count -= 1000;				// subtract 1000
		thousands++;				// increment thousands
	}
	while ( count >= 100 )
	{
		count -= 100;               // subtract 100
		hundreds++;                 // increment hundreds
	}
	while ( count >= 10 )
	{
		count -= 10;				// subtract 10
		tens++;						// increment tens
	}
		ones = count;				// remaining count equals ones


    Rx16_buf[0]= thousand10s + 0x30; //60th K

    Rx16_buf[1]= thousands + 0x30;	//5th K
    Rx16_buf[2]= hundreds + 0x30;

    Rx16_buf[3]= tens   + 0x30;
    Rx16_buf[4]= ones + 0x30;
    return;
}







