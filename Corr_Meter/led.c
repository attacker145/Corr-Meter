/*
 * led.c
 *
 *  Created on: Aug 29, 2017
 *      Author: RChak
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

extern void DelayMs1(int Ms);

void blnk_green (unsigned char nBlink){
	unsigned char i;
	for(i = 0; i < nBlink; i++){
	   GPIO_setOutputHighOnPin(GPIO_PORT_P4,GPIO_PIN7); // Turn ON green LED P4.7
	   DelayMs1 (100);
	   GPIO_setOutputLowOnPin(GPIO_PORT_P4,GPIO_PIN7); // Turn OFF green LED P4.7
	   DelayMs1 (100);
	}
}

