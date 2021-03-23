#include <string.h>
#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                     //USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"

extern uint8_t HiLimitMSB, LoLimitMSB;
extern uint32_t LoPlsCnt, HiPlsCnt;

/*
 * NOTE: Modify hal.h to select a specific evaluation board and customize for
 * your own board.
 */
#include "hal.h"

/*
 *          P1 - P0
 * PlsCnt = -------- ( t - t0) + P0
 *          t1 - t0
 *
 */


void convertStrToPulsecount(void){
	//LoLimitMSB, HiLimitMSB
	if ((LoLimitMSB < 50) && (LoLimitMSB >= 40)){ 					// 1602 - 1443: 50 C - 40 C
		LoPlsCnt = ((1602 - 1443)/10 * (LoLimitMSB - 40) + 1443);
	}
	else if((LoLimitMSB < 40) && (LoLimitMSB >= 30)){ 				// 1443 - 1284: 40 C - 30 C
		LoPlsCnt = ((1443 - 1284)/10 * (LoLimitMSB - 30) + 1284);
	}
	else if((LoLimitMSB < 30) && (LoLimitMSB >= 20)){ 				// 1284 - 1125: 30 C - 20 C
		LoPlsCnt = ((1284 - 1125)/10 * (LoLimitMSB - 20) + 1125);
	}
	else if((LoLimitMSB < 20) && (LoLimitMSB >= 10)){ 				// 1125 - 966: 20 C - 10 C
		LoPlsCnt = ((1125 - 966)/10 * (LoLimitMSB - 10) + 966);
	}
	else if((LoLimitMSB < 10) && (LoLimitMSB >= 0)){ 				// 966 - 808: 10 C - 0 C
		LoPlsCnt = ((966 - 808)/10 * (LoLimitMSB) + 808);
	}
	/*
	 * 10 = 0000 1010
	 * -10 = 1111 0101 + 0000 0001 = 1111 0110 = 0xF6 = 246 = character
	 */
	else if((LoLimitMSB < 0) && (LoLimitMSB >= 0xF6)){ 				// 808 - 651: 0 C - -10 C ?????
		LoPlsCnt = ((808 - 651)/10 * (LoLimitMSB) + 651);
	}
}
