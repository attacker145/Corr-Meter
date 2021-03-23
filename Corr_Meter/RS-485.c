
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

/*
 * uint8_t r_temp[] = {0x40, 0x45, 0xF1, 0x83, 0x00};//Read temperature - example
 * uint8_t r_irradiance[] = {0x40, 0x46, 0xB1, 0x82, 0x00};;
 *
 * buffer_full == 1 - Rx buffer is full. Filled in UART Rx interrupt
 * uart0_buf[uart_cntr] = UCA0RXBUF;
 * uint8_t *uart0_buf - Tx buffer (r_irradiance r_temp)
 */


uint8_t tx_RS485(uint8_t *uart0_buf, uint8_t *uart0_Rx_buf, uint8_t buffer_full, uint8_t *data){
    uint8_t cntr = 0;

    if (buffer_full == 0){                                  // Initial state. Transmit command if Rx buffer is empty (buffer_full == 0)
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);   // Use this pin as RS485 control. DI is enabled - ready to transmit. Set the pin to a known state
        //DelayMs1(1);
        for (cntr = 0; cntr < 5; cntr++){
            while (!(UCA0IFG&UCTXIFG));                     // USCI_A0 TX buffer ready?
            UCA0TXBUF = uart0_buf[cntr];                    // Send command: r_irradiance
        }
        buffer_full = 2;                                    // Command transmission is complete
        DelayMs1(2);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);    // RE is enabled - ready to receive
    }else {                                                 // Rx buffer is full, get data from the receive buffer.
        buffer_full = 0;                                    // Receive buffer is read - reset buffer_full flag
        *data = uart0_Rx_buf[2];                            // {address, function, data, checksum} The data is in location 2
    }
    return buffer_full;
}
