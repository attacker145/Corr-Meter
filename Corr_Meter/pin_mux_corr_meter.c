#include <string.h>
#include "driverlib.h"

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


void pin_mux(void){


	//LCD ******************* LCD ********************
	//P7DIR = 0xFF; // D0 – D7				- Set P7.0-P7.7 to Output direction.  (Bit = 1: Port pin is switched to output direction)

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);   //RlyEN4
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN2);   //RlyEN1
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN3);   //RlyEN2
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);   //RlyEN3

    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN1);    //Simulates dead IC for testing WDT push-button
    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN4);    //H_Alarm 01192018
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);   //Slave reset Used in SPI
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN7);   //BLUE 01192018
	//P2DIR = 0x0F;
	/*
	 * 4-BIT mode:
	 * Under the 4-bit data transfer, DB4 to DB7 are used as bus lines. DB0 to DB3 are disabled.
	 * WS0010 page 35
	 */
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN0);    //DB4 P2.0	LCD PIN 11
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN1);    //DB5 P2.1	LCD PIN 12
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN2);    //DB6 P2.2	LCD PIN 13
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN3);    //DB7 P2.3  LCD PIN 14
	//Sensors
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN4);    //Sensor 1A
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN5);    //Sensor 1B
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN6);    //Sensor 2A
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN7);    //Sensor 2B

	GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0);   // Sensor 3A
	//GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN5);   // RS P6.5
	//GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN4);   // RW P6.4
	//GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN3);   // En P6.3
	//GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0);   // XRESET P8.0

	GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN0);  //Current Switch 5 state
	GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN2);  //PG 01192018

	GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5); //RlyEN5
	GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6); //BRD_PWR_EN 01192018
	GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7); //SPI ADC_SPI_CS


	GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN0); //P5.0
	GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN1); //P5.1
	GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN6); //P5.6
	GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN7); //P5.7


	GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN1); //P8.1 ON LED
	GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN1);// ON LED - turn ON after power up

	GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0);
	GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN1);
	GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN2);
	GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN3);
	GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN4);
	GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN5);
	GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN6);
	GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN7);

	//P1.1 Wake up button - Unit is running input *********************** PB PORT1 ***************************
	P1SEL &= (~BIT1); 							// Set P1.1 SEL as GPIO
	P1DIR &= (~BIT1); 							// Set P1.1 SEL as Input. P1.1 is read to determine if the instrument is running
	P1IES |= (BIT1); 							// Falling Edge
	P1IFG &= (~BIT1); 							// Clear interrupt flag for P1.1
	P1IE |= (BIT1); 							// Enable interrupt for P1.1 **********************IE
	P1REN |= BIT1;                          	// Enable P1.1 internal resistance
	P1OUT |= BIT1;                          	// Set P1.1 as pull-Up resistance
	//P1.4 Holiday Alarm input
	P1SEL &= (~BIT4); 							// Set P1.4 SEL as GPIO
	P1DIR &= (~BIT4); 							// Set P1.4 SEL as Input
	P1IES |= (BIT4); 							// Falling Edge
	P1IFG &= (~BIT4); 							// Clear interrupt flag for P1.4
	P1IE |= (BIT4); 							// Enable interrupt for P1.4 **********************IE
	P1REN |= BIT4;                          	// Enable P1.4 internal resistance
	P1OUT |= BIT4;                          	// Set P1.4 as pull-Up resistance

	//Initialize GPIO *********************************************************************
	//P2DIR |= BIT0;                // P2.0/LED output direction
	//Select CBOUT function on P1.6/CBOUT and set P1.6 to output direction
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN6);//Select CBOUT function on P1.6
	//Set P2.0 to output direction
	//GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
	//P2DIR |= BIT0;		// P2.0 set as output
	//P2DIR |= BIT1;		// P2.1 set as output
	//P2DIR |= BIT2;		// P2.2 set as output
	//P4DIR |= BIT7;		// P4.7 set as output Onboard GREEN LED
	GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7); // P4.7 set as output Onboard GREEN LED

	GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN6); //LCD power pin

	GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6);// LCD power OFF

	GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);

	return;
}


void LCD_pins_to_inpt (void){
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN0); //DB4 P2.0
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN1); //DB5 P2.1
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN2); //DB6 P2.2
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN3); //DB7 P2.3
	GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN5); // RS P6.5
	GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN4); // RW P6.4
	GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN3); // En P6.3
	GPIO_setAsInputPin(GPIO_PORT_P8, GPIO_PIN0); // XRESET P8.0
	return;
}


