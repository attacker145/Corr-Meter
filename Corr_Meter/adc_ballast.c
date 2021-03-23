#include <string.h>
#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                     //USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"

#include <msp430x552x.h>

uint8_t messageADC[] = 		"\nADC Interrupt\n";
uint8_t messageADC1[] = 	"\n No interrupt\n";
uint8_t messageADC2[] = 	"\nADC overflow \n";
uint8_t messageADC3[] = 	"\ntmng overflow\n";
uint8_t messageADC4[] = 	"\nADC12IFG0 Vec\n";
uint8_t messageADC5[] = 	"\nADC12IFG1 Vec\n";
uint8_t messageADC6[] = 	"\nADC12IFG2 Vec\n";
uint8_t messageADC7[] = 	"\nADC12IFG3 Vec\n";
uint8_t messageADC8[] = 	"\nADC12IFG4 Vec\n";
uint8_t messageADC9[] = 	"\nADC12IFG5 Vec\n";
uint8_t messageADC10[] = 	"\nADC12IFG6 Vec\n";
uint8_t messageADC11[] = 	"\nADC12IFG7 Vec\n";
uint8_t messageADC12[] = 	"\nADC12IFG8 Vec\n";
uint8_t messageADC13[] = 	"\nADC12IFG9 Vec\n";
uint8_t messageADC14[] = 	"\nADC12IFG10 Ve\n";
uint8_t messageADC15[] = 	"\nADC12IFG11 Ve\n";
uint8_t messageADC16[] = 	"\nADC12IFG12 Ve\n";
uint8_t messageADC17[] = 	"\nADC12IFG13 Ve\n";
uint8_t messageADC18[] = 	"\nADC12IFG14 Ve\n";

extern uint8_t ADCflag;
extern uint16_t BatStat;
extern uint16_t VSET;
extern void DelayMs1(int Ms);
extern void writedata4_NHD_C0216CU_FSW_GBW_3V3(char byte);
extern void writecom4_NHD_C0216CU_FSW_GBW_3V3(char cmd);

void adc_int(void){

	unsigned int i;

	//   ADC
	//	 Description: A single sample is made on A1 with reference to AVcc.
	//   Software sets ADC12SC to start sample and conversion - ADC12SC
	//   automatically cleared at EOC. ADC12 internal oscillator times sample (16x)
	//   and conversion. In Mainloop MSP430 waits in LPM0 to save power until ADC12
	//   conversion complete, ADC12_ISR will force exit from LPM0 in Mainloop on
	//   reti. If A0 > 0.5*AVcc, P1.0 set, else reset.
	//
	//                MSP430F552x
	//             -----------------
	//         /|\|                 |
	//          | |                 |
	//          --|RST              |
	//            |                 |
	//    Vbat -->|P6.1/CB1/A1  P1.0|--> LED
	//	  Vset -->|P6.2/CB2/A2	    |
	//
	//    Temperature AD590 connected to P6.2
	//******************************************************************************
	//ADC12_A Control Register 0
	//ADC12CTL0 = ADC12ON + ADC12MSC + ADC12SHT02 + ADC12REFON + ADC12REF2_5V;// ADC12SHT02 + 16 ADC12CLK cycles + ADC12_A on + ADC12_A multiple sample and conversion
	ADC12CTL0 = ADC12ON + ADC12MSC;
	//REFCTL0 &= ~REFMSTR;                      // Reset REFMSTR to hand over control to
	                                            // ADC12_A ref control registers
	//ADC12CTL0 = ADC12ON + ADC12MSC + ADC12REFON + ADC12REF2_5V;
	/* 15 14 13 12   11 10 9  8    7  6  5  4   3  2  1  0
	 * 0  0  0  0    0  1  0  0    0  0  0  1   0  0  0  0
	 *
	 * ADC12CTL0
	 * 15-12	ADC12SHT1x = 0b0000 ADC12_A 			sample-and-hold time. These bits define the number of ADC12CLK cycles in the sampling period for registers ADC12MEM8 to ADC12MEM15.
	 * 11-8 	ADC12SHT0x = 0000b = 4 ADC12CLK cycles 	sample-and-hold time. These bits define the number of ADC12CLK cycles in the sampling period for registers ADC12MEM0 to ADC12MEM7. 0100b = 64 ADC12CLK cycles
	 * 7    	ADC12MSC = 0b1  						The sampling timer requires a rising edge of the SHI signal to trigger each sample-and-convert.
	 * 6    	ADC12REF2_5V = 0b0; 0 = 1.5 V,  1 = 2.5V	ADC12_A reference generator voltage. ADC12REFON must also be set. The internal AVCC can also be used as the reference
	 * 5		ADC12REFON	= 0b0;						Reference to AVcc (3.3V REF)
	 * 4		ADC12ON = 0b1;							ADC12_A on
	 * 3		ADC12OVIE = 0b0;						0b = Overflow interrupt disabled
	 */


	//ADC12CTL1 = ADC12SHP + ADC12SSEL1 + (!ADC12SSEL0) + ADC12DIV0 + ADC12DIV1 + ADC12DIV2; // Use sampling timer - 0x0200 - 0000 0010 0000 0000 ADC12CTL1 = ADC12SHP;

	ADC12CTL1 = ADC12SHP + ADC12SHS_0 + ADC12CONSEQ0 + ADC12CONSEQ1;

	//The PxSEL.y bits provide the ability to disable the port pin input and output buffers.
	P6SEL |= 0x06;							//0b0000 0110: P6.1 - Bat level, P6.2 - Vset ADC option select. 0110
	//ADC12MCTL0 = ADC12SREF_1;				// Vr+=Vref+ and Vr-=AVss
	ADC12MCTL1 = ADC12INCH0;//A1
	ADC12MCTL2 = ADC12INCH1 + ADC12EOS;//A2
	for ( i=0; i<0x30; i++);
	//ADC12 Interrupt Enable (ADC12IE) bits enable interrupt request on a write to the respective ADC12MEMx register
	ADC12IE = ADC12IE2;
	__enable_interrupt();//Enable global interrupt.
	//ADC12IE = ADC12IE1 + ADC12IE2 + ADC12IE3 + ADC12IE4;          // Enable interrupt on 0000 0110 CH1 and CH2. ADC12IE = 0x01;<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


	/*
	 * 15 14 13 12  11 10 9  8   7  6  5  4   3  2  1  0
	 * 0  0  0  0   0  0  1  0   0  0  0  0   0  0  0  0
	 *
	 * ADC12CTL1
	 * 15-12	ADC12CSTARTADDx = 0b0000		ADC12_A conversion start address
	 * 11-10	ADC12SHSx = 00b = ADC12SC bit
	 */


	ADC12CTL0 |= ADC12ENC;					//Enable Conversion

	 ADC12IFG &= ~BIT0;
	 ADC12IFG &= ~BIT1;
	 ADC12IFG &= ~BIT2;
	 ADC12IFG &= ~BIT3;
	 ADC12IFG &= ~BIT4;
	 ADC12IFG &= ~BIT5;
	 ADC12IFG &= ~BIT6;
	 ADC12IFG &= ~BIT7;
	 ADC12IFG &= ~BIT8;
	 ADC12IFG &= ~BIT9;
	 ADC12IFG &= ~BITA;
	 ADC12IFG &= ~BITB;
	 ADC12IFG &= ~BITC;
	 ADC12IFG &= ~BITD;
	 ADC12IFG &= ~BITE;
	 ADC12IFG &= ~BITF;

}


//   ADC
//   Description: A single sample is made on A1 with reference to AVcc.
//   Software sets ADC12SC to start sample and conversion - ADC12SC
//   automatically cleared at EOC. ADC12 internal oscillator times sample (16x)
//   and conversion. In Mainloop MSP430 waits in LPM0 to save power until ADC12
//   conversion complete, ADC12_ISR will force exit from LPM0 in Mainloop on
//   reti. If A0 > 0.5*AVcc, P1.0 set, else reset.
//
//                MSP430F552x
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//    Vbat -->|P6.1/CB1/A1  P1.0|--> LED
//    Vset -->|P6.2/CB2/A2      |
//
//    Temperature AD590 connected to P6.2
//
//******************************************************************************
void adc_int_Vref_3_3(void){

    unsigned int i;
    /*
     * ADC12ON - 1b = ADC12_A on
     * ADC12MSC - ADC12_A multiple sample and conversion. Valid only for sequence or repeated modes.
     * 1b = The first rising edge of the SHI signal triggers the sampling timer, but further
     * sample-and-conversions are performed automatically as soon as the prior
     * conversion is completed.
     */
    ADC12CTL0 = ADC12ON + ADC12MSC;

    /*
     * ADC12SHP - 1b = SAMPCON signal is sourced from the sampling timer.
     * ADC12_A sample-and-hold pulse-mode select. This bit selects the source of the
     * sampling signal (SAMPCON) to be either the output of the sampling timer or the
     * sample-input signal directly.
     * ADC12SHSx - 10b = Timer source (see device-specific data sheet for exact timer and locations)
     * ADC12CONSEQx - ADC12_A conversion sequence mode select
     */
    ADC12CTL1 = ADC12SHP + ADC12SHS_0 + ADC12CONSEQ0 + ADC12CONSEQ1;
    P6SEL |= 0x06;                      //0b0000 0110: P6.1 - Bat level, P6.2 - Vset ADC option select. 0110

    ADC12MCTL1 = ADC12INCH0;            //A1
    ADC12MCTL2 = ADC12INCH1 + ADC12EOS; //A2
    for ( i=0; i<0x30; i++);

    ADC12IE = ADC12IE2;
    __enable_interrupt();//Enable global interrupt.

    ADC12CTL0 |= ADC12ENC;                  //Enable Conversion

     ADC12IFG &= ~BIT0;
     ADC12IFG &= ~BIT1;
     ADC12IFG &= ~BIT2;
     ADC12IFG &= ~BIT3;
     ADC12IFG &= ~BIT4;
     ADC12IFG &= ~BIT5;
     ADC12IFG &= ~BIT6;
     ADC12IFG &= ~BIT7;
     ADC12IFG &= ~BIT8;
     ADC12IFG &= ~BIT9;
     ADC12IFG &= ~BITA;
     ADC12IFG &= ~BITB;
     ADC12IFG &= ~BITC;
     ADC12IFG &= ~BITD;
     ADC12IFG &= ~BITE;
     ADC12IFG &= ~BITF;

}


void adc_int_3_3V_4ch(void){

    unsigned int i;
    /*
     * ADC12ON - 1b = ADC12_A on
     * ADC12MSC - ADC12_A multiple sample and conversion. Valid only for sequence or repeated modes.
     * 1b = The first rising edge of the SHI signal triggers the sampling timer, but further
     * sample-and-conversions are performed automatically as soon as the prior
     * conversion is completed.
     */
    ADC12CTL0 = ADC12ON + ADC12MSC;

    /*
     * ADC12SHP - 1b = SAMPCON signal is sourced from the sampling timer.
     * ADC12_A sample-and-hold pulse-mode select. This bit selects the source of the
     * sampling signal (SAMPCON) to be either the output of the sampling timer or the
     * sample-input signal directly.
     * ADC12SHSx - 10b = Timer source (see device-specific data sheet for exact timer and locations)
     * ADC12CONSEQx - ADC12_A conversion sequence mode select
     */
    ADC12CTL1 = ADC12SHP + ADC12SHS_0 + ADC12CONSEQ0 + ADC12CONSEQ1;
    P6SEL |= 0x06;                      //0b0000 0110: P6.1 - Bat level, P6.2 - Vset ADC option select. 0110

    ADC12MCTL1 = ADC12INCH0;            //A1
    //ADC12MCTL2 = ADC12INCH1 + ADC12EOS; //A2
    ADC12MCTL2 = ADC12INCH1;


    for ( i=0; i<0x30; i++);

    ADC12IE = ADC12IE2;
    __enable_interrupt();//Enable global interrupt.

    ADC12CTL0 |= ADC12ENC;                  //Enable Conversion

     ADC12IFG &= ~BIT0;
     ADC12IFG &= ~BIT1;
     ADC12IFG &= ~BIT2;
     ADC12IFG &= ~BIT3;
     ADC12IFG &= ~BIT4;
     ADC12IFG &= ~BIT5;
     ADC12IFG &= ~BIT6;
     ADC12IFG &= ~BIT7;
     ADC12IFG &= ~BIT8;
     ADC12IFG &= ~BIT9;
     ADC12IFG &= ~BITA;
     ADC12IFG &= ~BITB;
     ADC12IFG &= ~BITC;
     ADC12IFG &= ~BITD;
     ADC12IFG &= ~BITE;
     ADC12IFG &= ~BITF;

}

void battery_test( uint8_t ADCflag, uint32_t BatStat, uint32_t temp, uint8_t* Bat_voltage,
		const unsigned char* usb_msg10, const unsigned char* usb_msg7, const unsigned char* usb_msg8, uint8_t* Set_thck ){
	unsigned int i;

	if (ADCflag == 1){// Set the flag when sample is ready
		//temp = ((uint32_t)BatStat * 805);// Determine battery voltage

		if (temp > 2472960){ //3072*805 = 2472960. Testing for low battery (75% of 3.3V). 0.75*4096 = 3072
			writecom4_NHD_C0216CU_FSW_GBW_3V3(0x80);
			for(i=0;i<8;i++){
				writedata4_NHD_C0216CU_FSW_GBW_3V3(Set_thck[(i)]);// 5.0mm (thickness)
			}
			//writedata4_NHD_C0216CU_FSW_GBW_3V3(' ');//86
			//writedata4_NHD_C0216CU_FSW_GBW_3V3(' ');//87
			//writecom4_NHD_C0216CU_FSW_GBW_3V3(0x88);//First line - Print Battery Voltage
			DelayMs1(1);
			for(i=0;i<4;i++){//8
				writedata4_NHD_C0216CU_FSW_GBW_3V3(usb_msg10[i]);//"Bat.: " or "BAT.: " - 6
				DelayMs1(30);
			}
			//while (LCDbusy4_NHD_C0216CU_FSW_GBW_3V3());
			for(i=1;i<5;i++){//7
				writedata4_NHD_C0216CU_FSW_GBW_3V3(Bat_voltage[(i)]); //Refresh the voltage reading "str[1], str[2], str[3], str[4]" +4 - 10
				//while (LCDbusy_NHD_C0216CU_FSW_GBW_3V3());
				DelayMs1(30);
			}
			//for(i=0;i<16;i++){
			//writedata4_NHD_C0216CU_FSW_GBW_3V3(' ');//The rest of the LCD
				//while (LCDbusy_NHD_C0216CU_FSW_GBW_3V3());
				//DelayMs1(30);
			//}
		}
		else{
			writecom4_NHD_C0216CU_FSW_GBW_3V3(0x80);//First line - Print Battery Voltage
			DelayMs1(1);
			for(i=0;i<16;i++){
				writedata4_NHD_C0216CU_FSW_GBW_3V3(usb_msg8[i]); //Low Battery
				//while (LCDbusy_NHD_C0216CU_FSW_GBW_3V3());
				DelayMs1(30);
			}
		}
	}
	else{
		//usb_msg4[i]
		writecom4_NHD_C0216CU_FSW_GBW_3V3(0x80);//First line - No sample
		DelayMs1(1);
		//while (LCDbusy4_NHD_C0216CU_FSW_GBW_3V3());

		for(i=0;i<16;i++){
			//LCD_dat4(Msg2[i]);
			writedata4_NHD_C0216CU_FSW_GBW_3V3(usb_msg7[i]); //Refresh the temperature reading
			//while (LCDbusy_NHD_C0216CU_FSW_GBW_3V3());
			DelayMs1(1);
		}
	}
}
