/*
 * api.h
 *
 *  Created on: Sep 30, 2020
 *      Author: Roman.Chak
 */

#ifndef API_H_
#define API_H_
extern void convertCountBinToASCII(uint8_t* str);
extern void convertVoltageBinToASCII(uint8_t* str);
extern void convertSetVoltageBinToASCII(uint8_t* str);
extern void convertTimeLapseBinToASCII(uint8_t* str);
extern void DelayMs1(int Ms);
extern void init4_NHD_C0216CU_FSW_GBW_3V3(void); // was - extern void init4_NHD_C0216CU_FSW_GBW_3V3();
extern void writecom4_NHD_C0216CU_FSW_GBW_3V3(char cmd);
extern void writedata4_NHD_C0216CU_FSW_GBW_3V3(char byte);
extern void init4_WS_0010_5V(void);
extern void convertTimeBinToASCII(uint8_t* str);
extern void convertDateBinToASCII(uint8_t* str);
extern void pin_mux(void);
extern void blnk_green (unsigned char nBlink);
extern void adc_int(void);
//extern __interrupt void ADC12_ISR(void);
//extern void RTC_A_ISR(void);
//extern __interrupt void Port_1(void);
extern void battery_test( uint8_t ADCflag, uint32_t BatStat, uint32_t temp, uint8_t* Bat_voltage,
        const unsigned char* usb_msg10, const unsigned char* usb_msg7, const unsigned char* usb_msg8, uint8_t* Set_thck );
extern void convertThckBinToASCII(uint8_t* str);
extern void convertSetVoltageBinToASCII_NoUSB(uint8_t* str);
extern void convertThckBinToASCII_NoUSB(uint8_t* str);
extern void temperature(uint8_t* str);
extern void init4_RUS_WS_0010_5V(void);
extern void adc_int_Vref_3_3(void);
extern void uart_init (void);
extern void uart_init_no_int (void);
extern void UART_TX(char * tx_data);
extern void uart_init_19200_8MHz(void);
extern void uart_init_19200_8MHz_8E1(void);
extern uint8_t tx_RS485(uint8_t *uart0_buf, uint8_t *uart0_Rx_buf, uint8_t buffer_full, uint8_t *data);
extern void uchar_str( uint8_t var, uint8_t *Rx_buf);
extern void adc_int_3_3V_4ch(void);
extern void VoltageBinToASCII(uint8_t* str, uint16_t VIN);
extern void hexdec_long( uint32_t count );
extern void hexdec_to_str( uint32_t count, uint8_t *Rx_buf);
#endif /* API_H_ */
