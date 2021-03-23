/*
 * To_ASCII.c
 *
 *  Created on: Jan 20, 2017
 *      Author: rchak
 */

#include <msp430x552x.h>
#include <string.h>
#include "driverlib.h"
#include "hal.h"
#include "TypeDef.h"

void convertTwoDigBinToASCII(uint8_t bin, uint8_t* str);
void convertTimeBinToASCII(uint8_t* str);
void convertDateBinToASCII(uint8_t* str);
extern void hexdec_long( uint32_t count );

extern volatile uint8_t hour, min, sec;  // Real-time clock (RTC) values.  4:30:00
extern volatile uint8_t day, month;
extern volatile uint8_t Rx_buf[10];

extern yeartype year;

// Convert a number 'bin' of value 0-99 into its ASCII equivalent.  Assumes
// str is a two-byte array.
void convertTwoDigBinToASCII(uint8_t bin, uint8_t* str)
{
    str[0] = '0';
    if (bin >= 10)
    {
        str[0] = (bin / 10) + 48;
    }
    str[1] = (bin % 10) + 48;
}

// Convert the binary globals hour/min/sec into a string, of format "hr:mn:sc"
// Assumes str is an nine-byte string.
void convertTimeBinToASCII(uint8_t* str)
{
    uint8_t hourStr[2], minStr[2], secStr[2];

    //convertTwoDigBinToASCII(hour, hourStr);
    //convertTwoDigBinToASCII(min, minStr);
    //convertTwoDigBinToASCII(sec, secStr);

    convertTwoDigBinToASCII(hour, hourStr);
    convertTwoDigBinToASCII(min, minStr);
    convertTwoDigBinToASCII(sec, secStr);

    str[0] = '\n';
    str[1] = hourStr[0];
    str[2] = hourStr[1];
    str[3] = ':';
    str[4] = minStr[0];
    str[5] = minStr[1];
    str[6] = ':';
    str[7] = secStr[0];
    str[8] = secStr[1];
    str[9] = '\n';
}



// Convert MONTH DATE YEAR
void convertDateBinToASCII(uint8_t* str)
{
    uint8_t dayStr[2], monthStr[2];


    convertTwoDigBinToASCII(day, dayStr);
    convertTwoDigBinToASCII(month, monthStr);
    //void hexdec_long( uint32_t count )
    hexdec_long((uint32_t) year.currentyear);

    str[0] 	= '\n';
    str[1] 	= dayStr[0];
    str[2] 	= dayStr[1];
    str[3] 	= ':';
    str[4] 	= monthStr[0];
    str[5] 	= monthStr[1];
    str[6] 	= ':';
    str[7] = Rx_buf[6];
    str[8] = Rx_buf[7];
    str[9] = Rx_buf[8];
    str[10] = Rx_buf[9];
    str[11] = '\n';

}

/*
 int * ChooseList(int * list1, int * list2)
 {
   if (list1[0] < list2[0])
     return list1;
   else
     return list2;    // returns a copy of the address of the array
 }
And an example usage of this function:

 int numbers[5] = {1,2,3,4,5};
 int numList[3] = {3,5,7};
 int * p;
 p = ChooseList(numbers, numList);

 */
void uchar_str( uint8_t var, uint8_t *Rx_buf)
{
    uint8_t ones;
    uint8_t tens;
    uint8_t hundreds;

    hundreds        = 0;
    tens            = 0;
    ones            = 0;

    while ( var >= 100 )
    {
        var -= 100;                 // subtract 100
        hundreds++;                 // increment hundreds
    }
    while ( var >= 10 )
    {
        var -= 10;                  // subtract 10
        tens++;                     // increment tens
    }
        ones = var;                 // remaining count equals ones


    Rx_buf[0]= hundreds + 0x30;
    Rx_buf[1]= tens   + 0x30;
    Rx_buf[2]= ones + 0x30;
    return;
}
