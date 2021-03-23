/*
 * delay.c
 *
 *  Created on: Mar 10, 2021
 *      Author: Roman.Chak
 */

#include <msp430x552x.h>
#include <string.h>
#include "driverlib.h"
#include "hal.h"

void DelayMs(int Ms)
{
    int i;
    while(Ms>0)
    {
        for(i=0;i<104;i++);
        Ms--;
    }
}

void DelayMs1(int Ms)
{

    while(Ms>0)
    {
        __delay_cycles(8000); // delay 1ms - 32768/1000 counts. delay 1ms - 8000 000 / 1000 = 8000 counts
        Ms--;
    }
}

