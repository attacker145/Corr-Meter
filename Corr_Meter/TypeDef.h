/*
 * TypeDef.h
 *
 *  Created on: Jan 20, 2017
 *      Author: rchak
 */

#ifndef TYPEDEF_H_
#define TYPEDEF_H_

typedef volatile union yeartype{
		uint16_t currentyear;

        struct {
			uint8_t LSB;
			uint8_t MSB;
        };
} yeartype;

typedef volatile union monthtype{
		uint16_t month;

        struct {
			uint8_t LSB;
			uint8_t MSB;
        };
} monthtype;

typedef volatile union daytype{
		uint16_t day;

        struct {
			uint8_t LSB;
			uint8_t MSB;
        };
} daytype;

typedef volatile union Styeartype{
		uint32_t year;

        struct {
			uint8_t LSB;
			uint8_t NSB1;
			uint8_t NSB2;
			uint8_t MSB;
        };
} Styeartype;

#endif /* TYPEDEF_H_ */
