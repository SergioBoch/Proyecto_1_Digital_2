/* 
 * File:   BMP280.h
 * Author: Compu Fire
 *
 * Created on 13 de septiembre de 2022, 1:16
 */

#ifndef bmp280_H
#define	bmp280_H

#include <xc.h> // include processor files - each processor file is guarded.
#include <stdint.h>

int calculateTemp(uint32_t temp, uint16_t dig1, uint16_t dig2, uint16_t dig3);
uint32_t delta(uint32_t temp1, uint16_t dig11, uint16_t dig22, uint16_t dig33);
unsigned long readPressure(uint32_t adc_P, uint16_t dig01, uint16_t dig02, uint16_t dig03,
        uint16_t dig04, uint16_t dig05, uint16_t dig06, uint16_t dig07,
        uint16_t dig08, uint16_t dig09);

#endif	/* BMP280_H */
