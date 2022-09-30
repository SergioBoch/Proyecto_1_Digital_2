/* 
 * File:   bmp280.c
 * Author: Jose Pablo Petion
 *          Sergio Alejandro Boch Ixén
 *
 * Created on September 13, 2022, 7:08 PM
 */

#include <stdio.h>
#include <stdlib.h>

#include "bmp280.h"
uint32_t t_fine;

int calculateTemp(uint32_t temp, uint16_t dig1, uint16_t dig2, uint16_t dig3){
  uint32_t var1, var2;

  var1 = ((((temp >> 3) - ((uint32_t)dig1 << 1))) *((uint32_t)dig2)) >>11;

  var2 = (((((temp >> 4) - ((uint32_t)dig1)) *((temp >> 4) - ((uint32_t)dig1))) >> 12) *
          ((uint32_t)dig3)) >> 14;

  t_fine = var1 + var2;

  int T = (t_fine * 5 + 128) >> 8;
  
  return T;
}

/*
 *
 */

//uint32_t delta(uint32_t temp1, uint16_t dig11, uint16_t dig22, uint16_t dig33){
//  uint32_t var11, var22;
//
//  var11 = ((((temp1 >> 3) - ((uint32_t)dig11 << 1))) *((uint32_t)dig22)) >>11;
//
//  var22 = (((((temp1 >> 4) - ((uint32_t)dig11)) *((temp1 >> 4) - ((uint32_t)dig11))) >> 12) *
//          ((uint32_t)dig33)) >> 14;
//
//  uint32_t t_fine = var11 + var22;
//
//  
//  return t_fine;
//}

/*
 * 
 */

unsigned long readPressure(uint32_t adc_P, uint16_t dig01, uint16_t dig02, uint16_t dig03,
        uint16_t dig04, uint16_t dig05, uint16_t dig06, uint16_t dig07,
        uint16_t dig08, uint16_t dig09){
  uint32_t var1, var2;
  uint32_t p;

  // Must be done first to get the t_fine variable set up
  //dato ya shifteado a la derecha

  var1 = ((uint32_t)t_fine>>1) - (uint32_t)64000;
  var2 = (((var1>>2) * (var1>>2))>>11) * ((uint32_t)dig06);
  var2 = var2 + ((var1*((uint32_t)dig05))<<1);
  var2 = (var2>>2) + (((uint32_t)dig04) << 16);
  var1 = (((dig03*(((var1>>2)*(var1>>2))>>13))>>3)+((((uint32_t)dig02)*var1)>>1))>>18;
  var1 = ((((32768+var1))*((uint32_t)dig01))>>15);
  
  if (var1 == 0){return 0;}
  
  p = (((uint32_t)(((uint32_t)1048576)-adc_P)-(var2>>12)))*3125;
  if (p< 0x80000000){
      p = (p<<1)/((uint32_t)var1);
  }
  else
  {
      p = (p/(uint32_t)var1)*2;
  }
  var1 = (((uint32_t)dig09)*((uint32_t)(((p>>3)*(p>>3))>>13)))>>12;
  var2 = (((uint32_t)(p>>2))*((uint32_t)dig08))>>13;
  
  p = (uint32_t)((uint32_t)p+((var1+var2+dig07)>>4));
  return p;
}

/*
 *
 */
