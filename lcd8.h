/* 
 * File: LCD.h  
 * Se utilizó y se adaptaron las librerías de Ligo George 
 * de la página www.electrosome.com
 * Enlace: https://electrosome.com/lcd-pic-mplab-xc8/
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef lcd8
#define lcd8

#ifndef _XTAL_FREQ
#define _XTAL_FREQ 4000000
#endif

#ifndef RS
#define RS PORTCbits.RC0
#endif

#ifndef EN
#define EN PORTCbits.RC1
#endif


#include <xc.h> // include processor files - each processor file is guarded.  

//LCD Functions Developed by electroSome

void Lcd8_Port(char a);

void Lcd8_Cmd(char a);

void Lcd8_Clear(void);

void Lcd8_Set_Cursor(char a, char b);

void Lcd8_Init(void);

void Lcd8_Write_Char(char a);

void Lcd8_Write_String(char *a);

void Lcd8_Shift_Right(void);

void Lcd8_Shift_Left(void);

#endif	/* LCD_H */
