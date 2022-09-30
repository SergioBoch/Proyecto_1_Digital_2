/* 
 * File: LCD.c  
 * Se utilizó y se adaptaron las librerías de Ligo George 
 * de la página www.electrosome.com
 * Enlace: https://electrosome.com/lcd-pic-mplab-xc8/
 * Revision history: 
 */

//LCD Functions Developed by electroSome
#include <stdint.h> 
#include <stdio.h>
#include "lcd8.h"

void Lcd8_Port(char a)
{
    PORTD=a;
}

void Lcd8_Cmd(char a)
{
    RS = 0;
    PORTD=a;
    EN = 1;
    __delay_ms(4);
    EN = 0;
}

void Lcd8_Clear(void) {
    Lcd8_Cmd(0);
    Lcd8_Cmd(1);
}

void Lcd8_Set_Cursor(char a, char b)
{
    char x;
    if (a == 1) {                   // Si es fila 1, seleccionar columna
                                    // (fila, columna)
        x = 0x80 + b - 1;
        Lcd8_Cmd(x);
    }
    else if (a == 2) {              // Si es fila 2, seleccionar columna 
                                    // (fila, columna)
        x = 0xC0 + b - 1;
        Lcd8_Cmd(x);
    }
}

void Lcd8_Init(void) {
    Lcd8_Port(0x00);
    __delay_ms(20);
    Lcd8_Cmd(0x30);
    __delay_ms(5);
    Lcd8_Cmd(0x30);
    __delay_us(100);
    Lcd8_Cmd(0x30);
    __delay_us(100);    //pausa 100ms
    /////////////////////////////////////////////////////
    Lcd8_Cmd(0x38);
    Lcd8_Cmd(0x08);
    Lcd8_Cmd(0x01);
    Lcd8_Cmd(0x06);
    Lcd8_Cmd(0x0C);

    
}

void Lcd8_Write_Char(char a) {

    RS = 1;
    PORTD=a;
    EN = 1;
    __delay_us(40);
    EN = 0;
}

void Lcd8_Write_String(char *a) {
    int i;
    for (i = 0; a[i] != '\0'; i++)
        Lcd8_Write_Char(a[i]);
}

void Lcd8_Shift_Right(void) {
    Lcd8_Cmd(0x01);
    Lcd8_Cmd(0x0C);
}

void Lcd8_Shift_Left(void) {
    Lcd8_Cmd(0x01);
    Lcd8_Cmd(0x08);
}
