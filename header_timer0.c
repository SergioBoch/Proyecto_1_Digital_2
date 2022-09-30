/* 
 * File:   header_timer0.c
 * Author: Jose Pablo Petion
 *          Sergio Alejandro Boch Ixén
 *
 * Created on August 4, 2022, 8:48 PM
 */
#include <xc.h>
#include <stdint.h>
#include "header_timer0.h"


/*
 * FUNCIONES A TRABJAR
 */

void tmr0_init (uint16_t prescaler, uint8_t valor_TMR0){
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.PSA  = 0;
    
    switch (prescaler){
        case 0:
            OPTION_REGbits.PSA  = 0;
            break;
        case 2:
            OPTION_REGbits.PS = 0b000;
            break;
        case 4:
            OPTION_REGbits.PS = 0b001;
            break;
        case 8:
            OPTION_REGbits.PS = 0b010;
            break;
        case 16:
            OPTION_REGbits.PS = 0b011;
            break;
        case 32:
            OPTION_REGbits.PS = 0b100;
            break;
        case 64:
            OPTION_REGbits.PS = 0b101;
            break;
        case 128:
            OPTION_REGbits.PS = 0b110;
            break;
        case 256:
            OPTION_REGbits.PS = 0b111;
            break;
    }
    
    TMR0=valor_TMR0;
    
    INTCONbits.T0IE = 1;
    INTCONbits.T0IF = 0;
    return;
}

void tmr0_reload(uint8_t valor_TMR0){
    
    TMR0 = valor_TMR0;
    T0IF = 0;
    return;

}
