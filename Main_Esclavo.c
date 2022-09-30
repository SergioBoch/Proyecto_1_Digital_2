/* 
 * File:   main_S.c
 * Author: Jose Pablo Petion
 *          Sergio Alejandro Boch Ixén
 *
 * Created on September 9, 2022, 8:43 AM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h> 
#include <stdio.h>
#include "lcd8.h"
#include "header_spi.h"
#include "header_timer0.h"

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 4000000

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
char valor[];
uint16_t hpascal = 0;
uint8_t cont = 0, recivo, paso=0;                // Valor inicial del contador
uint8_t temp=0, pres1 = 0, pres2 = 0, pres3 = 0, pres4 = 0, velocidad = 0, srv = 0;
uint32_t analog = 0;
unsigned short CCPR = 0;
unsigned char paso_doble[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    


    //-------------------------Comunicación serial------------------------------
    if(PIR1bits.RCIF){
        cont++;
        if(cont==1){temp = RCREG;}
        if(cont==2){pres1 = RCREG;}
        if(cont==3){pres2 = RCREG;}
        if(cont==4){pres3 = RCREG;}
        if(cont==5){pres4 = RCREG;}
        if(cont==6){velocidad = RCREG;}
        if(cont==7){srv = RCREG;}
        if(cont>7){cont = 0;}
    }
    //*******************************COMUN. SPI*********************************
    if(PIR1bits.SSPIF){
        recivo=SSPBUF;
        if(recivo==0){
            spiWrite((uint8_t)(hpascal&0x00FF));
        }
        if(recivo==1){
            spiWrite((uint8_t)((hpascal>>8)&0x00FF));
        }
        if(recivo==2){
            spiWrite(temp);
        }
        if(recivo==3){
            spiWrite(velocidad);
        }
        if(recivo==4){
            spiWrite(srv&0x01);
        }
        PIR1bits.SSPIF=0;
    }
    //********************************TIMER0 STEPEPR****************************
    if(T0IF){
        if(hpascal>912){
            PORTA=paso_doble[paso];
            paso++;
            if(paso>8){paso=0;}
            tmr0_reload(178);
        }
        else {
            tmr0_reload(178);
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){

            __delay_ms(1000);
            //Union de las variables
            analog = pres4;
            analog = (analog<<8)|pres3;
            analog = (analog<<8)|pres2;
            analog = (analog<<8)|pres1;
            hpascal = (uint16_t)(analog/100);
            //Mostrar datos del sensor temperatura
            sprintf(valor,"%d C  %d hPa", temp, hpascal);
            Lcd8_Set_Cursor(1, 1);
            Lcd8_Write_String("Temp: PRESION:");
            Lcd8_Set_Cursor(2, 1);
            Lcd8_Write_String(valor);
            __delay_ms(1000);
            Lcd8_Clear();
            Lcd8_Set_Cursor(1, 1);
            Lcd8_Write_String("speed: state:");
            sprintf(valor,"%d    %d ", velocidad, srv&0b00000001);
            Lcd8_Set_Cursor(2, 1);
            Lcd8_Write_String(valor);
            //Mostrar datos del sensor Presión
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0;
    ANSELH = 0;                 // I/O digitales
    
    OSCCONbits.IRCF = 0b110;    // 4MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    TRISC = 0b10011000;         // SCL and SDA as input
    PORTC = 0;
    
    TRISD = 0;
    PORTD = 0x00;
    
    TRISA = 0b00100000;
    PORTA = 0x00;
    
    //LCD
    Lcd8_Init();
    Lcd8_Clear();
    spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_ACTIVE_2_IDLE);
    
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 103;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
    //-----------------------------------------------------------------------
    //TMR0 - stepper
    tmr0_init (256, 217);
    //nterrupciones
    PIR1bits.SSPIF = 0;
    PIE1bits.SSPIE = 1;
    PIE1bits.RCIE = 1;          //Int. Bnd Recepción
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de perifericos
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
}

/*
 *
 */
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
        unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
