/* 
 * File:   main_M.c
 * Author: Jose Pablo Petion
 *         Sergio Alejandro Boch Ixén
 *
 * Created on September 9, 2022, 8:12 AM
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
#include "bmp280.h"
/*------------------------------------------------------------------------------
 * CONSTANTES 
 -----------------------------------------------------------------------------*/
#define _XTAL_FREQ 4000000
#define I2C_SPEED 100000
#define ADDRESS 0b01001000
#define ADDRESS1 0x76
#define READ 0b0
#define WRITE 0b1
#define PUSH PORTBbits.RB0
#define IN_MIN 0                // Valor minimo de entrada del potenciometro
#define IN_MAX 255              // Valor máximo de entrada del potenciometro
#define OUT_MIN 0               // Valor minimo de ancho de pulso de señal PWM
#define OUT_MAX 500             // Valor máximo de ancho de pulso de señal PWM
/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t data = 0, data1 = 0, response = 0, velocidad = 0, servo = 0;
uint8_t prs0 = 0, prs1 = 0, prs2 = 0, presure=0;
uint8_t p0 = 0, p1 = 0, p2 =0, mux = 0;
uint16_t dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
uint16_t dig_T1, dig_T2, dig_T3;
uint32_t tempBMP = 0, prsBMP = 0, presion=0;
uint8_t bits0 = 0, bits1 = 0, bits2 = 0, bits3 = 0;
unsigned short CCPR, CCPR_0;
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
void wait_I2C(void);
void start_I2C(void);
void restart_I2C(void);
void stop_I2C(void);
void send_ACK(void);
void send_NACK(void);
__bit write_I2C(uint8_t data);
uint8_t read_I2C(void);
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    
    //INTERRUPCIONES DEL ADC - CONTROL MANUAL
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0){
            if(servo==0){    
                CCPR = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX); // Valor de ancho de pulso
                CCPR1L = (uint8_t)(CCPR>>2);    // Guardamos los 8 bits mas significativos en CPR1L
                CCP1CONbits.DC1B = CCPR & 0b11; // Guardamos los 2 bits menos significativos en DC1B
                velocidad=ADRESH;
            }
            if(servo>0){
                CCPR_0 = map(ADRESH, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);
                CCPR2L = (uint8_t)(CCPR_0>>2);
                CCP2CONbits.DC2B0 = CCPR_0 & 0b01;
                CCP2CONbits.DC2B1 = ((uint8_t)CCPR_0 >>1) & 0b1;
                
            }
        }
        PIR1bits.ADIF = 0;
    }
    
    if(INTCONbits.RBIF){
        if (!PUSH){
            servo = ~servo;
        }
        INTCONbits.RBIF = 0;
    }
    
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    
    while(1){ 
//***********************************ADC****************************************
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion
            ADCON0bits.CHS = 0;
            __delay_us(40);
            ADCON0bits.GO = 1;
        }
//**************************SENSOR LM75*****************************************
        //TEMPERATURA 
        data = (uint8_t)((ADDRESS<<1)+WRITE);
        start_I2C();                 // Iniciamos comunicación
        write_I2C(data);             // Enviamos dirección de esclavo a recibir datos
        PORTD=read_I2C();
        send_NACK();
        stop_I2C();                  // Finalizamos la comunicación
        __delay_ms(250);

//************************SENSOR BMP280*****************************************
        
        //PRESIÓN ATMOSFERICA - SETTINGS
        start_I2C();
        write_I2C(0xEC);
        write_I2C(0xF4);                        //Mandar direccion de registro a modificar
        write_I2C(0x57);                        // bin: 
        write_I2C(0xF5);
        write_I2C(0x90);
        send_NACK();
        stop_I2C();         
        //PRESION ATMOSFERICA - LECTURA
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC ESCLAVO
        write_I2C(0xFA);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        prs2=read_I2C();
        send_NACK();
        stop_I2C();                 // Finalizamos la comunicación
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0xFB);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        prs1=read_I2C();
        send_NACK();
        stop_I2C();                 // Finalizamos la comunicación
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0xFC);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        prs0=read_I2C();
        send_NACK();
        stop_I2C();                 // Finalizamos la comunicación
        // VALOR DE TEMPBMP EN CONJUNTO
        tempBMP = prs2;
        tempBMP = (tempBMP<<8) | prs1;
        tempBMP = (tempBMP<<8) | prs0;
        tempBMP = tempBMP>>4;
        //TIPOS DE MUESTRA PARA IMPLEMENTAR LA FUNCIÓN
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x89);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);             //0b0111 0110 -> 0b1110 1101 modo READ
        dig_T1=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x88);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_T1=(dig_T1<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x8B);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_T2=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x8A);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_T2=(dig_T2<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x8D);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_T3=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x8C);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_T3=(dig_T3<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        //VALORES ADC DE PRESIÓN ATM
        start_I2C();                  //Byte mas signif.
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0xF7);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        p2=read_I2C();
        send_NACK();
        stop_I2C();                 // Finalizamos la comunicación
        
        start_I2C();                  //Byte intermedio 
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0xF8);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        p1=read_I2C();
        send_NACK();
        stop_I2C();                 // Finalizamos la comunicación
        
        start_I2C();                  //Byte menor 
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0xF9);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        p0=read_I2C();
        send_NACK();
        stop_I2C();                 // Finalizamos la comunicación
        // VALOR DE PRESBMP EN CONJUNTO
        prsBMP = p2;
        prsBMP = (prsBMP<<8) | p1;
        prsBMP = (prsBMP<<8) | p0;
        prsBMP = (prsBMP>>4);
        // MUESTRAS PARA CONSEGUIR NUESTROS VALORES
        start_I2C();                  //Muestra 1
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x8F);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P1=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x8E);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P1=(dig_P1<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();                  //muestra 2
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x91);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P2=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x90);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P2=(dig_P2<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();                  //Muestra 3
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x93);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P3=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x92);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P3=(dig_P3<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();                  //Muestra 4
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x95);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P4=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x94);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P4=(dig_P4<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();                  //Muestra 5
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x97);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P5=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x96);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P5=(dig_P5<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();                  //Muestra 6
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x99);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P6=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x98);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P6=(dig_P6<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();                  //Muestra 7
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x9B);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P7=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x9A);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P7=(dig_P7<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();                  //Muestra 8
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x9D);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P8=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x9C);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P8=(dig_P8<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();                  //MUESTRA9
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x9F);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P9=read_I2C();
        send_NACK();
        stop_I2C();
        
        start_I2C();
        write_I2C(0xEC);              //0b76 modo 0 = WRITE -> 0xEC
        write_I2C(0x9E);              //Registro a leer
        restart_I2C();                //Previo reinicio        
        write_I2C(0xED);            //0b0111 0110 -> 0b1110 1101 modo READ
        dig_P9=(dig_P9<<8)|read_I2C();
        send_NACK();
        stop_I2C();
        //LLAMADO DE FUNCIONES
        presure = calculateTemp(tempBMP, dig_T1, dig_T2, dig_T3)/100;
        presion = readPressure(prsBMP, dig_P1, dig_P2, dig_P3,
        dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);
        bits0 = (uint8_t)(presion&0x000000FF);
        bits1 = (uint8_t)((presion>>8)&0x000000FF);
        bits2 = (uint8_t)((presion>>16)&0x000000FF);
        bits3 = (uint8_t)((presion>>24)&0x000000FF);
        
        
    //********************************FIN**************************************    
       if(PIR1bits.TXIF){ //Secuencia de envio de caracter ASCII
        TXREG = PORTD;       
        __delay_ms(100);   // TIEMPO ENTRE ENTRE ENVIO
        TXREG = bits0;       
        __delay_ms(100);   // TIEMPO ENTRE ENTRE ENVIO        
        TXREG = bits1;       
        __delay_ms(100);   // TIEMPO ENTRE ENTRE ENVIO
        TXREG = bits2;       
        __delay_ms(100);   // TIEMPO ENTRE ENTRE ENVIO
        TXREG = bits3;       
        __delay_ms(100);   // TIEMPO ENTRE ENTRE ENVIO
        TXREG = velocidad;       
        __delay_ms(100);   // TIEMPO ENTRE ENTRE ENVIO
        TXREG = servo;       
        __delay_ms(100);   // TIEMPO ENTRE ENTRE ENVIO
        }
    }
    
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b1;
    ANSELH = 0;                 // I/O digitales
    
    OSCCONbits.IRCF = 0b110;    // 4MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    TRISC = 0b10011000;         // SCL and SDA as input
    PORTC = 0;
    
    TRISD = 0;
    PORTD = 0;
    
    TRISB = 0b1;
    PORTB = 0;
    
    TRISA = 0b1;
    PORTA = 0;

    // Configuración ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(100);             // Sample time
    //Configuración del PULL-UP PORTB
    OPTION_REGbits.nRBPU = 0; 
    WPUB = 0b111;
    IOCB = 0b111;
    PORTB=PORTB;
    // Configuración PWM
    TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
    TRISCbits.TRISC1 = 1;       //
    PR2 = 124;                   // periodo de 2ms -> 0.002 = (PR2+1)*4*1/(Fosc)*Pes
    
    // Configuración CCP
    CCP1CON = 0;                // Apagamos CCP1
    CCP1CONbits.P1M = 0;        // Modo single output
    CCP1CONbits.CCP1M = 0b1100; // PWM
    CCP2CON = 0;                // Apagamos CCP2
    CCP2CONbits.CCP2M = 0b1100;
    
    CCPR1L = 125>>2;
    CCPR2L = 125>>2;                 //Resolucion 125
    CCP1CONbits.DC1B = 125 & 0b11;   // 0.5ms ancho de pulso / 25% ciclo de trabajo
    CCP2CONbits.DC2B0 = 125 & 0b01;  // 
    CCP2CONbits.DC2B1 = (125 & 0b10)>>1; 
    
    //TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    while(!PIR1bits.TMR2IF);    // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2 nuevamente
    
    TRISCbits.TRISC2 = 0;       // Habilitamos salida de PWM
    TRISCbits.TRISC1 = 0;
    //I2C COMUNICACTION
    SSPADD = ((_XTAL_FREQ)/(4*I2C_SPEED)) - 1;  // 100 kHz
    SSPSTATbits.SMP = 1;        // Velocidad de rotación
    SSPCONbits.SSPM = 0b1000;   // I2C master mode, clock= Fosc/(4*(SSPADD+1))
    SSPCONbits.SSPEN = 1;       // Habilitamos pines de I2C
    
    // Configuraciones de comunicacion serial USART
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
    INTCONbits.RBIF = 0;        //Limpiamos la bandera del Pullups
    INTCONbits.RBIE = 1;        //Habilitamos int. Puerto B
    PIR1bits.SSPIF = 0;         // Limpiamos bandera de interrupción de I2C
    PIE1bits.RCIE = 1;          //Int. Bnd Recepción
    PIR1bits.ADIF = 0;          // Limpiamos bandera de int. ADC
    PIE1bits.ADIE = 1;          // Habilitamos int. de ADC
    INTCONbits.PEIE = 1;        //Perifericas
    INTCONbits.GIE = 1;         //Globales
}

/*------------------------------------------------------------------------------
 * Funciones 
 ------------------------------------------------------------------------------*/

void wait_I2C(void){
    while(!PIR1bits.SSPIF);     // Esperamos a que se ejecute instruccion de I2C
    PIR1bits.SSPIF = 0;         // Limpimos bandera
}

void start_I2C(void){
    SSPCON2bits.SEN = 1;        // Inicializar comunicación
    wait_I2C();
}

void restart_I2C(void){
    SSPCON2bits.RSEN = 1;       // Reiniciar de comunicación
    wait_I2C();
}

void stop_I2C(void){
    SSPCON2bits.PEN = 1;        // Finalizar comunicación
    wait_I2C();
}

void send_ACK(void){
    SSPCON2bits.ACKDT = 0;      // Confirmar que se recibió la data
    SSPCON2bits.ACKEN = 1;      // Envio de ack al esclavo
    wait_I2C();
}

void send_NACK(void){
    SSPCON2bits.ACKDT = 1;      // Confirmar recepción al finalizar comunicación
    SSPCON2bits.ACKEN = 1;      // Envio de nack al esclavo
    wait_I2C();
}
__bit write_I2C(uint8_t data){
    SSPBUF = data;              // Cargar dato a enviar en el buffer
    wait_I2C();
    return ACKSTAT;             // Obtener ACK del esclavo
}

uint8_t read_I2C(void){
    SSPCON2bits.RCEN = 1;       // Pedir dato al esclavo  
    wait_I2C();
    return SSPBUF;              // Regresar dato recibido
}

/*
 *
 */
unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
        unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
} 
