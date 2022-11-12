/*
********************************************************************************
Universidad del Valle de Guatemala
IE2023 Programación de microcontroladores
Autor: Gabriel Andrade
compilador: XC8
proyecto: Brazo mecánico
hardware: PIC 16F887, 3 potenciómetros, servomotores, 3 leds
creado: 12/11/2022
última modificación: 12/11/2022
********************************************************************************
 */

#define _XTAL_FREQ 8000000 //frecuencia de 8 MHz
#include <xc.h>

// CONFIG1
#pragma config FOSC=INTRC_NOCLKOUT //Oscilador interno sin salida
#pragma config WDTE=OFF //Reinicio repetitivo del pic
#pragma config PWRTE=ON //espera de 72 ms al iniciar el pic
#pragma config MCLRE=OFF //El pin MCLR se utiliza como entrada/salida
#pragma config CP=OFF //Sin protecci�n de c�digo
#pragma config CPD=OFF //Sin protecci�n de datos
#pragma config BOREN=OFF //Sin reinicio cuando el input voltage es inferior a 4V
#pragma config IESO=OFF //Reinicio sin cambio de reloj de interno a externo
#pragma config FCMEN=OFF //Cambio de reloj externo a interno en caso de fallas
#pragma config LVP=ON //Programaci�n en low voltage permitida

// CONFIG2
#pragma config WRT=OFF //Proteccion de autoescritura por el programa desactivada
#pragma config BOR4V=BOR40V //Reinicio abajo de 4V


////////////////////////////////////////////////////////////////////////////////
//*****************************subrutinas***************************************
////////////////////////////////////////////////////////////////////////////////
void setup(void);
void setupINTOSC(void);
void setupADC(void);
void setupPWM(void);
void setupTMR2(void);

////////////////////////////////////////////////////////////////////////////////
//*****************************interrupciones***********************************
////////////////////////////////////////////////////////////////////////////////
void __interrupt() isr(void)
{
    if (ADIF == 1) //bandera de conversion encendida
    {
        if(ADCON0bits.CHS == 0) //canal analogo AN0
        {
            CCPR1L = (ADRESH>>1)+124; // se guarda el resultado en una variable
            // de esta manera para tomar el angulo de 180 grados

            //precision de significancia de bits
            CCP1CONbits.DC1B1 = ADRESH & 0b01;
            CCP1CONbits.DC1B0 = (ADRESL>>7);

            ADCON0bits.CHS = 1; //fixed ref
        }

//SEGUNDO SERVOMOTOR
        else
        {
            CCPR2L = (ADRESH>>1)+124; // se guarda el resultado en una variable
            // de esta manera para tomar el angulo de 180 grados

            //precision de significancia de bits
            CCP2CONbits.DC2B1 = ADRESH & 0b01;
            CCP2CONbits.DC2B0 = (ADRESL>>7);

            ADCON0bits.CHS = 0; //fixed ref
        }

        //REINICIO DEL CICLO
        __delay_us(20);             //delay de funcionamiento
        PIR1bits.ADIF = 0;          //bandera reseteada
        ADCON0bits.GO = 1;          //comienza la conversion de nuevo
    }
}

void main(void)
{
  setup();
  setupINTOSC();
  setupADC();
  setupPWM();
  setupTMR2();

    //INTERRUPCIONES
    PIR1bits.ADIF = 0; //conversion del A/D sin terminar
    PIE1bits.ADIE = 1; //interrupcion del  ADC
    INTCONbits.PEIE = 1;//habilitacion de perifericos
    INTCONbits.GIE  = 1;//habilitacion de interrupciones globales
    ADCON0bits.GO = 1;//ciclo A/D en proceso
    while (1)
    {
    }
}

////////////////////////////////////////////////////////////////////////////////
//*************************CONFIGURACION RELOJ INTERNO**************************
////////////////////////////////////////////////////////////////////////////////
void setupINTOSC(void)
{
    OSCCONbits.IRCF = 0b0111;// reloj configurado a 8MHz
    OSCCONbits.SCS = 1;//oscilador interno configurado
}

////////////////////////////////////////////////////////////////////////////////
//*****************************CONFIGURACION E/S********************************
////////////////////////////////////////////////////////////////////////////////
void setup(void)
{
  ANSELH = 0; //Entradas/salidas digitales
  //inputs: RA0 y RA1
  TRISA  = 3;

  ANSELbits.ANS0  = 1;//RA0 analogo
  ANSELbits.ANS1  = 1;//RA1 analogo

  //limpieza en los puertos
  TRISC  = 0;
  PORTA  = 0;
  PORTC  = 0;
}

////////////////////////////////////////////////////////////////////////////////
//*****************************CONFIGURACION ADC********************************
////////////////////////////////////////////////////////////////////////////////
void setupADC(void)
{
  ADCON0bits.ADCS = 2; //frecuencia de Fosc/32

  ADCON0bits.CHS0 = 0; //canal AN0
  ADCON1bits.VCFG1 = 0;//Vss
  ADCON1bits.VCFG0 = 0;//Vdd
  ADCON1bits.ADFM = 0;//justificacion izquierda
  ADCON0bits.ADON = 1;//ADC habilitado
  __delay_us(20); //delay de funcionamiento
}

////////////////////////////////////////////////////////////////////////////////
//*****************************CONFIGURACION ADC********************************
////////////////////////////////////////////////////////////////////////////////
void setupPWM(void)
{
  //SERVOMOTOR #1
  TRISCbits.TRISC2 = 1;//input CCP1;

  PR2 = 250;
  CCP1CONbits.P1M = 0; //PWM output confg: CCP1M <3:2>
  CCP1CONbits.CCP1M = 0b1100; //PWM mode

  CCPR1L = 0x0f;//initial work cycle
  CCP1CONbits.DC1B = 0; //precision

  //SERVOMOTOR #2
  TRISCbits.TRISC1 = 1;//CCP2: input ;
  CCP2CONbits.CCP2M = 0b1100;//PWM mode

  CCPR2L = 0x0f;//initial work cycle
  CCP2CONbits.DC2B1 = 0;//precision
}

////////////////////////////////////////////////////////////////////////////////
//*****************************CONFIGURACION TMR2*******************************
////////////////////////////////////////////////////////////////////////////////
void setupTMR2(void)
{
  PIR1bits.TMR2IF = 0; //no hay match
  T2CONbits.T2CKPS = 0b11;//prescaler de 1:16
  T2CONbits.TMR2ON = 1; //se enciende el TMR 2
  while(PIR1bits.TMR2IF == 0);// mientras no haga match
  PIR1bits.TMR2IF = 0; //no hay match
  TRISCbits.TRISC2 = 0; //C2 es output
  TRISCbits.TRISC1 = 0;//C1 es output
}
