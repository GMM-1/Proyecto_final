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

 #define _XTAL_FREQ 4000000
 #define _tmr0_value 176
 #define adressEEPROM 0x00
 #include <xc.h>

//********************************************************
//VARIABLES
//******************************************************************
uint8_t VAL;
uint8_t VAL1;
uint8_t VAL2;
uint8_t VAL3;
uint8_t PWM1;
uint8_t PWM2;
uint8_t POT3;
uint8_t POT4;
uint8_t val1;
uint8_t val2;
uint8_t val3;
uint8_t val4;
uint8_t VALOR = 0;
uint8_t VALOR1;
uint8_t VALOR2;
uint8_t FLAG;

//*********************************************************************
//PROTOTIPOS DE FUNCIONES
//******************************************************************************
void setup(void);
void canales(void);
void MTMR0(void);

//******************************************************************************
//INTERRUPCIONES
//******************************************************************************
void __interrupt() isr(void)
{
    //interrupcion del ADC
    if(PIR1bits.ADIF == 1) //si la conversion del ADC esta completa
    {
        switch(ADCON0bits.CHS) // switch a los canales analogicos
        {
            //ADRESH == resultado del ADC
            case 0:
                VAL = ADRESH; //asignamos valor a pot1
                break;
            case 1:
                VAL1 = ADRESH; //asignamos valor a pot2
                break;
            case 2:
                VAL2 = ADRESH; //asignamos valor a pot3
                break;
            case 3:
                VAL3 = ADRESH; //asignamos valor a pot4
                break;
        }
        PIR1bits.ADIF = 0; //apagamos la bandera del ADC
    }

    //interrupcion del TIMER0
    if(INTCONbits.T0IF == 1) //si la bandera del Timer esta encendida (hizo overflow)
    {
        PWM1++; //se incrementa el contador del PWM
        if(PWM1 <= POT3) //el valor del periodo depende del POT
        {
            PORTCbits.RC3 = 1; //se enciende el pin
        }
        else
        {
            PORTCbits.RC3=0; //apaga el pin
        }
        if(PWM1 <= POT4)
        {
            PORTCbits.RC4 = 1;
        }
        else
        {
            PORTCbits.RC4 = 0;
        }
        if(PWM1 >= 250)//si se cumplen los 20ms reiniciar la variable
        {
          PWM1 = 0;
        }
        TMR0 = _tmr0_value; //se inicia el TMR0
        INTCONbits.T0IF = 0; //se limpia la bandera del timer
    }

//************************************************************************
//CONFIGURACION
//*************************************************************************
void setup(void)
{
    //confg puertos
    ANSEL = 0B00011111; //pines digitales al puerto A
    ANSELH = 0x00; //puerto B digital

    TRISA = 0B00011111; //outputs
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISBbits.TRISB3 = 1;
    TRISC = 0B10000000;
    TRISD = 0B00;

    //inicializamos los puertos
    PORTA = 0x00;
    PORTD = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;

    //PULLUPS
    IOCB = 0xFF; //interrupt on change = 1
    OPTION_REGbits.nRBPU = 0; //habilitamos los pull ups internos
    WPUB = 0B00001111; //bits colocados pullups (RB0, RB1, RB2, RB3)

    //configuracion del TMR0
    OPTION_REG = 0B00001000; //preescaler 1:2 al Warchdog timer reset (contador automatico)
    TMR0 = _tmr0_value; //inicializamos el TMR0
    INTCONbits.GIE = 1; //global interrupts enable
    INTCONbits.PEIE = 1; // periphel interrupts enable
    INTCONbits.T0IE = 1; //interrupcion del Timer0 activada
    INTCONbits.T0IF = 0; //bandera del timer0 limpiada
    INTCONbits.RBIE = 1; //change on interrupt del puerto B
    INTCONbits.RBIF = 0; //limpiar bandera PORTB

    //configuracion del oscilador
    OSCCONbits.SCS = 1; //usamos oscilador interno
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 0; // oscilador a 4MHz
    PIR1bits.TMR2IF = 0; //limpiar la bandera del TMR 2
    T2CON = 0x26; //encender TMR2ON Pre:16 Post:5

    //configuracion del modulo ADC
    ADCON0bits.CHS = 0; //canal AN0
    ADCON0bits.CHS = 2; //canal AN2
    __delay_us(100);

    PIE1bits.ADIE = 1; //habilitamos la interrupcion del ADC
    PIR1bits.ADIF = 0; //limpiar bandera de interrupcion del ADC
    ADCON0bits.ADON = 1; //habilitamos ADC
    ADCON0bits.ADCS = 1; //FOSC/8
    ADCON1bits.ADFM = 0; //justificacion a la izquierda
    ADCON1bits.VCFG0 = 0; //voltaje de referencia VDD
    ADCON1bits.VCFG1 = 0; //voltaje de referencia VSS

    //configuracion del PWM
    PR2 = 250; //periodo TMR2
    CCP1CON = 0B00001100; //modo PWM para el ccp1
    CCP2CON = 0B00001111; //modo PWM para el ccp2


//****************************************************************************
//codigo principal
//************************************************************************
void main (void)
{
    setup();
    while (1)
    {
        canales();
    }
}

//**********************************************************
//FUNCIONES
//********************************************************

void canales()
{
    if(ADCON0bits.GO == 0) // si la conversion esta terminada
    {
        switch(ADCON0bits.CHS) //swtch a los canales
        {
            case 0:
                CCPR1L = ((0.247*VAL)+62); //funcion para el servo en el CCP
                VALOR1 = CCPR1L;
                ADCON0bits.CHS = 1; //canal 2
                __delay_us(100); //delay para activar una medicion
                ADCON0bits.GO = 1; //comienza el ciclo del ADC
                break;

            case 1: //PWM codificado
                POT4 = ((0.049*VAL1)+7);
                ADCON0bits.CHS = 2; //canal 0
                __delay_us(250); //delay para activar una medicion
                ADCON0bits.GO = 1; //comienza el ciclo del ADC
                break;

            case 2:
                CCPR2L = ((0.247*VAL2)+62); // funcion para el servo
                VALOR2 = CCPR2L;
                ADCON0bits.CHS = 3; //canal 3
                __delay_us(100); //delay para activar una medicion
                ADCON0bits.GO = 1; //comienza el ciclo del ADC
                break;

            case 3: //PWM codificado
                POT3 = ((0.049*VAL3)+7);
                ADCON0bits.CHS = 0; //canal 1
                __delay_us(250); //delay para activar una medicion
                ADCON0bits.GO = 1; //comienza el ciclo del ADC
                break;

            default:
                break;
        }
    }
}
