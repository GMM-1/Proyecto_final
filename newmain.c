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
unsigned char I[72] = " \n Bienvenido, presione 1 para continuar con la comunicacion serial \n ";
unsigned char R[60] = " \n Que servomotor desea mover? \n 1) PD \n 2) FI \n 3) CD \n 4) CI \n ";
unsigned char M[36] = " \n Ingrese un numero entre 0 y 9 \n ";

//*********************************************************************
//PROTOTIPOS DE FUNCIONES
//******************************************************************************
void setup(void);
void canales(void);
void MTMR0(void);
void escribir(uint8_t data, uint8_t address);
uint8_t leer(uint8_t address);
void UART(void);
void INS(void);
void OTRO(void);
void MENSAJE(void);

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
    //interrupcion del PORTB
    if(INTCONbits.RBIF == 1) //habiilitacion del change on interrupt flag
    {
        if(PORTBbits.RB2 == 0) //si presionamos el push de b2 (modo UART)
        {
            FLAG = 1; //activamos la bandera del UART
            while(FLAG == 1)
            {
                TXSTAbits.TXEN = 1; //transmicion habilitada
                UART(); //rutina para imprimir caracteres y funciones
            }
            TXSTAbits.TXEN = 0; //transmicion deshabilitada
        }
        if(PORTBbits.RB0 == 0) //si presionamos el push b0 (modo EEPROM READ)
        {
            PORTDbits.RD0 = 1; //led indicador de modo
            PORTDbits.RD1 = 0; //led indicador de modo
            escribir(VALOR1, 0x10); //guardar en memoria EEPROM primer valor
            escribir(VALOR2, 0x12); //guardar en memoria EEMPROM segundo valor
            escribir(POT3, 0x12); //guardar en memoria EEPROM tercer valor
            escribir(POT4, 0x13); //guardaarr en memoria EEPROM cuarto valor
            __delay_ms(500);
        }
        if(PORTBbits.RB1 == 0) //si presionamos el push b1 (modo EEPROM WRITE )
        {
            ADCON0bits.ADON = 0; //deshablitamos el ADC
            PORTDbits.RD0 = 0; //led indicador de modo
            PORTDbits.RD1 = 1; //led indicador de modo
            val1 = leer(0x10); //extraemos valor val1 de la eeprom
            val2 = leer(0x11); //extraemos valor val2 de la eeprom
            val3 = leer(0x12); //extraemos valor val3 de la eeprom
            val4 = leer(0x13); //extraemos valor val4 de la eeprom

            CCPR1L = val1; //valor del ccp1 asignado a val1
            CCPR2L = val2; //valor del ccp2 asignado a val2
            POT3 = val3; //valor del pot3 asignado a val3
            POT4 = val4; //valor del pot4 asignado a val4
            __delay_ms(3000);
            ////////////////////////////////////////////////////////////////////
            //////////////////////ARREGLAR/////////////////////////////////////
            //if(PORTBbits.RB3 == 0)
            //{
                ADCON0bits.ADON = 1; //hablitamos enable del adc
            //}
            ////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
        }
        INTCONbits.RBIF = 0; //limpiar la bandera del PORTB (must be cleared in software)
    }
    PIR1bits.TMR2IF = 0; // se limpia la bandera del TMR2 (no match de TMR2 - PR2
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

    //configuracion del UART
    PIR1bits.RCIF = 0; //flag del reciever apagada
    PIE1bits.RCIE = 0; //interrupcion del reciever activada
    PIE1bits.TXIE = 0; //interrupcion del transmiter activada
    TXSTAbits.TX9 = 0; //transmicion de 8 bits
    TXSTAbits.TXEN = 1;//transmicion habilitada
    TXSTAbits.SYNC = 0; //modo asincrono
    TXSTAbits.BRGH = 1; //high speed
    RCSTAbits.RX9 = 0; // recepcion de 8 bits
    RCSTAbits.CREN = 1; //receptor habilitado
    RCSTAbits.SPEN = 1; // entrada y salida

    //generador de baudios del UART
    BAUDCTLbits.BRG16 = 0; //activar el generador de baudios de 8 bits
    SPBRG = 25; //9600
    SPBRGH = 1;
}

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

//funcion para escribir en la EEPROM
void escribir(uint8_t data, uint8_t address)
{
    EEADR = address;
    EEDAT = data; //valor a escribir

    EECON1bits.EEPGD = 0; //apuntar a la data memory
    EECON1bits.WREN = 1; //habilitar escritura
    INTCONbits.GIE = 0; //se apagan las interrupciones globales

    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; //iniciamos la escritura

    while(PIR2bits.EEIF == 0); //esperar al final de la escritura
    PIR2bits.EEIF = 0; //apagamos la bandera
    EECON1bits.WREN = 0; //nos aseguramos que no este escribiendo
    INTCONbits.GIE = 0; //habilitar las interrupciones globales
}

//funcion para leer en la EEPROM
uint8_t leer(uint8_t address)
{
    EEADR = address;
    EECON1bits.EEPGD = 0; //apuntar a la program memory
    EECON1bits.RD = 1; //indicar que se leera
    uint8_t data = EEDATA;
    return data;
}
//funcion para escribir en la EEPROM
void escribir(uint8_t data, uint8_t address)
{
    EEADR = address;
    EEDAT = data; //valor a escribir

    EECON1bits.EEPGD = 0; //apuntar a la data memory
    EECON1bits.WREN = 1; //habilitar escritura
    INTCONbits.GIE = 0; //se apagan las interrupciones globales

    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; //iniciamos la escritura

    while(PIR2bits.EEIF == 0); //esperar al final de la escritura
    PIR2bits.EEIF = 0; //apagamos la bandera
    EECON1bits.WREN = 0; //nos aseguramos que no este escribiendo
    INTCONbits.GIE = 0; //habilitar las interrupciones globales
}

//funcion para leer en la EEPROM
uint8_t leer(uint8_t address)
{
    EEADR = address;
    EECON1bits.EEPGD = 0; //apuntar a la program memory
    EECON1bits.RD = 1; //indicar que se leera
    uint8_t data = EEDATA;
    return data;
}

//funcion para escribir en la EEPROM
void escribir(uint8_t data, uint8_t address)
{
    EEADR = address;
    EEDAT = data; //valor a escribir

    EECON1bits.EEPGD = 0; //apuntar a la data memory
    EECON1bits.WREN = 1; //habilitar escritura
    INTCONbits.GIE = 0; //se apagan las interrupciones globales

    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; //iniciamos la escritura

    while(PIR2bits.EEIF == 0); //esperar al final de la escritura
    PIR2bits.EEIF = 0; //apagamos la bandera
    EECON1bits.WREN = 0; //nos aseguramos que no este escribiendo
    INTCONbits.GIE = 0; //habilitar las interrupciones globales
}

//funcion para leer en la EEPROM
uint8_t leer(uint8_t address)
{
    EEADR = address;
    EECON1bits.EEPGD = 0; //apuntar a la program memory
    EECON1bits.RD = 1; //indicar que se leera
    uint8_t data = EEDATA;
    return data;
}

//imprimir el mensaje en la terminal
void UART(void)
{
    __delay_ms(500);
    VALOR = 0;
    do
    {
        VALOR++;
        TXREG = I[VALOR];
        __delay_ms(50);
    }
    while(VALOR<=72); //numero de caracteres
    while(RCIF == 0); //mientras no este full el buffer
    INS();
}

//mensaje a desplegar
void INS(void)
{
    OP = RCREG;
    // " \nBienvenido, presione 1 para
    switch(OP)
    {
        case 49: //si se presiona el 1 opcion manualmente
            __delay_ms(300);
            VALOR = 0;
            do
            {
              VALOR++;
              TXREG = R[VALOR];
              __delay_ms(50);
            }
            while(VALOR<=60); //cantidad de caracteres del array
                while(RCIF == 0);
            OP = 0; //limpiar la variable que hace el cambio
            OTRO();
            break;
        case 50: // si se presiona el 2 opcion de comunicacion serial
            TXSTAbits.TXEN = 0; //apagar la bandera
        OP = 0;
        break;
    }
}

void OTRO(void)
{
    OP = RCREG;
    switch(OP)
    {
        case 49:
            MENSAJE();
            if(RCREG >= 48 && RCREG <= 57)
            {
                VAL = RCREG;
                canales();
            }
            break;
        case 50:
            MENSAJE();
            if(RCREG >= 48 && RCREG <= 57)
            {
                VAL1 = RCREG;
                canales();
            }
            break;
        case 51:
            MENSAJE();
            if(RCREG >= 48 && RCREG <= 57)
            {
                VAL2 = RCREG;
                canales();
            }
            break;
        case 52:
            MENSAJE();
            if(RCREG >= 48 && RCREG <= 57)
            {
                VAL3 = RCREG;
                canales();
            }
            break;
    }
}

void MENSAJE(void)
{
    __delay_ms(500);
    VALOR = 0;
    do
    {
        VALOR++;
        TXREG = M[VALOR];
        __delay_ms(50);
    }
    while(VALOR<=36); //cantidad de caracteres
    while(RCIF == 0)
    {
        OP = 0; //limpiamos la bandera
    }   
}
