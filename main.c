/*
 * File:                main.c
 * Author:              Equipo 3 (A01254123, A01384683, A00833414)
 * Development board:   Curiosity HPC
 * MCU:                 PIC18F45K50
 */

//++++++++++++++++++++++++++++LIBRARIEs SECTION+++++++++++++++++++++++++++++++++
#include "main.h"
#include <stdint.h>
#include <xc.h>
#include "myprintf.h"
#include "i2c.h"
#include "color.h"
//++++++++++++++++++++++++++++DIRECTIVEs SECTION++++++++++++++++++++++++++++++++
//Defines
#define _XTAL_FREQ 16000000//       MACRO for __delay_ms() function

#define RLED LATDbits.LATD5
#define GLED LATDbits.LATD6
#define BLED LATDbits.LATD7

#define ENABLER_TIMER0       T0CONbits.TMR0ON
#define TIMER0_FLAG          INTCONbits.TMR0IF
#define SOFT_PWM_STATUS_PIN  LATCbits.LATC5

#define WHITE_COLOR    (uint8_t[]){ 255, 255, 255}
#define RED_COLOR      (uint8_t[]){ 255,   0,   0}
#define LIME_COLOR     (uint8_t[]){   0, 255,   0}
#define BLUE_COLOR     (uint8_t[]){   0,   0, 255}
#define YELLOW_COLOR   (uint8_t[]){ 255, 255,   0}
#define CYAN_COLOR     (uint8_t[]){   0, 255, 255}
#define MAGENTA_COLOR  (uint8_t[]){ 255,   0, 255}
#define BLACK_COLOR    (uint8_t[]){   0,   0,   0}
#define MAROON_COLOR   (uint8_t[]){ 127,   0,   0}
#define GREEN_COLOR    (uint8_t[]){   0, 127,   0}
#define NAVY_COLOR     (uint8_t[]){   0,   0, 127}
#define CUSTOM_COLOR_A (uint8_t[]){ 169,   3, 252}

//Enumerations
enum timer0 { PWMtimer = 0b01000001, count_high = 0x00, count_low = 0x00 };

//Variables, constants
uint8_t pwmvalue = 0;
uint8_t actualColor[3] = {0, 0, 0};

const uint8_t hNibble = 4;
const uint8_t twentyMS = 20;
const uint8_t notKey = 0xFF;
uint8_t key;

const uint8_t keys[4][4] = {
    { 1,  2,  3, 10}, 
    { 4,  5,  6, 11}, 
    { 7,  8,  9, 12}, 
    {15,  0, 14, 13}
};

uint16_t measure1[3] = {0, 0, 0};
uint16_t minError[3] = {65535, 65535, 65535};
uint8_t  minErrorPWM[3] = {0, 0, 0};

//+++++++++++++++++++++++++++++++ISRs SECTION+++++++++++++++++++++++++++++++++++
//ISR vector (address: 0x0008) initialization
__interrupt( high_priority ) void high_isr( void ){
    if( TIMER0_FLAG ) {
        TIMER0_FLAG = 0;
        
        TMR0L = SOFT_PWM_STATUS_PIN ? (pwmvalue) : (255-pwmvalue);
        SOFT_PWM_STATUS_PIN = ~SOFT_PWM_STATUS_PIN;
        
        if(pwmvalue == 0) LATCbits.LATC0 = 0;
        else if(pwmvalue == 255) LATCbits.LATC0 = 1;
        else LATCbits.LATC0 = SOFT_PWM_STATUS_PIN;
    }
}

//++++++++++++++++++++++++FUNCTIONs DECLARATION+++++++++++++++++++++++++++++++++
void CLK_Initialize( void );
void PORT_Initialize( void );
void UART_Initialize( void );
void CCP_PWM_Initialize( void );
void Software_PWM_Initialize( void );

void TMR2_Initialize( void );

void dutyCycle1_Update( float duty );
void dutyCycle2_Update( float duty );
void dutyCycle3_Update( uint8_t duty );

void ColorValues_Display( void );
void Interrupts_Initialize( void );
void lectureLEDone( void );
void calibration( void );

void UART_TransmitByte( uint8_t data );
void UART_TransmitNum(uint16_t num);

void updateActualColor(uint8_t rgb_values[3]);
void colorDisplay(uint8_t rgb_values[3]);
uint8_t getKey( void );
void Keyboard_Functions ( uint8_t pressedKey );

//+++++++++++++++++++++++++++++MAIN SECTION+++++++++++++++++++++++++++++++++++++
void main() {
    CLK_Initialize( );
    PORT_Initialize( );
    UART_Initialize( );
    I2C_Init_Master(I2C_100KHZ);//      Inicializa el protocolo I2C en modo maestro
    Color_Initialize( );
    CCP_PWM_Initialize( );//            CCP initializations as PWM
    TMR2_Initialize( );//               Timer 2 initializations
    Interrupts_Initialize( );
    
    Software_PWM_Initialize();
    
    while(1) {
        My_Color_Read();
        Color_Display();
        
        ColorValues_Display();
        
        key = getKey();//                           call the function to get the Key
        
        //   show the value of Key in LEDs    
        if( key != notKey ){//                      if a Key was returned
            while( ( PORTB & 0b00111100 ) != 0b00111100 );//          wait until Key is released
            LATA = (uint8_t)( key << hNibble );
            Keyboard_Functions(key);
        }
    }
}

//++++++++++++++++++++++++++FUNCTIONs SECTION+++++++++++++++++++++++++++++++++++
void CLK_Initialize( void ){
    OSCCONbits.IRCF = 0b111;      // set HFINTOSC to 16 MHz
    SLRCON = 0;                   // set a standard slew rate in pin PORTS 
}
void Interrupts_Initialize( void ){
    RCONbits.IPEN = 0;            // disables priority levels on interrupts
    INTCONbits.PEIE = 0;          // disables all peripheral interrupts
    TIMER0_FLAG = 0;              // clears TMR0 overflow interrupt flag
    INTCON2bits.INTEDG0 = 0;      // INT0 interrupt on falling edge
    INTCONbits.TMR0IE = 1;        // enables the TMR0 overflow interrupt
    INTCONbits.PEIE = 1;          // enables all unmasked core interrupts
    INTCONbits.GIE = 1;           // enables all unmasked core interrupts
}
void PORT_Initialize( void ){
    // Pin configurations for RGB LEDs (D5-D7)
    LATD = 0;                     // clear PORTD data latches
    TRISD = TRISD & 0b00011111;   // set RD5-RD7 as output
    
    // Pin configurations for the key status LEDs (A4-A7) and Keypad Outputs (A0-A3)
    LATA = 0;                     // clear PORTA data latches
    TRISA = TRISA & 0b00000000;   // set RA7-RA0 as output
    
    // Pin configurations for the Keypad Inputs (B2-B5)
    LATB = 0;                     // clear PORTB data latches
    TRISB = 0;                    // set PORTB as output
    ANSELB = ANSELB & 0b11000011; // enable digital input buffer in RB3-RB5
    TRISB = TRISB | 0b00111100;   // set RB2-RB5 as input    
    WPUB = WPUB | 0b00111100;     // connect an internal resistor in pins RB2-RB5
    INTCON2bits.RBPU = 0;         // enable the internal pull-ups in PORTB
    
    // Pin configurations for UART (C6-C7)
    ANSELCbits.ANSC6 = 0;         // configure RC6 (TX) pin as:
    TRISCbits.TRISC6 = 1;         // EUSART asynchronous transmit data output
    ANSELCbits.ANSC7 = 0;         // configure RC7 (RX) pin as:
    TRISCbits.TRISC7 = 1;         // EUSART asynchronous receive data in
    
    // Pin configurations for I2C (B0-B1)
    ANSELBbits.ANSB1 = 0;         // 
    TRISBbits.TRISB1 = 1;         // set pin RB1 as MSSP I2C clock output
    ANSELBbits.ANSB0 = 0;         // 
    TRISBbits.TRISB0 = 1;         // set pin RB0 as MSSP I2C data I/O
    
    // Pin configurations for CCPx output (C1-C2) (CCP2 output is multiplexed with RC1, 
    // this is configured in "main.h" file)
    TRISCbits.TRISC1 = 1;         // disable RC1 as CCP2 PWM output
    TRISCbits.TRISC2 = 1;         // disable RC1 as CCP2 PWM output
    
    // Pin configuration for PWM software (C0,C5)
    LATCbits.LATC0 = 1;           // Clear data latches
    TRISCbits.TRISC0 = 0;         // Set 0 as output
    LATCbits.LATC5 = 1;           // Clear data latches
    TRISCbits.TRISC5 = 0;         // Set 0 as output
}
void UART_Initialize( void ){
    BAUDCON1bits.BRG16 = 1;       // 16-bit baudrate
    //Configure a 9600 BAUDRATE according to TABLE 17-5 of the datasheet
    SPBRGH1 = 0x01;       
    SPBRG1 = 0xA0;
    TXSTA1bits.BRGH = 1;          // high-speed
    TXSTA1bits.SYNC = 0;          // asynchronous mode
    RCSTA1bits.SPEN = 1;          // serial port enabled
    TXSTA1bits.TXEN = 1;          // transmit enabled
    RCSTA1bits.CREN = 0;          // disables receiver 
}
void UART_TransmitByte( uint8_t data ){
    while( TXSTA1bits.TRMT == 0 ); // wait until TSR is empty
    TXREG1 = data;                 // start byte transmission
}
void UART_TransmitNum(uint16_t num){
    uint32_t k = 100000;
    uint8_t d = 0;
    uint8_t fd_flag = 0;
    for (uint8_t i = 5; i > 0; i--){
        d = (uint8_t)((num%k)/(k/10));
        fd_flag = fd_flag ? 1 : (d ? 0 : 1);
        UART_TransmitByte(d + '0');
        k /= 10;
    }
}
void TMR2_Initialize( void ){
    T2CON = 0b00000001;            // configure the TMR2 prescaler 1:4
    TMR2 = 0;                      // clear TMR2 count   
    PIR1bits.TMR2IF = 0;           // clear TMR2 interrupt flag
    T2CONbits.TMR2ON = 1;          // Timer 2 enabled
    // in order to send a complete duty cycle and period on the 1st PWM output
    while( !PIR1bits.TMR2IF ){     // wait until TMR2 overflows
        TRISCbits.TRISC1 = 0;      // enable the CCP2 pin output
        TRISCbits.TRISC2 = 0;      // enable the CCP2 pin output
    }
}
void CCP_PWM_Initialize( void ){
    PR2 = 249;//                        PWM = 4 kHz     
    CCP2CON = 0b00001100;//             CCP2 module: PWM mode
    CCPR2L = 0;//                       initialize duty cycle to 0%  
    CCP1CON = 0b00001100;//             CCP2 module: PWM mode
    CCPR1L = 0;//                       initialize duty cycle to 0%  
}
void Software_PWM_Initialize( void ) {
    TMR0H = count_high;           // load initial count, high-byte first  
    TMR0L = count_low;            // then low-byte      
    TIMER0_FLAG = 0;              // clear the Timer 0 ovf flag
    T0CON = PWMtimer;             // initialize Timer 0 with:
    //                               mode=8, internal instr. clock, PRE=2
    ENABLER_TIMER0 = 1;           // enables Timer 0
}
void dutyCycle1_Update( float duty ){
    uint16_t dutyTemp = (uint16_t)(duty * 4 * (PR2 + 1));// calculating the 10-bit duty  
    CCPR1L = (uint8_t)( dutyTemp >> 2 );//      load (bit10 to bit2) of the 10-bit duty 
    CCP1CONbits.DC1B = dutyTemp & 0x0003;//     load (bit1 and bit0) of the 10-bit duty
}
void dutyCycle2_Update( float duty ){
    uint16_t dutyTemp = (uint16_t)(duty * 4 * (PR2 + 1));// calculating the 10-bit duty  
    CCPR2L = (uint8_t)( dutyTemp >> 2 );//      load (bit10 to bit2) of the 10-bit duty 
    CCP2CONbits.DC2B = dutyTemp & 0x0003;//     load (bit1 and bit0) of the 10-bit duty
}
void dutyCycle3_Update( uint8_t duty ){
    pwmvalue = duty;
}
void updateActualColor(uint8_t rgb_values[3]){
    actualColor[0] = rgb_values[0];
    actualColor[1] = rgb_values[1];
    actualColor[2] = rgb_values[2];
    colorDisplay(rgb_values);
}
void colorDisplay(uint8_t rgb_values[3]){    
    dutyCycle1_Update((float)rgb_values[0]/(float)255);
    dutyCycle2_Update((float)rgb_values[1]/(float)255);
    dutyCycle3_Update(rgb_values[2]);
}

uint8_t getKey( void ){
    for(uint8_t column = 0; column < 4; column++){
        LATA = LATA | 0b00001111;                       // disable with '1' all the columns of the keypad 
        LATA &= ~(1 << column);                         // enable with '0' the Ci
        for(uint8_t row = 0; row < 4; row++){
            if( (PORTB & (1 << (row + 2))) == 0){       // if we find the '0' in Ri 
                __delay_ms( twentyMS );                 // call a delay to debounce the input
                if( (PORTB & (1 << (row + 2))) != 0 ){  // double checking
                    return notKey;                      // if button was not pressed, no match
                }
                return keys[row][column];               // if button was really pressed, assign value of according to pressed key. 
            }
        }
    }
    return notKey;
}
void lectureLEDone( void ) {
    // Medicion (promedio)
    uint8_t samplesNumber = 10;
    uint32_t avg = 0;
    uint16_t colorReadArray[3] = {0,0,0};
    uint8_t measuredColor[3] = {0,0,0};
     
    for (uint8_t n = 0; n < 3; n++){
        measuredColor[0] = 0;
        measuredColor[1] = 0;
        measuredColor[2] = 0;
        measuredColor[n] = actualColor[n];
        
        colorDisplay(measuredColor); 

        __delay_ms( 20 );
        
        avg = 0;  
        
        for (uint8_t k = 0; k < samplesNumber; k++){
            Color_Read_RGB(n);
            colorReadArray[0] = RedData;
            colorReadArray[1] = GreenData;
            colorReadArray[2] = BlueData;

            avg += colorReadArray[n];
        }
        measure1[n] = (uint16_t)(avg/samplesNumber);
        __delay_ms( 5 ); 
    }
    
    //colorDisplay((uint8_t[]){actualColor[0],actualColor[1],actualColor[2]});
    colorDisplay(actualColor);
    
    myprintf("\nLED1 Measured Values:\nR: ");
    UART_TransmitNum(measure1[0]);
    myprintf(", G: ");
    UART_TransmitNum(measure1[1]);
    myprintf(", B: ");
    UART_TransmitNum(measure1[2]);
    myprintf("\n\n");
}

void calibration( void ) {
    // Medicion (promedio)
    uint8_t  samplesNumber = 2;
    uint32_t avg = 0;
    uint16_t measure2[3];
    uint16_t error;
    uint8_t  measuredColor[3] = {0,0,0};
    
    // Initialize error at maximum
    minError[0] = 65535;
    minError[1] = 65535;
    minError[2] = 65535;
    
    // Iteraciones para calibracion
    uint8_t colorReadArray[3] = {0,0,0};
    
    for(uint8_t m = 0; m < 3; m++){
        measuredColor[0] = 0;
        measuredColor[1] = 0;
        measuredColor[2] = 0;
        
        colorDisplay(BLACK_COLOR);
        __delay_ms( 20 );
        
        for(uint16_t j = 0; j <= 255; j++){
            avg = 0;
            measuredColor[m] = (uint8_t)j;
            colorDisplay(measuredColor);
            
            for (uint8_t k = 0; k < samplesNumber; k++) {
                Color_Read_RGB(m);
                if (m == 0) avg += RedData;
                if (m == 1) avg += GreenData;
                if (m == 2) avg += BlueData;
                
            }
            
            // Calcula y guarda promedio
            measure2[m] = (uint16_t)(avg/samplesNumber);
            //myprintf("Measure:  ");
            //UART_TransmitNum(measure2[m]);
            
            error = (measure2[m] > measure1[m]) ? (measure2[m] - measure1[m]) : (measure1[m] - measure2[m]);
            //myprintf(", Error:  ");
            //UART_TransmitNum(error);
            
            if (error < minError[m]){
                minError[m] = error;
                minErrorPWM[m] = (uint8_t)j;
                //myprintf(", Actualizado:  ");
                //UART_TransmitNum(minErrorPWM[m]);
            }
            //myprintf("\n");
        }
    }
    
    // Generar color
    updateActualColor(minErrorPWM);
    myprintf("\nPWM Generado:  R: ");
    UART_TransmitNum(minErrorPWM[0]);
    myprintf(", G: ");
    UART_TransmitNum(minErrorPWM[1]);
    myprintf(", B: ");
    UART_TransmitNum(minErrorPWM[2]);
    myprintf("\n\n");
}

void ColorValues_Display( void ){
    myprintf(", R: ");
    UART_TransmitNum(RedData);

    myprintf(", G:  ");
    UART_TransmitNum(GreenData);

    myprintf(", B:  ");
    UART_TransmitNum(BlueData);

    myprintf(" : H: ");
    UART_TransmitNum((uint16_t)Hue);

    myprintf(", S: ");
    UART_TransmitNum(Saturation);

    myprintf(", V: ");
    UART_TransmitNum(Value);

    UART_TransmitByte('\n');
}

void Keyboard_Functions ( uint8_t pressedKey ){
    switch( pressedKey ){
        case 1:
            updateActualColor(WHITE_COLOR);
            break;
        case 2:
            updateActualColor(RED_COLOR);
            break;
        case 3:
            updateActualColor(LIME_COLOR);
            break;
        case 4:
            updateActualColor(BLUE_COLOR);
            break;
        case 5:
            updateActualColor(YELLOW_COLOR);
            break;
        case 6:
            updateActualColor(CYAN_COLOR);
            break;
        case 7:
            updateActualColor(MAGENTA_COLOR);
            break;
        case 8: 
            updateActualColor(MAROON_COLOR);
            break;
        case 9:
            updateActualColor(GREEN_COLOR);
            break;
        case 0:
            updateActualColor(NAVY_COLOR);
            break;
        case 10:
            updateActualColor(CUSTOM_COLOR_A);
            break;
        case 13:
            updateActualColor(minErrorPWM);
            break;
        case 14:
            calibration();
            break;
        case 15:
            lectureLEDone();
            break;
        default:
            updateActualColor(BLACK_COLOR);
            break;
    }
}
//+++++++++++++++++++++++++++++++++++END++++++++++++++++++++++++++++++++++++++++