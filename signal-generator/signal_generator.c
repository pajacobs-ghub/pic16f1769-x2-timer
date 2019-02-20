// signal_generator.c
//
// Bare-bones signal generator for shock-speed timer.
// This simulates the 10 km/s shock passing the two
// upstream pressure sensors.
// Based on a PIC18F14K22 MCU as shown on page 27 
// of workbook from September 2010.
//
// PJ, 28-Sep-2010
// Update to XC8 2.05 20-Feb-2019
// 

// Clock frequency in Hz
#define _XTAL_FREQ (16000000L)
#include "xc.h"

#pragma config FOSC = IRC
#pragma config PLLEN = OFF
#pragma config FCMEN = ON
#pragma config IESO = OFF
#pragma config PWRTEN = ON
#pragma config BOREN = OFF
#pragma config WDTEN = OFF
#pragma config MCLRE = ON
#pragma config LVP = OFF
#pragma config XINST = OFF

#define OUT1 (LATBbits.LATB5)
#define OUT2 (LATBbits.LATB6)

void main( void )
{
    // Initialize hardware.
    // In the config bits above, we have selected the internal
    // high-speed oscillator, which defaults to 1MHz.
    OSCCON |= 0b01110000; // set IRFC<2:0> for 16MHz F_OSC
    ANSELHbits.ANS11 = 0; // enable digital buffer on RB5
	TRISB = 0b10011111; // RB5, RB6 output
	LATB = 0;
    __delay_ms(5);
    
    // 50 microseconds = 200 instruction cycles @ F_OSC = 16 MHz
    OUT1 = 1;
    __delay_us(50);
    OUT2 = 1;
    
    __delay_ms(100);
    OUT1 = 0;
    OUT2 = 0;
    while ( 1 ) ; // hang around forever        
}
