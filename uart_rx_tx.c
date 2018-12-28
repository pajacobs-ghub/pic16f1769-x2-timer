// uart_rx_tx.c
// Demonstrate transmitting and receiving characters 
// with the PIC16F1778 UART1 peripheral.
// PJ, 2018-01-29
//
#include <xc.h>
#pragma config FOSC = INTOSC
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config MCLRE = ON
#pragma config PPS1WAY = OFF
#pragma config PLLEN = ON


#include "global_defs.h"
#include "uart.h"
#include <stdio.h>

int main(void)
{
    char buf[32];
    char* buf_ptr;
    OSCCONbits.IRCF = 0b1110; // 8Mhz into 4xPLL to give FOSC=32MHz
    TRISBbits.TRISB0 = 0; // output to LED on pin 21
    ANSELBbits.ANSB0 = 0; // enable digital input
    LATBbits.LATB0 = 1;
    uart1_init(38400);
    printf("\r\nDemonstration board with PIC16F1778.");
    while (1) {
        __delay_ms(500);
        LATBbits.LATB0 ^= 1;
        printf("\r\nEnter some text: ");
        // Characters are echoed as they are typed.
        // Backspace deleting is allowed.
        buf_ptr = gets(buf); 
        printf("\rEntered text was: ");
        if (buf_ptr) { puts(buf); }
        printf("\r\nNow, do it again.");
    }
    uart1_close();
    return 0;
} // end main
