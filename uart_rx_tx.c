// uart_rx_tx.c
// Demonstrate transmitting and receiving characters 
// with the PIC16F1769 UART1 peripheral.
// PJ, 2018-12-28 adapted from pic16f1778 version.
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
    char buf[80];
    char* buf_ptr;
    OSCCONbits.IRCF = 0b1110; // 8Mhz into 4xPLL to give FOSC=32MHz
    TRISCbits.TRISC6 = 0; // output to LED on pin 8
    ANSELCbits.ANSC6 = 0; // enable digital input
    LATCbits.LATC6 = 1;
    uart1_init(38400);
    printf("\r\nDemonstration board with PIC16F1769.");
    while (1) {
        __delay_ms(500);
        LATCbits.LATC6 ^= 1;
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
