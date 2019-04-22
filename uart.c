// uart.c
// Functions to provide a shim between the C-standard library functions
// and UART1 peripheral device on the PIC16F1769 microcontroller.
// PJ,
// 2018-12-28 Adapted from PIC16F1778 example.
// 2018-12-30 RTS/CTS flow control

#include <xc.h>
#include "global_defs.h"
#include "uart.h"
#include <stdio.h>
#include <conio.h>

void uart1_init(unsigned int baud)
{
    unsigned int brg_value;
    // Configure PPS MCU_RX=RB5, MCU_TX=RB7 (like PIC18F14K22-I/P)
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RXPPS = 0b01101; // RB5 is default
    RB7PPS = 0b10110; // UART TX
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    ANSELBbits.ANSB7 = 0;
    TRISBbits.TRISB7 = 0; // TX is output
    LATBbits.LATB7 = 1; // idle high
    TRISBbits.TRISB5 = 1; // RX is input
    ANSELBbits.ANSB5 = 0; // enable digital input
    // Hardware Flow Control
    // Host-CTS# RA2 pin 17, MCU output
    ANSELAbits.ANSA2 = 0;
    TRISAbits.TRISA2 = 0;
    // Start out saying that it is not clear to send.
    LATAbits.LATA2 = 1;
    // Host-RTS# RC0 pin 16, MCU input
    ANSELCbits.ANSC0 = 0;
    TRISCbits.TRISC0 = 1;
    // Use 8N1 asynchronous
    TX1STAbits.SYNC = 0;
    BAUD1CONbits.BRG16 = 0;
    TX1STAbits.BRGH = 1;
    brg_value = (unsigned int) (FOSC/baud/16 - 1);
    // for 32MHz, 38400 baud, expect value of 51.
    SP1BRGH = 0; // high byte
    SP1BRGL = brg_value & 0x00ff; // low byte
    TX1STAbits.TXEN = 1;
    RC1STAbits.CREN = 1;
    RC1STAbits.SPEN = 1;
    return;
}

void putch(char data)
{
    // Wait until PC/Host is requesting.
    while (PORTCbits.RC0) { CLRWDT(); }
    // Wait until shift-register empty, then send data.
    while (!TX1STAbits.TRMT) { CLRWDT(); }
    TX1REG = data;
    return;
}

__bit kbhit(void)
// Returns true is a character is waiting in the receive buffer.
//
// Note that this will not work with our hardware RTS/CTS, unless
// we allow the characters come through for a sufficient length of time.
// Will probably have to discard them, to save confusion in getch.
// For a fix, see the code in pic16f18426-pressure-logger.
{
    return (PIR1bits.RCIF);
}

char getch(void)
{
    char c;
    // Let the PC/Host know that it is clear to send.
    LATAbits.LATA2 = 0;
    // Block until a character is available in buffer.
    while (!PIR1bits.RCIF) { CLRWDT(); }
    // Let PC/Host know that it is not clear to send.
    LATAbits.LATA2 = 1;
    // Get the data that came in.
    c = RC1REG;
    // Clear possible overflow error.
    if (RC1STAbits.OERR) { 
        RC1STAbits.CREN = 0;
        NOP();
        RC1STAbits.CREN = 1;
    }
    return c;
}

char getche(void)
{
    char data = getch();
    putch(data); // echo the character
    return data;
}

void uart1_close()
{
    TX1STAbits.TXEN = 0;
    RC1STAbits.CREN = 0;
    RC1STAbits.SPEN = 0;
    return;
}
