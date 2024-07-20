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
#include <string.h>

#define MCU_RTSn LATAbits.LATA2
#define MCU_CTSn PORTCbits.RC0

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
    // MCU-RTS# (Host-CTS#) RA2 pin 17, MCU output
    ANSELAbits.ANSA2 = 0;
    TRISAbits.TRISA2 = 0;
    // Start out saying that it is not clear to send.
    MCU_RTSn = 1;
    // MCU_CTS# (Host-RTS#) RC0 pin 16, MCU input
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

int getch(void)
{
    char c;
    // Request that the PC/Host send data.
    MCU_RTSn = 0;
    // Block until a character is available in buffer.
    while (!PIR1bits.RCIF) { CLRWDT(); }
    // Let PC/Host know that it should not send data.
    MCU_RTSn = 1;
    // Get the data that came in.
    c = RC1REG;
    // Clear possible overflow error.
    if (RC1STAbits.OERR) { 
        RC1STAbits.CREN = 0;
        NOP();
        RC1STAbits.CREN = 1;
    }
    return (int)c;
}

char getche(void)
{
    char data = (char)getch();
    putch(data); // echo the character
    return data;
}

void uart1_close(void)
{
    TX1STAbits.TXEN = 0;
    RC1STAbits.CREN = 0;
    RC1STAbits.SPEN = 0;
    return;
}

// Convenience functions for strings.

int getstr(char* buf, int nbuf)
// Read (without echo) a line of characters into the buffer,
// stopping when we see a return character.
// Returns the number of characters collected,
// excluding the terminating null char.
{
    int i = 0;
    char c;
    uint8_t done = 0;
    while (!done) {
        c = (char)getch();
        if (c != '\n' && c != '\r' && c != '\b' && i < (nbuf-1)) {
            // Append a normal character.
            buf[i] = c;
            i++;
        }
        if (c == '\r') {
            // Stop collecting on receiving a carriage-return character.
            done = 1;
            buf[i] = '\0';
        }
        if (c == '\b' && i > 0) {
            // Backspace.
            i--;
        }
    }
    return i;
}

void putstr(char* str)
{
    for (size_t i=0; i < strlen(str); i++) putch(str[i]); 
    return;
}
