// uart.h
// PJ, 2018-01-02, 2018-12-30

#ifndef MY_UART
#define MY_UART
void uart1_init(unsigned int baud);
void putch(char data);
__bit kbhit(void);
char getch(void);
char getche(void);
void uart1_close();

#define XON 0x11
#define XOFF 0x13

#endif
