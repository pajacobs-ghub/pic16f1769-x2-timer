// x2timer_pic16f1769.c
// X2 trigger and timer box rebuilt around a PIC16F1769-I/P.
// PJ, MechEng, UQ.
// 2018-01-30 start this firmware for PIC16F1778-I/SP.
// 2018-02-02 mode 1 complete with save/restore registers.
// 2019-02-19 port to PIC16F1769
//
// Build with XC8 v2.05 C90 standard 
// because the C99 project option seems to result in 
// unresolved dependencies.
// 
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
#include "HEFlash.h"
#include <conio.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

const char * version_string = "Version 0.1 2019-02-19 PJ";

// Some pin mappings; others are given in init_peripherals().
#define LED_ARM LATCbits.LATC6
#define LED_PWR LATCbits.LATC7

// Parameters controlling the device are stored in an array of registers.
#define NUMREG 4
int16_t registers[NUMREG]; // working copy in SRAM
// register description
// 0        mode
// 1        trigger level as a 10-bit count, 0-1023
// 2        delay-A as a 16-bit count
// 3        delay-B as a 16-bit count

// Text buffer for incoming commands.
// They are expected to be short.
// Be careful, overruns are not handled well.
char cmd_buf[32];

// High Endurance Flash block is used to hold the parameters
// when the power is off.
const char HEFdata[FLASH_ROWSIZE] __at(HEFLASH_START) = {
    1,0, 200,0, 1,0, 1,0,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
};

char save_registers_to_HEF()
{
    return HEFLASH_writeBlock(0, (void*)registers, NUMREG*2);
}

char restore_registers_from_HEF()
{
    return HEFLASH_readBlock((void*)registers, 0, NUMREG*2);
}

void init_peripherals()
{
    // Initializing the UART sets pins RA2,RC0,RB5,RB7.
    uart1_init(38400);
    //
    // Pins RB7,RC4,RC5,RC6,RC7 are digital-outputs.
    TRISBbits.TRISB7 = 0; ANSELBbits.ANSB7 = 0; LATBbits.LATB7 = 0;
    TRISCbits.TRISC4 = 0; LATCbits.LATC4 = 0;
    TRISCbits.TRISC5 = 0; LATCbits.LATC5 = 0;
    // We also want the high-current drive enabled on RC4 and RC5.
    HIDRVC = 0b00110000;
    TRISCbits.TRISC6 = 0; ANSELCbits.ANSC6 = 0; LATCbits.LATC6 = 0; // LED_ARM
    TRISCbits.TRISC7 = 0; ANSELCbits.ANSC7 = 0; LATCbits.LATC7 = 0; // LED_PWR
    //
    // RA5 used as digital-input for arm button on box.
    // Enable its weak pull-up but disable all others.
    WPUA = 0;
    WPUB = 0;
    WPUC = 0;
    TRISAbits.TRISA5 = 1; WPUAbits.WPUA5 = 1;
    OPTION_REGbits.nWPUEN = 0;
    //
    // Fixed voltage reference to 4.096V
    FVRCONbits.ADFVR = 0b11;
    FVRCONbits.CDAFVR = 0b11;
    FVRCONbits.FVREN = 1;
    __delay_ms(10);
    //
    // We want to have analog signals on pins
    // RC1/AN5/CXIN1-, RC2/AN6/C12IN2-
    TRISCbits.TRISC1 = 1; ANSELCbits.ANSC1 = 1; WPUCbits.WPUC1 = 0;
    TRISCbits.TRISC2 = 1; ANSELCbits.ANSC2 = 1; WPUCbits.WPUC2 = 0;
    //
    // Analog to digital converter running on its own RC clock,
    // right-justified results.
    ADCON1bits.ADFM = 1; // right-justified format
    ADCON1bits.ADCS = 0b111; // FRC
    ADCON1bits.ADNREF = 0; // negative reference Vss
    ADCON1bits.ADPREF = 0b11; // positive reference FVR module
    ADCON0bits.ADON = 1;
    //
    // Digital to analog converter 1
    DAC1CON0bits.PSS = 0b10; // FVR_buffer2
    DAC1CON0bits.NSS = 0; // Vss
    DAC1CON0bits.FM = 0; // right justified format
    DAC1CON0bits.EN = 1;
    DAC1REF = (uint16_t)registers[1];
    DACLDbits.DAC1LD = 1;
    //
    // Comparator 1
    CM1NSELbits.NCH = 0b001; // C1IN1-/RC1 pin
    CM1PSELbits.PCH = 0b1010; // DAC1_output
    CM1CON0bits.POL = 1; // invert output polarity
    CM1CON0bits.ON = 1;
    //
    // Comparator 2
    CM2NSELbits.NCH = 0b010; // C2IN2-/RC2 pin
    CM2PSELbits.PCH = 0b1010; // DAC1_output
    CM2CON0bits.POL = 1; // invert output polarity
    CM2CON0bits.ON = 1;
    //
    // Configurable logic cell 3 will be set up later,
    // in the arm_mode() function.
    //
    // Flash the LEDs a moment and then settle to power-on.
    for (int8_t i=0; i < 2; ++i ) {
        LED_ARM = 1; LED_PWR = 1; __delay_ms(500);
        LED_ARM = 0; LED_PWR = 0; __delay_ms(500);
    }
    LED_PWR = 1;
}

void update_dac1(void)
{
    DAC1REF = (uint16_t)registers[1];
    DACLDbits.DAC1LD = 1;
}

uint16_t read_adc(uint8_t chan)
{
    ADCON0bits.CHS = chan;
    ADCON0bits.GO = 1;
    NOP();
    while (ADCON0bits.GO) { NOP(); }
    return ADRES;
}

void arm_and_wait_for_event(void)
{
    int nchar;
    switch (registers[0]) {
        case 1:
            nchar = printf("armed mode 1, both outputs immediate. ");
            if (CMOUTbits.MC1OUT) {
                printf("C1OUT already high. fail");
                break;
            }
            // Set up CLC3 to extend the Comparator pulse indefinitely.
            CLC3CONbits.EN = 0;
            CLC3CONbits.MODE = 0b100; // D flip-flop
            CLC3POLbits.POL = 0; // not inverted
            CLC3SEL0bits.D1S = 0x000111; // sync_C1OUT as data1 for 
            CLC3SEL1bits.D2S = 0; // none (CLCIN0 via PPS, but is ignored)
            CLC3SEL2bits.D3S = 0; // none
            CLC3SEL3bits.D4S = 0; // none
            // Steer the output to the MCU pins.
            GIE = 0; PPSLOCK = 0x55; PPSLOCK = 0xaa; PPSLOCKED = 0;
            RC4PPS = 0b00011; // LC3_out
            RC5PPS = 0b00011; // LC3_out also
            PPSLOCK = 0x55; PPSLOCK = 0xaa; PPSLOCKED = 1;
            //
            CLC3GLS0 = 0x10; // gate 1, send data1 through true
            CLC3GLS1 = 0; // gate 2, logic 0
            CLC3GLS2 = 0; // gate 3, logic 0
            CLC3GLS3 = 0; // gate 4, logic 0
            CLC3POLbits.G1POL = 0; 
            CLC3POLbits.G2POL = 1; // invert 0 to get logic 1
            CLC3POLbits.G3POL = 0;
            CLC3POLbits.G4POL = 0;
            CLC3CONbits.EN = 1;
            NOP();
            // At this point we have nothing to do but
            // wait for the comparator to go high.
            LED_ARM = 1;
            while (!CLCDATAbits.MCLC3OUT) { CLRWDT(); }
            __delay_ms(500);
            // End of output pulse; clean up by steering
            // the original data latch values to pins.
            GIE = 0; PPSLOCK = 0x55; PPSLOCK = 0xaa; PPSLOCKED = 0;
            RC4PPS = 0; // Return control to LATC4
            RC5PPS = 0; // Return control to LATC5
            PPSLOCK = 0x55; PPSLOCK = 0xaa; PPSLOCKED = 1;
            CLC3CONbits.EN = 0; // Disable flip-flop
            LED_ARM = 0;
            nchar = printf("triggered. ok");
            break;
        case 2:
            nchar = printf("need to implement mode 2 delay. fail");
            break;
        default:
            nchar = printf("unknown mode. fail");
    }
}

void wait_for_manual_arm(void)
{
    while (1) {
        CLRWDT();
        if (!PORTAbits.RA5) {
            __delay_ms(10); // to be sure that it is not a glitch
            if (!PORTAbits.RA5) {
                arm_and_wait_for_event(); 
                break;
            }
        }
    }    
}

void interpret_command()
{
    char* token_ptr;
    const char * sep_tok = ", ";
    int nchar;
    uint8_t i, n;
    int16_t v;
    // printf("\rCommand text was: "); puts(cmd_buf);
    // printf("\r\nNumber of characters in buffer: %u", strlen(cmd_buf));
    switch (cmd_buf[0]) {
        case 'v':
            nchar = printf("%s ok", version_string);
            break;
        case 'a':
            arm_and_wait_for_event();
            break;
        case 'm':
            nchar = printf("wait for arm button on box ");
            wait_for_manual_arm();
            break;
        case 'n':
            nchar = printf("%u ok", NUMREG);
            break;
        case 'r':
            // Report a register value.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume register number.
                i = (uint8_t) atoi(token_ptr);
                if (i < NUMREG) {
                    v = registers[i];
                    nchar = printf("%d ok", v);
                } else {
                    nchar = printf("fail");
                }
            } else {
                nchar = printf("fail");
            }
            break;
        case 's':
            // Set a register value.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text; assume register number.
                // printf("text:\"%s\"", token_ptr);
                i = (uint8_t) atoi(token_ptr);
                if (i < NUMREG) {
                    token_ptr = strtok(NULL, sep_tok);
                    if (token_ptr) {
                        // Assume text is value for register.
                        v = (int16_t) atoi(token_ptr);
                        registers[i] = v;
                        nchar = printf("reg[%u] %d ok", i, v);
                        if (i == 1) { update_dac1(); }
                    } else {
                        nchar = printf("fail");
                    }
                } else {
                    nchar = printf("fail");
                }
            } else {
                nchar = printf("fail");
            }
            break;
        case 'R':
            if (restore_registers_from_HEF()) {
                nchar = printf("fail");
            } else {
                nchar = printf("ok");
            }
            break;
        case 'S':
            if (save_registers_to_HEF()) {
                nchar = printf("fail");
            } else {
                nchar = printf("ok");
            }
            break;
        case 'c':
            // Report an ADC value.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume channel number.
                i = (uint8_t) atoi(token_ptr);
                if (i <= 11) {
                    v = read_adc(i);
                    nchar = printf("%d ok", v);
                } else {
                    nchar = printf("fail");
                }
            } else {
                nchar = printf("fail");
            }
            break;
        default:
            nchar = printf("fail");
    } // end switch
} // end interpret_command())

int main(void)
{
    char* buf_ptr;
    int nchar;
    //
    // Initialize core, then peripherals.
    OSCCONbits.IRCF = 0b1110; // 8Mhz into 4xPLL to give FOSC=32MHz
    init_peripherals();
    //
    // Power-on values for registers.
    if (restore_registers_from_HEF()) {
        nchar = printf("Failed to restore registers from HEF.");
        registers[0] = 1; // mode
        registers[1] = 200; // trigger level 0-1023
        registers[2] = 1; // delay1 in counts
        registers[3] = 1; // delay2 in counts
    }
    update_dac1();
    //
    // Announce that the box is awake.
    nchar = printf("\r\nX2 trigger and timer, %s.", version_string);
    //
    // The basic behaviour is to be forever checking for a text command.
    nchar = printf("\r\ncmd> ");
    while (1) {
        // Characters are echoed as they are typed.
        // Backspace deleting is allowed.
        buf_ptr = gets(cmd_buf); 
        if (buf_ptr) {
            interpret_command();
            nchar = printf("\r\ncmd> ");
        }
    } // end while
    uart1_close();
    return 0;
} // end main
