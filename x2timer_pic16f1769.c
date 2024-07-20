// x2timer_pic16f1769.c
// X2/T6 trigger and timer box rebuilt around a PIC16F1769-I/P.
//
// Peter Jacobs (1) and Peter Collen (2)
// (1) School of Mechanical and Mining Engineering, University of Queensland.
// (2) Oxford Thermofluids Institute, University of Oxford
//
// 2018-01-30 start this firmware for PIC16F1778-I/SP.
// 2018-02-02 mode 1 complete with save/restore registers.
// 2019-02-19 port to PIC16F1769
// 2019-02-21 modes 3 and 4 available
// 2019-03-04 mode 5 measured-delay-using-hardware
// 2019-03-11 mode 6: mode 4, but accounts for desired distance from tube end.
// 2021-04-26 Refresh for UQ X2 use.
// 2021-05-25 Update delay calculation for X2 geometry.
// 2021-09-09 Add delay calculation for X3 expansion tube.
// 2024-07-20 Refresh for recent programming environment,
//            and implement 0-10V input mode for Aaron Cunningham.
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
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

const char * version_string = "Version 0.13 2024-07-20 PJ&PC";

// Some pin mappings; others are given in init_peripherals().
#define LED_ARM LATCbits.LATC6
#define LED_PWR LATCbits.LATC7

// Parameters controlling the device are stored in virtual registers.
#define NUMREG 4
int16_t vregister[NUMREG]; // working copy in SRAM
const char* hints[NUMREG] = { "mode", "level-a", "level-b", "delay" }; 

void set_registers_to_original_values()
{
    vregister[0] = 1;   // trigger mode
    vregister[1] = 5;   // trigger level 1 as a 10-bit count, 0-1023
    vregister[2] = 5;   // trigger level 2 as a 10-bit count, 0-1023
    vregister[3] = 0;   // delay as a 16-bit count
}

// High Endurance Flash block is used to hold the parameters
// when the power is off.
const char HEFdata[FLASH_ROWSIZE] __at(HEFLASH_START) = {
    1,0, 5,0, 5,0, 0,0,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
    0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff
};

char save_registers_to_HEF()
{
    return HEFLASH_writeBlock(0, (void*)vregister, NUMREG*2);
}

char restore_registers_from_HEF()
{
    return HEFLASH_readBlock((void*)vregister, 0, NUMREG*2);
}

// Text buffer for incoming commands.
// They are expected to be short.
// Be careful, overruns are not handled well.
#define NBUF 32
char cmd_buf[NBUF];

void init_peripherals()
{
    // Initializing the UART sets pins RA2,RC0,RB5,RB7.
    uart1_init(38400);
    //
    // Pins RB7,RC3,RC4,RC5,RC6,RC7 are digital-outputs.
    TRISBbits.TRISB7 = 0; ANSELBbits.ANSB7 = 0; LATBbits.LATB7 = 0;
    TRISCbits.TRISC3 = 0; ANSELCbits.ANSC3 = 0; LATCbits.LATC3 = 0;
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
    // RC1/AN5/CXIN1-, RC2/AN6/C12IN2-, RC3/AN7/OPA2OUT
    TRISCbits.TRISC1 = 1; ANSELCbits.ANSC1 = 1; WPUCbits.WPUC1 = 0;
    TRISCbits.TRISC2 = 1; ANSELCbits.ANSC2 = 1; WPUCbits.WPUC2 = 0;
    TRISCbits.TRISC3 = 1; ANSELCbits.ANSC3 = 1; WPUCbits.WPUC3 = 0;
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
    DAC1REF = (uint16_t)vregister[1];
    DACLDbits.DAC1LD = 1;
    //
    // Digital to analog converter 2
    DAC2CON0bits.PSS = 0b10; // FVR_buffer2
    DAC2CON0bits.NSS = 0; // Vss
    DAC2CON0bits.FM = 0; // right justified format
    DAC2CON0bits.EN = 1;
    DAC2REF = (uint16_t)vregister[2];
    DACLDbits.DAC2LD = 1;
    //
    // Digital to analog converter 3 to provide 2V output.
    DAC3CON0bits.PSS = 0b10; // FVR_buffer2
    DAC3CON0bits.NSS = 0; // Vss
    DAC3CON0bits.EN = 1;
    DAC3REF = 16; // half way to 32 for this 5-bit DAC
    // Buffer DAC3 with OPA2
    OPA2PCHS = 0b100; // DAC3_out
    OPA2CONbits.UG = 1; // Unity gain
    OPA2CONbits.EN = 1;
    //
    // Comparator 1
    CM1NSELbits.NCH = 0b001; // C1IN1-/RC1 pin
    CM1PSELbits.PCH = 0b1010; // DAC1_output
    CM1CON0bits.POL = 1; // invert output polarity
    CM1CON0bits.ON = 1;
    //
    // Comparator 2
    CM2NSELbits.NCH = 0b010; // C2IN2-/RC2 pin
    CM2PSELbits.PCH = 0b1011; // DAC2_output
    CM2CON0bits.POL = 1; // invert output polarity
    CM2CON0bits.ON = 1;
    //
    // Configurable logic cell 3 will be set up later,
    // in the trigger_using_hardware() function.
    //
    // Flash the LEDs a moment and then settle to power-on.
    for (int8_t i=0; i < 2; ++i ) {
        LED_ARM = 1; LED_PWR = 1; __delay_ms(500);
        LED_ARM = 0; LED_PWR = 0; __delay_ms(500);
    }
    LED_PWR = 1;
}

void update_DACs(int16_t val1, int16_t val2)
{
    DAC1REF = (uint16_t)val1;
    DACLDbits.DAC1LD = 1;
    DAC2REF = (uint16_t)val2;
    DACLDbits.DAC2LD = 1;
}

uint16_t read_adc(uint8_t chan)
{
    ADCON0bits.CHS = chan;
    __delay_ms(10);
    ADCON0bits.GO = 1;
    NOP();
    while (ADCON0bits.GO) { NOP(); }
    return ADRES;
}

void trigger_simple_firmware(void)
{
    // Firmware-controlled trigger, both outputs immediate.
    // Use MCU to monitor and control the state of bits.
    //
    // Leave RC4 and RC5 controlled by their latch.
    LATC &= 0b11001111;
    LED_ARM = 1;
    // Wait for the comparator to go high.
    while (!CMOUTbits.MC1OUT) { CLRWDT(); }
    LATC |= 0b00110000;
    __delay_ms(500);
    LATC &= 0b11001111;
    LED_ARM = 0;
} // end trigger_simple_firmware()

void trigger_simple_hardware(void)
{
    // Hardware-only trigger, both outputs immediate.
    // Set up CLC3 to extend the Comparator pulse indefinitely.
    // This should be the fastest response to an event.
    //
    CLC3CONbits.EN = 0;
    CLC3CONbits.MODE = 0b100; // D flip-flop
    CLC3POLbits.POL = 0; // not inverted
    CLC3SEL0bits.D1S = 0b000111; // sync_C1OUT as data1 for 
    CLC3SEL1bits.D2S = 0; // none (CLCIN0 via PPS, but is ignored)
    CLC3SEL2bits.D3S = 0; // none
    CLC3SEL3bits.D4S = 0; // none
    // Steer the output to the MCU pins.
    GIE = 0; PPSLOCK = 0x55; PPSLOCK = 0xaa; PPSLOCKED = 0;
    RC4PPS = 0b00011; // LC3_out
    RC5PPS = 0b00011; // LC3_out also
    PPSLOCK = 0x55; PPSLOCK = 0xaa; PPSLOCKED = 1;
    //
    CLC3GLS0 = 0b10; // gate 1, send data1 through true
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
} // end trigger_simple_hardware()

void trigger_delayed_firmware(void)
{
    // Trigger from comparator 1.
    // Output 1 immediate, output 2 fixed delay.
    //
    uint16_t cmp_count = (uint16_t) vregister[3];
    int nchar;
    //
    // Leave RC4 and RC5 controlled by their latch.
    LATC &= 0b11001111;
    // Set up Timer1, driven by 8MHz (FOSC/4) instruction clock.
    T1CONbits.ON = 0;
    T1CONbits.CS = 0b00;
    T1CONbits.CKPS = 0b00; // Prescale of 1.
    T1GCONbits.GE = 0; // Timer is always counting, once on.
    TMR1 = 0;
    PIR1bits.TMR1IF = 0;
    // Set up Compare module, looking at Timer1.
    CCP1CONbits.MODE = 0b1000; // Compare mode, set output on match.
    CCPR1 = cmp_count;
    PIR1bits.CCP1IF = 0;
    LED_ARM = 1;
    // Wait for comparator 1 to go high.
    while (!CMOUTbits.MC1OUT) { CLRWDT(); }
    LATC |= 0b00100000; // RC5 is immediate output.
    CCP1CONbits.EN = 1;
    T1CONbits.ON = 1;
    if (cmp_count == 0) {
        // We have no delay; proceed to signal immediately.
    } else {
        // Wait for compare match with timer.
        while (!CCP1CONbits.OUT) { CLRWDT(); }
    }
    // We have waited the appointed time.
    LATC |= 0b00010000; // RC4 is delayed output.
    //
    // Our work is done, so we now clean up at a leisurely pace.
    T1CONbits.ON = 0;
    CCP1CONbits.EN = 0;
    CCP1CONbits.MODE = 0;
    __delay_ms(500);
    LATC &= 0b11001111;
    LED_ARM = 0;    
} // end trigger_delayed_firmware()

void trigger_measured_delay(void)
{
    // Start timing on comparator 1.
    // Output 1 immediate.
    // Stop timing on comparator 2.
    // Output 2 after computed delay plus fixed delay.
    // Use MCU to monitor and control the state of bits.
    //
    uint16_t extra_delay = (uint16_t) vregister[3];
    uint16_t tmr_count, cmp_count;
    int nchar;
    //
    // Leave RC4 and RC5 controlled by their latch.
    LATC &= 0b11001111;
    // Set up Timer1, driven by 8MHz (FOSC/4) instruction clock.
    T1CONbits.ON = 0;
    T1CONbits.CS = 0b00;
    T1CONbits.CKPS = 0b00; // Prescale of 1.
    T1GCONbits.GE = 0; // Timer is always counting, once on.
    TMR1 = 0;
    PIR1bits.TMR1IF = 0;
    // Set up Compare module, looking at Timer1.
    CCP1CONbits.MODE = 0b1000; // Compare mode, set output on match.
    PIR1bits.CCP1IF = 0;
    LED_ARM = 1;
    // Wait for comparator 1 to go high, event A.
    while (!CMOUTbits.MC1OUT) { CLRWDT(); }
    T1CONbits.ON = 1;
    LATC |= 0b00100000; // RC5 is immediate output.
    // With timer counting, wait for event B.
    while (!CMOUTbits.MC2OUT) { CLRWDT(); }
    tmr_count = TMR1;
    // Leave the timer counting and set up the compare value,
    // assuming that the time to go is roughly equal to the time
    // between events A and B.
    cmp_count = tmr_count * 2 + extra_delay;
    // [TODO] Check that we don't overflow.
    // This should not be a real problem for the cases that interest us.
    // A period of 50us between events A, B should give a first count of 400.
    // Doubling that and adding a bit should be OK.
    CCPR1 = cmp_count;
    CCP1CONbits.EN = 1;
    while (!CCP1CONbits.OUT) { CLRWDT(); }
    // We have waited the appointed time.
    LATC |= 0b00010000; // RC4 is delayed output.
    //
    // Our work is done, so we now clean up at a leisurely pace.
    T1CONbits.ON = 0;
    CCP1CONbits.EN = 0;
    CCP1CONbits.MODE = 0;
    __delay_ms(500);
    LATC &= 0b11001111;
    LED_ARM = 0;
    nchar = printf("\ntmr_count=%d cmp_count=%d\n", tmr_count, cmp_count);
} // end trigger_measured_delay()

void measured_delay_using_hardware(void)
{
    // Start timing on comparator 1.
    // Output 1 immediate, to indicate gating of Timer3.
    // Stop Timer3 on comparator 2 and, simultaneously, start Timer1.
    // Output 2 (CCP1_out) after measured time plus extra delay on Timer1.
    //
    // Use core-independent peripheral devices to gate Timers
    // so that we do not have the latencies associated with 
    // MCU while loops.
    //
    uint16_t extra_delay = (uint16_t) vregister[3];
    uint16_t tmr_count, cmp_count;
    int nchar;
    //
    // Outputs RC3, RC4 and RC5 will be controlled by peripheral devices
    // for the main event but, before and after that, set to zero.
    LATC &= 0b11000111;
    //
    GIE = 0; PPSLOCK = 0x55; PPSLOCK = 0xaa; PPSLOCKED = 0;
    // Timers will be gated by pin signals.
    T1GPPS = 0b10011; // RC3
    T3GPPS = 0b10101; // RC5
    // Steer the output signals to the MCU pins.
    RC3PPS = 0b00010; // LC2_out
    RC4PPS = 0b01100; // CPP1_out
    RC5PPS = 0b00011; // LC3_out
    PPSLOCK = 0x55; PPSLOCK = 0xaa; PPSLOCKED = 1;
    //
    // Set up CLC2 to extend the Comparator 2 pulse.
    // This should be the fastest response to an event.
    CLC2CONbits.EN = 0;
    CLC2CONbits.MODE = 0b100; // D flip-flop
    CLC2POLbits.POL = 0; // not inverted
    CLC2SEL0bits.D1S = 0b001000; // sync_C2OUT as data1 for clock
    CLC2SEL1bits.D2S = 0; // none (CLCIN0 via PPS, but is ignored)
    CLC2SEL2bits.D3S = 0; // none
    CLC2SEL3bits.D4S = 0; // none
    CLC2GLS0 = 0b00000010; // gate 1, send data1-true through
    CLC2GLS1 = 0; // gate 2, logic 0
    CLC2GLS2 = 0; // gate 3, logic 0
    CLC2GLS3 = 0; // gate 4, logic 0
    CLC2POLbits.G1POL = 0; 
    CLC2POLbits.G2POL = 1; // invert logic 0 to get logic 1
    CLC2POLbits.G3POL = 0;
    CLC2POLbits.G4POL = 0;
    //
    // Set up CLC3 to extend the Comparator 1 pulse,
    // until CLC2 resets it.
    CLC3CONbits.EN = 0;
    CLC3CONbits.MODE = 0b100; // D flip-flop
    CLC3POLbits.POL = 0; // not inverted
    CLC3SEL0bits.D1S = 0b000111; // sync_C1OUT as data1 for clock
    CLC3SEL1bits.D2S = 0; // none (CLCIN0 via PPS, but is ignored)
    CLC3SEL2bits.D3S = 0b000101; // LC2_out from CLC2 as data3 for reset
    CLC3SEL3bits.D4S = 0; // none
    CLC3GLS0 = 0b00000010; // gate 1, send data1-true through
    CLC3GLS1 = 0; // gate 2, logic 0
    CLC3GLS2 = 0b00100000; // gate 3, send data3-true through
    CLC3GLS3 = 0; // gate 4, logic 0
    CLC3POLbits.G1POL = 0; 
    CLC3POLbits.G2POL = 1; // invert logic 0 to get logic 1
    CLC3POLbits.G3POL = 0;
    CLC3POLbits.G4POL = 0;
    //
    CLC2CONbits.EN = 1;
    CLC3CONbits.EN = 1;
    //
    // Set up Timer3 to record time of flight between comparators.
    T3CONbits.ON = 0;
    T3CONbits.CS = 0b00; // driven by 8MHz (FOSC/4) instruction clock.
    T3CONbits.CKPS = 0b00; // Prescale of 1.
    T3GCONbits.GE = 1; // Timer3 is gated by hardware.
    T3GCONbits.GPOL = 1; // counts while gate is high
    T3GCONbits.GTM = 0; // disable gate toggle mode
    T3GCONbits.GSPM = 0; // disable single-pulse mode
    T3GCONbits.GSS = 0b00; // look to gate pin
    TMR3 = 0;
    PIR4bits.TMR3IF = 0;
    T3CONbits.ON = 1;
    // Timer3 should now be ready to count ticks 
    // between the comparator events.
    //
    // Timer1 will set the waiting period by starting to count
    // once LC2_out goes high.
    T1CONbits.ON = 0;
    T1CONbits.CS = 0b00;
    T1CONbits.CKPS = 0b00; // Prescale of 1.
    T1GCONbits.GE = 1; // Timer1 is gated by hardware.
    T1GCONbits.GPOL = 1; // counts while gate is high
    T1GCONbits.GTM = 0; // disable gate toggle mode
    T1GCONbits.GSPM = 0; // disable single-pulse mode
    T1GCONbits.GSS = 0b00; // look to gate pin
    TMR1 = 0;
    PIR1bits.TMR1IF = 0;
    T1CONbits.ON = 1;
    //
    // Set up Compare module, looking at Timer1.
    CCP1CONbits.MODE = 0b1000; // Compare mode, set output on match.
    PIR1bits.CCP1IF = 0;
    //
    LED_ARM = 1;
    // At this point, the hardware is all set to monitor
    // the comparator signals, not much else to do but wait
    // for the time of shock to be available .
    //
    // Wait for comparator 2 (stretched by LC2_out) to go high, event B.
    while (!CLC2CONbits.OUT) { CLRWDT(); }
    tmr_count = TMR3;
    // LC2_out going high should have started Timer1 counting.
    // We should set up the compare value, assuming that 
    // the time to go is roughly equal to the time
    // between events A and B.
    cmp_count = tmr_count + extra_delay;
    // [TODO] Check that we don't overflow.
    // This should not be a real problem for the cases that interest us.
    // A period of 50us between events A, B should give a count of 400.
    CCPR1 = cmp_count;
    CCP1CONbits.EN = 1;
    while (!CCP1CONbits.OUT) { CLRWDT(); }
    //
    // At this point CCP1_out will have appeared on RC4.
    // Our work is done, so we now clean up at a leisurely pace.
    while (!CCP1CONbits.OUT) { CLRWDT(); }
    __delay_ms(500);
    // End of output pulse; clean up by steering
    // the original data latch values to pins.
    GIE = 0; PPSLOCK = 0x55; PPSLOCK = 0xaa; PPSLOCKED = 0;
    RC3PPS = 0; // Return control to LATC3
    RC4PPS = 0; // Return control to LATC4
    RC5PPS = 0; // Return control to LATC5
    PPSLOCK = 0x55; PPSLOCK = 0xaa; PPSLOCKED = 1;
    CLC3CONbits.EN = 0; // Disable flip-flop
    CLC2CONbits.EN = 0; // Disable flip-flop
    T3CONbits.ON = 0;
    T1CONbits.ON = 0;
    CCP1CONbits.EN = 0;
    CCP1CONbits.MODE = 0;
    LATC &= 0b11000111;
    LED_ARM = 0;
    nchar = printf("\ntmr_count=%d cmp_count=%d\n", tmr_count, cmp_count);
} // end measured_delay_using_hardware()


void trigger_measured_extra_delay_oxford(void)
{
    // Start timing on comparator 1.
    // Output 1 immediate.
    // Stop timing on comparator 2.
    // Output 2 after computed delay,
    // based on geometry of Oxford tube/optical system.
    // Use MCU to monitor and control the state of bits.
    //
    uint16_t tmr_count, cmp_count;
    int nchar;
    
    // Leave RC4 and RC5 controlled by their latch.
    LATC &= 0b11001111;
    // Set up Timer1, driven by 8MHz (FOSC/4) instruction clock.
    T1CONbits.ON = 0;
    T1CONbits.CS = 0b00;
    T1CONbits.CKPS = 0b00; // Prescale of 1.
    T1GCONbits.GE = 0; // Timer is always counting, once on.
    TMR1 = 0;
    PIR1bits.TMR1IF = 0;
    // Set up Compare module, looking at Timer1.
    CCP1CONbits.MODE = 0b1000; // Compare mode, set output on match.
    PIR1bits.CCP1IF = 0;
    LED_ARM = 1;
    // Wait for comparator 1 to go high, event A.
    while (!CMOUTbits.MC1OUT) { CLRWDT(); }
    T1CONbits.ON = 1;
    LATC |= 0b00100000; // RC5 is immediate output.
    // With timer counting, wait for event B.
    while (!CMOUTbits.MC2OUT) { CLRWDT(); }
    tmr_count = TMR1;
    // Leave the timer counting and set up the compare value,
    // assuming that centre of the optics are at about 10% of the sensor separation.
    cmp_count = (tmr_count << 1) + (tmr_count >> 3);
    
    // [TODO] Check that we don't overflow.
    // This should not be a real problem for the cases that interest us.
    // A period of 50us between events A, B should give a first count of 400.
    // Doubling that and adding a bit should be OK.
    CCPR1 = cmp_count;
    CCP1CONbits.EN = 1;
    while (!CCP1CONbits.OUT) { CLRWDT(); }
    // We have waited the appointed time.
    LATC |= 0b00010000; // RC4 is delayed output.
    //
    // Our work is done, so we now clean up at a leisurely pace.
    T1CONbits.ON = 0;
    CCP1CONbits.EN = 0;
    CCP1CONbits.MODE = 0;
    __delay_ms(500);
    LATC &= 0b11001111;
    LED_ARM = 0;
    
    nchar = printf("\ntmr_count=%d cmp_count=%d\n", tmr_count, cmp_count);
} // end trigger_measured_extra_delay_oxford()


void trigger_measured_extra_delay_x2x3(void)
{
    // Modes 7 (X2), 8 (X3).
    // Start timing on comparator 1.
    // Output 1 immediate.
    // Stop timing on comparator 2.
    // Output 2 after computed delay, based on geometry of X2 tube and nozzle.
    // Measure AT4-AT7 (0.568m) test-model 1.863m after AT7.
    // We want final count to be 4.28 times measured count but we settle for
    // 4.25 (4+1/4) as the scale factor.
    // Use MCU to monitor and control the state of bits.
    //
    uint16_t operating_mode = (uint16_t) vregister[0];
    uint16_t tmr_count, cmp_count;
    uint16_t extra_delay = (uint16_t) vregister[3];
    int nchar;
    
    // Leave RC4 and RC5 controlled by their latch.
    LATC &= 0b11001111;
    // Set up Timer1, driven by 8MHz (FOSC/4) instruction clock.
    T1CONbits.ON = 0;
    T1CONbits.CS = 0b00;
    T1CONbits.CKPS = 0b00; // Prescale of 1.
    T1GCONbits.GE = 0; // Timer is always counting, once on.
    TMR1 = 0;
    PIR1bits.TMR1IF = 0;
    // Set up Compare module, looking at Timer1.
    CCP1CONbits.MODE = 0b1000; // Compare mode, set output on match.
    PIR1bits.CCP1IF = 0;
    LED_ARM = 1;
    // Wait for comparator 1 to go high, event A.
    while (!CMOUTbits.MC1OUT) { CLRWDT(); }
    T1CONbits.ON = 1;
    LATC |= 0b00100000; // RC5 is immediate output (outA).
    LED_ARM = 0; // Indicate that the event A has happened.
    // With timer counting, wait for event B.
    while (!CMOUTbits.MC2OUT) { CLRWDT(); }
    tmr_count = TMR1;
    // Leave the timer counting and set up the compare value.
    if (operating_mode == 7) {
        // For X2 AT4-AT7, set to be a factor of 4.25 time the measured value.
        cmp_count = (tmr_count << 2) + (tmr_count >> 2) + extra_delay;
    } else {
        // For X3 AT6-AT7, set to be 3.4375 (2+1+1/4+1/8+1/16).
        // At 8km/s shock speed, we have about 680 microseconds to do
        // the calculation.
        cmp_count = (tmr_count << 1) + tmr_count + (tmr_count >> 2) +
                (tmr_count >> 3) + (tmr_count >> 4) + extra_delay;
    }
    // [TODO] Check that we don't overflow.
    // This should not be a real problem for the cases that interests us.
    // In X2, at 9km/s, we expect a period of 63.1us between events A, B.
    // This should give a first count of 505.
    // Scaling that by 4.25 and adding a bit should be OK for a 16-bit int.
    // In X3, at 8km/s, we expect first count of 1600 and a final count of 5500.
    CCPR1 = cmp_count;
    CCP1CONbits.EN = 1;
    while (!CCP1CONbits.OUT) { CLRWDT(); }
    // We have waited the appointed time.
    LATC |= 0b00010000; // RC4 is delayed output (outB).
    //
    // Our work is done, so we now clean up at a leisurely pace.
    T1CONbits.ON = 0;
    CCP1CONbits.EN = 0;
    CCP1CONbits.MODE = 0;
    __delay_ms(500);
    LATC &= 0b11001111;
    
    nchar = printf("\ntmr_count=%d cmp_count=%d extra_delay=%d\n",
                   tmr_count, cmp_count, extra_delay);
} // end trigger_measured_extra_delay_x2x3()


void arm_and_wait_for_event(void)
{
    int nchar;
    uint16_t val1, val2;
    switch (vregister[0]) {
        case 0:
            nchar = printf("armed mode 0, differential trigger levels: ");
            val1 = read_adc(5); val2 = read_adc(6);
            val1 += read_adc(5); val2 += read_adc(6);
            val1 += read_adc(5); val2 += read_adc(6);
            val1 += read_adc(5); val2 += read_adc(6);
            val1 /= 4; val2 /= 4;
            nchar = printf("initial INa INb= %u %u ", val1, val2);
            update_DACs(vregister[1]+val1, vregister[2]+val2);
            __delay_ms(1); // Allow DACs to settle.
            if (CMOUTbits.MC1OUT) {
                nchar = printf("C1OUT already high. fail\n");
                break;
            }
            trigger_simple_hardware();
            nchar = printf("triggered. ok\n");
            break;
        case 1:
            nchar = printf("armed mode 1, simple firmware, both outputs immediate. ");
            if (CMOUTbits.MC1OUT) {
                nchar = printf("C1OUT already high. fail\n");
                break;
            }
            trigger_simple_firmware();
            nchar = printf("triggered. ok\n");
            break;
        case 2:
            nchar = printf("armed mode 2, simple hardware, both outputs immediate. ");
            if (CMOUTbits.MC1OUT) {
                nchar = printf("C1OUT already high. fail\n");
                break;
            }
            trigger_simple_hardware();
            nchar = printf("triggered. ok\n");
            break;
        case 3:
            nchar = printf("armed mode 3, one output immediate, one fixed delay. ");
            if (CMOUTbits.MC1OUT) {
                nchar = printf("C1OUT already high. fail\n");
                break;
            }
            trigger_delayed_firmware();
            nchar = printf("triggered. ok\n");
            break;
        case 4:
            nchar = printf("armed mode 4, one output immediate, one measured delay. ");
            if (CMOUTbits.MC1OUT) {
                printf("C1OUT already high. fail\n");
                break;
            }
            if (CMOUTbits.MC2OUT) {
                printf("C2OUT already high. fail\n");
                break;
            }
            trigger_measured_delay();
            nchar = printf("triggered. ok\n");
            break;
        case 5:
            nchar = printf("armed mode 5, measured delay using hardware. ");
            if (CMOUTbits.MC1OUT) {
                printf("C1OUT already high. fail\n");
                break;
            }
            if (CMOUTbits.MC2OUT) {
                printf("C2OUT already high. fail\n");
                break;
            }
            measured_delay_using_hardware();
            nchar = printf("triggered. ok\n");
            break;
		 case 6:
            nchar = printf("armed mode 6, one output immediate, one measured (Oxford optics). ");
            if (CMOUTbits.MC1OUT) {
                printf("C1OUT already high. fail\n");
                break;
            }
            if (CMOUTbits.MC2OUT) {
                printf("C2OUT already high. fail\n");
                break;
            }
            trigger_measured_extra_delay_oxford();
            nchar = printf("triggered. ok\n");
            break;
		 case 7:
            nchar = printf("armed mode 7, one output immediate, one measured (X2). ");
            if (CMOUTbits.MC1OUT) {
                printf("C1OUT already high. fail\n");
                break;
            }
            if (CMOUTbits.MC2OUT) {
                printf("C2OUT already high. fail\n");
                break;
            }
            trigger_measured_extra_delay_x2x3();
            nchar = printf("triggered. ok\n");
            break;
		 case 8:
            nchar = printf("armed mode 8, one output immediate, one measured (X3). ");
            if (CMOUTbits.MC1OUT) {
                printf("C1OUT already high. fail\n");
                break;
            }
            if (CMOUTbits.MC2OUT) {
                printf("C2OUT already high. fail\n");
                break;
            }
            trigger_measured_extra_delay_x2x3();
            nchar = printf("triggered. ok\n");
            break;
        default:
            nchar = printf("unknown mode. fail\n");
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
    // printf("Command text was: "\n); putstr(cmd_buf);
    // printf("Number of characters in buffer: %u\n", strlen(cmd_buf));
    switch (cmd_buf[0]) {
        case 'v':
            nchar = printf("%s ok\n", version_string);
            break;
        case 'n':
            nchar = printf("%u ok\n", NUMREG);
            break;
        case 'p':
            nchar = printf("Register values:\n");
            for (i=0; i < NUMREG; ++i) {
                nchar = printf("reg[%d]=%d (%s)\n", i, vregister[i], hints[i]);
            }
            nchar = printf("ok\n");
            break;
        case 'r':
            // Report a register value.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume register number.
                i = (uint8_t) atoi(token_ptr);
                if (i < NUMREG) {
                    v = vregister[i];
                    nchar = printf("%d (%s)ok\n", v, hints[i]);
                } else {
                    nchar = printf("fail");
                }
            } else {
                nchar = printf("fail\n");
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
                        vregister[i] = v;
                        nchar = printf("reg[%u] %d (%s) ok\n", i, v, hints[i]);
                        if (i == 1 || i == 2) {
                            update_DACs(vregister[1], vregister[2]);
                        }
                    } else {
                        nchar = printf("fail\n");
                    }
                } else {
                    nchar = printf("fail\n");
                }
            } else {
                nchar = printf("fail\n");
            }
            break;
        case 'R':
            if (restore_registers_from_HEF()) {
                nchar = printf("fail\n");
            } else {
                nchar = printf("ok\n");
            }
            break;
        case 'S':
            if (save_registers_to_HEF()) {
                nchar = printf("fail\n");
            } else {
                nchar = printf("ok\n");
            }
            break;
        case 'F':
            set_registers_to_original_values();
            nchar = printf("ok\n");
            break;
        case 'a':
            arm_and_wait_for_event();
            break;
        case 'm':
            nchar = printf("wait for arm button on box ");
            wait_for_manual_arm();
            break;
        case 'c':
            // Report an ADC value.
            token_ptr = strtok(&cmd_buf[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume channel number.
                i = (uint8_t) atoi(token_ptr);
                if (i <= 31) {
                    v = (int16_t)read_adc(i);
                    nchar = printf("%d ok\n", v);
                } else {
                    nchar = printf("fail\n");
                }
            } else {
                nchar = printf("fail\n");
            }
            break;
        case 'h':
        case '?':
            nchar = printf("PIC16F1769-I/P X2-X3-trigger+timer commands and registers\n");
            nchar = printf("\n");
            nchar = printf("Commands:\n");
            nchar = printf(" h or ? print this help message\n");
            nchar = printf(" v      report version of firmware\n");
            nchar = printf(" n      report number of registers\n");
            nchar = printf(" p      report register values\n");
            nchar = printf(" r <i>  report value of register i\n");
            nchar = printf(" s <i> <j>  set register i to value j\n");
            nchar = printf(" R      restore register values from HEFlash\n");
            nchar = printf(" S      save register values to HEFlash\n");
            nchar = printf(" F      set register values to original values\n");
            nchar = printf(" a      arm device and wait for event\n");
            nchar = printf(" m      wait for manual arm button press\n");
            nchar = printf(" c <i>  convert analogue channel i\n");
            nchar = printf("        i=30 DAC1_output\n");
            nchar = printf("        i=28 DAC2_output\n");
            nchar = printf("        i=5  RC1/AN5/C1IN1- (IN a)\n");
            nchar = printf("        i=6  RC2/AN6/C2IN2- (IN b)\n");
            nchar = printf("        i=7  RC3/AN7/OPA2OUT (V2V)\n");
            nchar = printf("\n");
            nchar = printf("Registers:\n");
            nchar = printf(" 0  mode: 0= simple trigger with differential levels\n");
            nchar = printf("          1= simple trigger (firmware), both outputs immediate\n");
            nchar = printf("          2= simple trigger (hardware), both outputs immediate\n");
            nchar = printf("          3= delayed trigger, output a immediate, b delayed\n");
            nchar = printf("          4= measured delay (firmware), output a immediate, b delayed\n");
            nchar = printf("          5= measured delay (hardware), output a immediate, b delayed\n");
            nchar = printf("          6= measured delay for Oxford tunnel, output a immediate, b delayed\n");
            nchar = printf("          7= measured delay for X2 AT4-AT7, output a AT4, b test-section\n");
            nchar = printf("          8= measured delay for X3 AT6-AT7, output a AT6, b test-section\n");
            nchar = printf(" 1  trigger level a as a 10-bit count, 0-1023\n");
            nchar = printf(" 2  trigger level b as a 10-bit count, 0-1023\n");
            nchar = printf(" 3  (extra) delay as 16-bit count (8 ticks per us)\n");
            nchar = printf("ok\n");
            break;
        default:
            nchar = printf("fail\n");
    } // end switch
} // end interpret_command())

int main(void)
{
    int nchar;
    //
    // Initialize core, then peripherals.
    OSCCONbits.IRCF = 0b1110; // 8Mhz into 4xPLL to give FOSC=32MHz
    init_peripherals();
    //
    // Power-on values for registers.
    if (restore_registers_from_HEF()) {
        nchar = printf("Failed to restore registers from HEF.\n");
        set_registers_to_original_values();
    }
    update_DACs(vregister[1], vregister[2]);
    //
    // Announce that the box is awake.
    nchar = printf("X2-X3 shock-speed trigger and timer, %s.\n", version_string);
    //
    // The basic behaviour is to be forever checking for a text command.
    nchar = printf("cmd> ");
    while (1) {
        // Characters are echoed as they are typed.
        // Backspace deleting is allowed.
        nchar = getstr(cmd_buf, NBUF); 
        if (nchar > 0) {
            interpret_command();
            nchar = printf("cmd> ");
        }
    } // end while
    uart1_close();
    return 0;
} // end main
