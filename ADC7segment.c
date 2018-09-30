
// PIC18LF25K22 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTB5   // ECCP2 B output mux bit (P2B is on RB5)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define _XTAL_FREQ 4000000  // clock frequency
#include <xc.h>
#include <pic18lf25k22.h>


unsigned int Hex(int digit)
{
    switch(digit)
    {
        case 1: return 0xF9; // common anode //0x06; //common cathode
        case 2: return 0xA4; // 0x5B; //
        case 3: return 0xB0; // 0x4F;
        case 4: return 0x99; // 0x66;
        case 5: return 0x92; // 0x6D;
        case 6: return 0x82; // 0x7D;
        case 7: return 0xF8; // 0x07;
        case 8: return 0x80; // 0x7F;
        case 9: return 0x90; // 0x6F;
        case 0: return 0xC0; // common anode // 0x3F; // common cathode
    }
    return 0;
}

int main(void)
{
    TRISC = 0x00; // all PORTC as outputs
    PORTC = 0; // clear all PORTC outputs
    TRISAbits.RA5 = 0; // configure RA0 as output
    TRISAbits.RA4 = 0; // configure RA1 as output
    TRISAbits.RA3 = 0; // configure RA2 as output
    //TRISBbits.RB0 = 1; // configure RB0 as input
    PORTAbits.RA5 = 0; // set RA0 to 0 to turn off 7-segment display
    PORTAbits.RA4 = 0; // set RA1 to 0 to turn off 7-segment display
    PORTAbits.RA3 = 0; // set RA2 to 0 to turn off 7-segment display
    ANSELA = 0x00; // set PORTA<5:3> to analog output
    int i0 = 0; // initialize counter i0 to 0
    int i1 = 0; // initialize counter i1 to 0
    int i2 = 0; // initialize counter i2 to 0
    int count = 33;
    
    // ADC configuration //
    /* Port Configuration */
    TRISAbits.RA0 = 1; // Configure AN0 as input
    ANSELAbits.ANSA0 = 1; // Disable digital input buffer at RA0
   
    // ADC conversion clock source
    //ADCON2.ACQT = 0b000; // default: manual acquisition
    //ADCON2.ADCS = 0b110; // select conversion clock FOSC/64 for FOSC=4MHz
    // Results formatting
    //ADCON2.ADFM = 1; // ADC result right justified in ADRESH:ADRESL registers
    ADCON2 = 0b10000110;
    
    /* ADC Voltage reference selection */
    //ADCON1.PVCFG = 0b00; // select VDD as positive voltage reference
    //ADCON1.NVCFG = 0b00; // select VSS as negative voltage reference
    ADCON1 = 0b0000000;
    
     /* Channel Selection */
    //ADCON0.CHS = 0b00000; // select AN0 as analog input channel to ADC
    ADCON0 = 0b00000000;
    
    // Enable the ADC module
    ADON = 1; // turn ON ADC module
    // Interrupt control (optional)
    //PIR1.ADIF = 0; // ADC interrupt flag bit cleared 
    //PIE1.ADIE = 0; // ADC Interrupt disabled
    //IPR1.ADIP = 0; // ADC interrupt priority 
    // enable/disable global interrupt ?
    
    /* -------------------  */
    unsigned char Binary7to0 = 0b00000000;
    unsigned char Binary15to8 = 0b00000000;
    unsigned short Binary2bytes = 0;
    double Vout = 0.0; // temperature sensor MCP9700 output voltage in V
    double V0 = 0.5; // output voltage in V of temperature sensor at 0 deg C
    double Tc = 0.01; // temperature coefficient of temperature sensor MCP9700
                      // in V/deg C.
    double Ta = 0.0; // ambient temperature
    unsigned char T;
    
    while(1)
    {
        for(int i=0; i<count; i++)
        {
            PORTAbits.RA5 = 1; // turn on digit 0 7-segment display
            PORTAbits.RA4 = 0; // turn off digit 1 7-segment display
            PORTAbits.RA3 = 0; // turn off digit 2 7-segment display
            PORTC = Hex(i0);
            __delay_ms(10); // delay for 25ms=0.010second
            
            PORTAbits.RA5 = 0; // turn off digit 0 7-segment display
            PORTAbits.RA4 = 1; // turn on digit 1 7-segment display
            PORTAbits.RA3 = 0; // turn off digit 2 7-segment display
            PORTC = Hex(i1);
            __delay_ms(10); // delay for 10ms=0.010second
        
            PORTAbits.RA5 = 0; // turn off digit 0 7-segment display
            PORTAbits.RA4 = 0; // turn off digit 1 7-segment display
            PORTAbits.RA3 = 1; // turn on digit 2 7-segment display
            PORTC = Hex(i2);
            __delay_ms(10); // delay for 25ms=0.010second
        }
        /*
        i0+=1; // increment integer
        if(i0==10) // if i is 10
        {
            i0=0; //reset i0 to 0
            i1+=1; // increment i1
            if(i1==10)
            {
                i1=0; // reset i1 to 0
                i2+=1; // increment i2
                if(i2==10)
                {
                    i2=0; // reset i2 to 0
                }
            }
        }*/
        __delay_ms(10); // software delay = 10msec
        GO = 1; // start conversion
        while(GO == 1)
        {
            // wait for conversion to complete (i.e ADCON0.GO=0)
        }
        /*PORTC = ADRESL; // send ADRESL<7:0> to PORTCbit<7:0>
        PORTB = ADRESH; // ADRESH1 to RB1, ADRESH0 to RB0
         */
        // 10-bit Binary to decimal conversion
        Binary2bytes = 0; // clear all bits
        Binary2bytes = ADRESL;       
        Binary15to8 = ADRESH;
        if((Binary15to8) & 0b00000001)
        {
            Binary2bytes += 256;
        }
        if((Binary15to8>>=1) & 0b00000001)
        {
            Binary2bytes += 512;
        }
        Vout = Binary2bytes/1024.0*3.3;
        Ta = (Vout-V0)/Tc;
        //T = static_cast<short>(Ta);
        T = Ta/1;
        i0 = T % 10;
        T = T / 10;
        i1 = T % 10;
        T = T / 10;
        i2 = T % 10;
        
    }
    return 0;
}





