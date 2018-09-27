// Test ADC on PIC18LF25K22

#define _XTAL_FREQ 4000000
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
#pragma config WDTEN = ON       // Watchdog Timer Enable bits (WDT is always enabled. SWDTEN bit has no effect)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
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

#include <xc.h>

void main()
{
    // Configure PORTB and PORTC as ADC outputs to LEDs
    TRISB = 0b00000000; // Initialized all register B ports as outputs.
                        // All ports are outputs by default.
    TRISC = 0b00000000; // Initialized all register C ports as outputs.
                        // All ports are outputs by default.
    
    /* Port Configuration */
    TRISA = 0b00000001; // Initialize port A as outputs except AN0 as input
    ANSELA = 0b11111111; // Disable digital input buffer at Port A
   
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
    
    while(1)
    {
        __delay_ms(10); // software delay = 10msec
        GO = 1; // start conversion
        while(GO == 1)
        {
            // wait for conversion to complete (i.e ADCON0.GO=0)
        }
        PORTC = ADRESL; // send ADRESL<7:0> to PORTCbit<7:0>
        PORTB = ADRESH; // ADRESH1 to RB1, ADRESH0 to RB0
    }
}

