#include <xc.h>
#include "delays.h"
#include "lcd.h"

#define DISTANCE 25.12

unsigned char outchar;
double TempLow, TempHigh, TimeElapsed, RealTimeElapsed;
int speed;

char Message1[] = "Speed in Km/Hr ";  // Define a 20 char string

void generateTone(void) {
    unsigned int k;

    for (k = 0; k < 100; k++) {
        delay_us(4000);
        PORTCbits.RC0 = !PORTCbits.RC0;
    }
}

void initializeIO() {
    // Set the directions of I/O pins
    TRISCbits.TRISC2 = 0;  // LED for distance warning
    TRISCbits.TRISC0 = 0;  // Buzzer for speed limit warning
    TRISAbits.TRISA0 = 1;  // Infrared Sensor
    TRISAbits.TRISA1 = 0;  // Ultrasonic Sensor trigger output
    TRISAbits.TRISA2 = 1;  // Ultrasonic Sensor trigger input
    TRISB = 0b00000000;    // Motor
}

void displayMessage(const char message[]) {
    lcd_write_cmd(0x80);
    for (int i = 0; message[i] != '\0'; i++) {
        outchar = message[i];
        lcd_write_data(outchar);
    }
}

void main(void) {
    initializeIO();
    int i;

    lcd_init();

    while (1) {
        displayMessage(Message1);
        PORTB = 0b00101011;
        delay_ms(300);

        // Infrared Sensor
        T0CON = 0b10000100;  // Timer on, 16-bit, Fosc/4, pre-scaler 32

        while (PORTAbits.RA0 == 0); // Wait for signal at RB0 to go high
        TMR0H = 0x00;
        TMR0L = 0x00;

        while (PORTAbits.RA0 == 1); // Wait for signal at RB0 to go low
        T0CONbits.TMR0ON = 0; // Stop timer

        // Time elapsed is now in TMR0H:TMR0L.
        TempLow = TMR0L;
        TempHigh = TMR0H;
        TimeElapsed = TempHigh * 256 + TempLow;
        RealTimeElapsed = TimeElapsed * 0.000002667;

        // Corrected speed calculation
        speed = (DISTANCE / RealTimeElapsed) * 0.036;

        lcd_write_data('T');

        // Display "S2" on the LCD
        lcd_write_cmd(0xC0);
        displayMessage("S2");

        delay_ms(1000);

        // Clear the LCD screen
        lcd_write_cmd(0x01);
    }
}
