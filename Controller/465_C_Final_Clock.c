#include <msp430.h>
#include <stdint.h>
#include "font.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////     Definitions     ////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//---------------- OLED Definitions ----------------
// Extraordinarily convoluted, I based this off the Adafruit library.

#define OLED_ADDR                   0x3C
#define OLED_CMD                    0x80  // Control byte for commands
#define OLED_DATA                   0x40  // Control byte for data

#define OLED_LCDWIDTH               128   // In pixels (Screen is 128 x 32)
#define OLED_LCDHEIGHT              32

#define OLED_SETCONTRAST            0x81
#define OLED_DISPLAYALLON_RESUME    0xA4
#define OLED_DISPLAYALLON           0xA5
#define OLED_NORMALDISPLAY          0xA6
#define OLED_INVERTDISPLAY          0xA7
#define OLED_DISPLAYOFF             0xAE
#define OLED_DISPLAYON              0xAF

#define OLED_SETDISPLAYOFFSET       0xD3
#define OLED_SETCOMPINS             0xDA

#define OLED_SETVCOMDETECT          0xDB

#define OLED_SETDISPLAYCLOCKDIV     0xD5
#define OLED_SETPRECHARGE           0xD9

#define OLED_SETMULTIPLEX           0xA8

#define OLED_SETLOWCOLUMN           0x00
#define OLED_SETHIGHCOLUMN          0x10

#define OLED_SETSTARTLINE           0x40

#define OLED_MEMORYMODE             0x20
#define OLED_COLUMNADDR             0x21
#define OLED_PAGEADDR               0x22

#define OLED_COMSCANINC             0xC0
#define OLED_COMSCANDEC             0xC8

#define OLED_SEGREMAP               0xA0

#define OLED_CHARGEPUMP             0x8D

#define OLED_EXTERNALVCC            0x1
#define OLED_SWITCHCAPVCC           0x2

#define OLED_DEACTIVATE_SCROLL   0x2E
//---------------- END OLED Definitions ----------------

//---------------- Clock Definitions ----------------
#define CLOCK_ADDR 0x68

#define CLOCK_TIME_REG 0x00
#define CLOCK_SQUARE_REG 0x0E

// These can be set at-will to change the initial time
#define DEF_SEC  0x15
#define DEF_MIN  0x56
#define DEF_HOUR 0x13
#define DEF_DAY  0x06
#define DEF_DOM  0x28
#define DEF_MON  0x04
#define DEF_YEAR 0x25
//---------------- End Clock Definitions ----------------

//---------------- BME280 Definitions ----------------
#define BME_ADDR 0x77

#define BME_HUM_REG 0xF2
#define BME_MEAS_REG 0xF4
#define BME_CONF_REG 0xF5

#define BME_READ_REG 0xF7
#define BME_CALI_REG = 0x88;  // Calibration register starting point for dig_T1
//---------------- End Clock Definitions ----------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////      Variables      ////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//---------------- OLED Variables ----------------

//---------------- End OLED Variables ----------------
char oled_command_buf[17]; // Buffer for I2C TX OLED Commands. No need for RX thankfully.

//---------------- Clock Variables ----------------

char clock_time_init[8] = { // Initialize default time
    CLOCK_TIME_REG,
    DEF_SEC, DEF_MIN, DEF_HOUR,
    DEF_DAY, DEF_DOM, DEF_MON, DEF_YEAR
};

char clock_square_wave[2] = {CLOCK_SQUARE_REG, 0x00}; // Command to enable the 1hz square wave on the clock

char clock_current_time[7]; // Current time and RX buffer for querying the RTC

//---------------- End Clock Variables ----------------

//---------------- BME280 Variables ----------------

/* CONFIGURATION CODE + BINARY -> HEX BREAKDOWN
 *
 * The BME280 is substantially more complicated than I had initially assumed, like actually being used in industry, smartphones, etc.
 * As such, it hosts an array of configurations that must be set to the user's desired application before being used, which at first
 * glance seem rather complicated and incredibly verbose. This is the same way for the actual temperature conversions, which I'll go over
 * later, but require a 32 bit number (keep in mind, the MSP430FR2355 hosts a 16 bit architecture) for each measurement.
 *
 * The registers onboard the BME280 take what can be somewhat complicated codes, with each byte essentially being a concatenation of
 * different codes/settings. Because of this, the codes themselves can be a little complicated, and while it's easier/more readable
 * to send binary, it can also be messy, so here's this translation table that indicates what each hex code I'm sending over actually
 * translates to.
 *
 * -----------------------------------------
 *
 * Reg 0xF2 - Humidity Oversampling Register
 * Bits 2 - 0: Humidity oversampling. I want 2x oversampling, so that would be xxxxx/010.
 * Since none of the othet bits matter, they'll just be zeroed out, becoming 00000/010.
 *
 * In hex, 00000010 -> 0x02
 *
 * -----------------------------------------
 *
 * Reg 0xF4 - Controls oversampling for Temp and Pressure, as well as sensor mode (how often it should read).
 * Bits 2 - 0: Pressure oversampling. I want 2x oversampling, so far that is xx/xxx/010.
 * Bits 5 - 3: Temperature oversampling. I want 4x oversampling, so far that is xx/011/010
 * Bits 7 - 6: Sensor mode. I want forced mode, meaning it reads once then goes back to sleep. That becomes 01/011/010
 *
 * In hex, 01011010 -> 0x5A
 *
 * -----------------------------------------
 *
 * Reg 0xF5 - None of this matters, as it's for the delay in normal sensor mode, a filter I don't need, and SPI enable, but I'm using I2C
 * So this would just be 0x00000000, as I don't want any of this to be set, or is completely irrelevant.
 *
 * In hex, this would obviously be 0x00
 *
 * -----------------------------------------
 *
 * Hopefully, this clears up my own ambiguity to what these hex codes actually translate to, as well as YOU READING THIS.
 *
 */


char bme_hum_init[2] = {BME_HUM_REG, 0x02}; // Initialize humidity with 2x oversampling

char bme_meas_init[2] = {BME_MEAS_REG, 0x5A}; // Initialize pressure with 2x OS, Temp with 2x OS, and forced sensor mode.

char bme_conf_init[2] = {BME_CONF_REG, 0x00}; // Standby = 0.5 ms (irrelevant), disable IIR filter, disable SPI mode

char bme_raw_out[8]; // RX buffer for querying the BME280.

volatile int32_t raw_pressure = 0;
volatile int32_t raw_temperature = 0;
volatile int32_t raw_humidity = 0;

volatile int32_t temperature_32;
volatile uint32_t pressure_32;
volatile uint32_t humidity_32;

volatile float temperature_C = 0;
volatile float pressure_Pa = 0;
volatile float humidity_RH = 0;

// CALIBRATION DATA
// These values are hard-coded into the BME, they need to be read at launch over I2C, and are used to calibrate the measurement.
// I'm not even going to pretend like I know how they're calculated, what they are, or how they're employed in each calculation.
// All of this was from Bosch (manufacturer), who basically said don't even try to wrap your head around it.

volatile int32_t tfine = 0;

// Temperature
volatile uint16_t dig_T1 = 0;
volatile int16_t  dig_T2 = 0;
volatile int16_t  dig_T3 = 0;

// Pressure
volatile uint16_t dig_P1 = 0;
volatile int16_t  dig_P2 = 0;
volatile int16_t  dig_P3 = 0;
volatile int16_t  dig_P4 = 0;
volatile int16_t  dig_P5 = 0;
volatile int16_t  dig_P6 = 0;
volatile int16_t  dig_P7 = 0;
volatile int16_t  dig_P8 = 0;
volatile int16_t  dig_P9 = 0;

// Humidity
volatile uint8_t  dig_H1 = 0;
volatile int16_t  dig_H2 = 0;
volatile uint8_t  dig_H3 = 0;
volatile int16_t  dig_H4 = 0;
volatile int16_t  dig_H5 = 0;
volatile int8_t   dig_H6 = 0;

//---------------- End BME280 Variables ----------------

//---------------- I2C Variables ----------------

char *i2c_buffer; // Point to which buffer we are writing to / reading from
volatile unsigned int i2c_length; // Length of the buffer we are writing to / reading from
volatile int transmission = 1; // Boolean for indicating an entire tranmission (tx or rx) is complete
volatile int tick = 0; // Boolean for if the 1hz tick interrupt has fired.

//---------------- End I2C Variables ----------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////      Functions      ////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tx_I2C(int addr, char *buffer, int len){
    while (!transmission); // Wait until previous transmission is completed

    transmission = 0;

    UCB0CTLW0 |= UCTR; // Put into Tx mode (WRITE)
    UCB0I2CSA = addr; // UCB0 address to what we passed in
    i2c_buffer = buffer;
    i2c_length = len;

    UCB0IE |= UCTXIE0; // local enable for TX0 IRQ

    UCB0CTLW0 |= UCTXSTT; // manually start message (START)

    while (!transmission); // Wait until current transmission is completed
    UCB0IE &= ~UCTXIE0; // disable TX0 IRQ
}

void rx_I2C(int addr, char *buffer, int len, char reg){
    while (!transmission); // Wait until previous transmission is completed

    tx_I2C(addr, &reg, 1); // TX the register we want to write to

    while (!transmission); // Wait until register bit is transmitted

    transmission = 0;

    UCB0I2CSA = addr; // UCB0 address to what we passed in
    i2c_buffer = buffer;
    i2c_length = len;

    UCB0CTLW0 &= ~UCTR; // Put into RX mode (READ)

    UCB0IE |= UCRXIE0; // local enable for RX0 IRQ

    UCB0CTLW0 |= UCTXSTT; // manually start message (START)

    while (!transmission); // Wait until current transmission is completed
    UCB0IE &= ~UCRXIE0; // disable RX0 IRQ
}

int32_t compensate_temperature(int32_t raw_temp) {
    volatile int32_t var1, var2, T;

    // Calculate fine temperature
    var1 = ((((raw_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((raw_temp >> 4) - ((int32_t)dig_T1)) * ((raw_temp >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    tfine = var1 + var2;

    // Calculate temperature
    T = (tfine * 5 + 128) >> 8;
    return T;
}

uint32_t compensate_pressure(int32_t raw_pressure) {
    volatile int32_t var4, var5;
    volatile uint32_t p;
    var4 = (((int32_t)tfine)>>1) - (int32_t)0xFA00;
    var5 = (((var4>>2) * (var4>>2)) >> 11 ) * ((int32_t)dig_P6);
    var5 = var5 + ((var4*((int32_t)dig_P5))<<1);
    var5 = (var5>>2)+(((int32_t)dig_P4)<<16);
    var4 = (((dig_P3 * (((var4>>2) * (var4>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var4)>>1))>>18;
    var4 = ((((0x8000+var4))*((int32_t)dig_P1))>>15);
    if (var4 == 0)
    {
        return 0; // Avoid exception caused by division by zero
    }
    p = (((uint32_t)(((int32_t)0x100000)-raw_pressure)-(var5>>12)))*0xC35;
    if (p < 0x80000000)
    {
        p = (p << 1) / ((uint32_t)var4);
    }
    else
    {
        p = (p / (uint32_t)var4) * 2;
    }
    var4 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
    var5 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
    p = (uint32_t)((int32_t)p + ((var4 + var5 + dig_P7) >> 4));
    return p;
}

uint32_t compensate_humidity(int32_t raw_humidity) {
    volatile int32_t var3;
    var3 = tfine - (int32_t)76800;

    var3 = ((((((int32_t)raw_humidity << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var3)) +
        ((int32_t)16384)) >> 15) * (((((((var3 * ((int32_t)dig_H6)) >> 10) * (((var3 *
        ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
        ((int32_t)dig_H2) + (int32_t)8192) >> 14));

    var3 = (var3 - (((((var3 >> 15) * (var3 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));

    if(var3 < 0) var3 = 0;
    if(var3 > 419430400) var3 = 419430400;

    return (uint32_t)(var3 >> 12);
}

void compensate(){
    temperature_32 = compensate_temperature(raw_temperature);
    pressure_32 = compensate_pressure(raw_pressure);
    humidity_32 = compensate_humidity(raw_humidity);
}

void read_calibration_data(void) {
    uint8_t addr = 0x88;  // Start address for temperature calibration data

    char temp_pres_calibration[24];

    // Read all the calibration data in one shot (24 bytes starting from 0x88)
    rx_I2C(BME_ADDR, temp_pres_calibration, sizeof(temp_pres_calibration), addr);

    // Temperature calibration data (first 6 bytes)
    dig_T1 = (temp_pres_calibration[0] | (temp_pres_calibration[1] << 8));  // Combine bytes to form 16-bit value
    dig_T2 = (temp_pres_calibration[2] | (temp_pres_calibration[3] << 8));  // Combine bytes to form 16-bit value
    dig_T3 = (temp_pres_calibration[4] | (temp_pres_calibration[5] << 8));  // Combine bytes to form 16-bit value

    // Pressure calibration data (next 18 bytes)
    dig_P1 = (temp_pres_calibration[6] | (temp_pres_calibration[7] << 8));  // 16-bit unsigned value
    dig_P2 = (temp_pres_calibration[8] | (temp_pres_calibration[9] << 8));  // 16-bit signed value
    dig_P3 = (temp_pres_calibration[10] | (temp_pres_calibration[11] << 8)); // 16-bit signed value
    dig_P4 = (temp_pres_calibration[12] | (temp_pres_calibration[13] << 8)); // 16-bit signed value
    dig_P5 = (temp_pres_calibration[14] | (temp_pres_calibration[15] << 8)); // 16-bit signed value
    dig_P6 = (temp_pres_calibration[16] | (temp_pres_calibration[17] << 8)); // 16-bit signed value
    dig_P7 = (temp_pres_calibration[18] | (temp_pres_calibration[19] << 8)); // 16-bit signed value
    dig_P8 = (temp_pres_calibration[20] | (temp_pres_calibration[21] << 8)); // 16-bit signed value
    dig_P9 = (temp_pres_calibration[22] | (temp_pres_calibration[23] << 8)); // 16-bit signed value

    char humidity_calibration[9];  // We need 9 bytes in total (H1, H2-H6)

    addr = 0xA1;
    // Read the first 2 bytes, starting at 0xA1 (dig_H1 and extra byte at 0xA2, which is ignored)
    rx_I2C(BME_ADDR, humidity_calibration, 2, addr);

    // Now read the next 7 bytes from address 0xE1 (dig_H2 to dig_H6)
    addr = 0xE1;
    rx_I2C(BME_ADDR, &humidity_calibration[1], 8, addr);  // Read H2-H6

    // Humidity calibration data assignments
    dig_H1 = humidity_calibration[0];  // Only use the first byte (H1)
    dig_H2 = (humidity_calibration[1] | (humidity_calibration[2] << 8));  // 16-bit signed value
    dig_H3 = humidity_calibration[3];  // 8-bit unsigned value
    dig_H4 = (humidity_calibration[4] << 4 | (humidity_calibration[5] & 0x0F));  // 12-bit signed value
    dig_H5 = (humidity_calibration[6] << 4 | ((humidity_calibration[7] >> 4) & 0x0F));  // 12-bit signed value
    dig_H6 = humidity_calibration[8];  // 8-bit signed value
}

void extract_raw_climate(){
    // Seemingly complicated yet quite elegant bit shift.
    raw_pressure = ((uint32_t)bme_raw_out[0] << 12) | ((uint32_t)bme_raw_out[1] << 4) | (bme_raw_out[2] >> 4);
    raw_temperature = ((uint32_t)bme_raw_out[3] << 12) | ((uint32_t)bme_raw_out[4] << 4) | (bme_raw_out[5] >> 4);
    raw_humidity = ((uint32_t)bme_raw_out[6] << 8) | bme_raw_out[7];
}

void OLED_command(unsigned char command) {
    oled_command_buf[0] = OLED_CMD;
    oled_command_buf[1] = command;

    tx_I2C(OLED_ADDR, oled_command_buf, 2);
}

void OLED_init(void) {
    // OLED init sequence
    OLED_command(OLED_DISPLAYOFF);                                // 0xAE
    OLED_command(OLED_SETDISPLAYCLOCKDIV);                        // 0xD5
    OLED_command(0x80);                                              // the suggested ratio 0x80

    OLED_command(OLED_SETMULTIPLEX);                              // 0xA8
    OLED_command(OLED_LCDHEIGHT - 1);

    OLED_command(OLED_SETDISPLAYOFFSET);                          // 0xD3
    OLED_command(0x0);                                               // no offset
    OLED_command(OLED_SETSTARTLINE | 0x0);                        // line #0
    OLED_command(OLED_CHARGEPUMP);                                // 0x8D
    OLED_command(0x14);                                              // generate high voltage from 3.3v line internally
    OLED_command(OLED_MEMORYMODE);                                // 0x20
    OLED_command(0x02);
    OLED_command(OLED_SEGREMAP | 0x1);
    OLED_command(OLED_COMSCANDEC);

    OLED_command(OLED_SETCOMPINS);                                // 0xDA
    OLED_command(0x02);
    OLED_command(OLED_SETCONTRAST);                               // 0x81
    OLED_command(0xCF);

    OLED_command(OLED_SETPRECHARGE);                              // 0xd9
    OLED_command(0xF1);
    OLED_command(OLED_SETVCOMDETECT);                             // 0xDB
    OLED_command(0x40);
    OLED_command(OLED_DISPLAYALLON_RESUME);                       // 0xA4
    OLED_command(OLED_NORMALDISPLAY);                             // 0xA6

    OLED_command(OLED_DEACTIVATE_SCROLL);

    OLED_command(OLED_DISPLAYON);                                 //--turn on oled panel
}

void OLED_set_position(uint8_t column, uint8_t page) {
    OLED_command(OLED_COLUMNADDR);      // Command to set column address
    OLED_command(column);               // Column start address (0–127)
    OLED_command(OLED_LCDWIDTH - 1);    // Column end address (127)

    OLED_command(OLED_PAGEADDR);        // Command to set page address
    OLED_command(page);                 // Page start address (0–3)
    OLED_command(page);                 // Page end address
}

void OLED_clear_display(void) {

    uint8_t page;
    for (page = 0; page < 4; page++) {  // SSD1306 128x32 has 4 pages
        OLED_set_position(0, page);  // Start of each page

        oled_command_buf[0] = OLED_DATA;  // Data control byte

        // Fill rest of buffer with 0xFF to turn all pixels ON for testing
        int i;
        for (i = 1; i < 17; i++) {
            oled_command_buf[i] = 0x00;
        }

        uint8_t col;
        for (col = 0; col < 128; col += 16) {
            tx_I2C(OLED_ADDR, oled_command_buf, 17);  // 16 bytes of data
        }
    }
}

void OLED_draw_text(uint8_t column, uint8_t page, const char *text) {
    uint8_t i, j;
    uint8_t c;

    OLED_set_position(column, page);  // Set starting location

    oled_command_buf[0] = OLED_DATA;  // First byte always control byte for data
    i = 1;  // Index into oled_command_buf — starting at 1 because 0 is control byte

    while (*text && column < (OLED_LCDWIDTH - 6)) {
        c = *text++;  // Read next character from string

        // Basic ASCII range check (font_5x7 starts at ASCII 32 — space)
        if (c < 32 || c > 127) {
            c = '?';  // Use '?' as fallback for unsupported chars
        }

        // Copy 5 bytes of the character glyph
        for (j = 0; j < 5; j++) {
            oled_command_buf[i++] = font_5x7[c - 32][j];
            column++;

            // If we filled the buffer (16 bytes of data max), send it
            if (i == 17) {
                tx_I2C(OLED_ADDR, oled_command_buf, 17);
                i = 1;  // Reset to start new data payload
            }
        }

        // Add 1-pixel space between characters
        oled_command_buf[i++] = 0x00;
        column++;

        if (i == 17) {
            tx_I2C(OLED_ADDR, oled_command_buf, 17);
            i = 1;
        }
    }

    // Send any remaining bytes in the buffer
    if (i > 1) {
        tx_I2C(OLED_ADDR, oled_command_buf, i);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////        Main         ////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //----------------- Setup UCBO & P1 for I2C -----------------
    UCB0CTLW0 |= UCSWRST; // put in software reset

    UCB0CTLW0 |= UCSSEL_3; // choose SMCLK
    UCB0BRW = 10; // set prescalar to 10 (1 MHz / 10 = 100 kHz)

    UCB0CTLW0 |= UCMODE_3; //put into I2C mode
    UCB0CTLW0 |= UCMST; // Set as master

    UCB0CTLW0 &= ~UCSWRST; // take B0 out of SW reset

    // Setup P1.3 for SCL
    P1SEL1 &= ~BIT3;
    P1SEL0 |= BIT3;

    // Setup P1.2 for SDA
    P1SEL1 &= ~BIT2;
    P1SEL0 |= BIT2;
    //-----------------END  Setup UCBO & P1 for I2C -----------------

    //----------------- Setup P1.1 for input interrupt -----------------
    // Configure P1.1 for digital I/O
    P1SEL0 &= ~BIT1;
    P1SEL1 &= ~BIT1;

    P1DIR &= ~BIT1; // Clear P1.1 Direction = input mode
    P1REN |= BIT1; // Enable pull up/down resistor
    P1OUT |= BIT1; // Make resistor a pull up

    P1IES |= BIT1; // Enable high to low sensitivity
    P1IFG &= ~BIT1; // Clear IFG flag
    P1IE |= BIT1; // Enable interrupt for P1
    //----------------- End Setup P1.1 for input interrupt -----------------


    PM5CTL0 &= ~LOCKLPM5; // Turn on I/0
    __enable_interrupt(); // enable maskables

    // OLED Initialization
    OLED_init();

    // Clock Initialization
    //tx_I2C(CLOCK_ADDR, clock_square_wave, sizeof(clock_square_wave)); // Enable 1hz square wave on the DS321

    //tx_I2C(CLOCK_ADDR, clock_time_init, sizeof(clock_time_init)); // Set initial time

    //BME280 Initialization
    //read_calibration_data();

    //tx_I2C(BME_ADDR, bme_conf_init, sizeof(bme_conf_init)); // Configurations

    //tx_I2C(BME_ADDR, bme_hum_init, sizeof(bme_hum_init)); // Set Humidity

    OLED_clear_display();

    OLED_draw_text(0, 0, "I'm on that good");

    OLED_draw_text(0, 1, "kush and alcohol.");

    OLED_draw_text(0, 2, "I got some down");

    OLED_draw_text(0, 3, "bitches I can call.");

    while(1){}

    while(1){
        if(tick){
            tick = 0;

            //rx_I2C(CLOCK_ADDR, clock_current_time, sizeof(clock_current_time), CLOCK_TIME_REG); // Get the time

            tx_I2C(BME_ADDR, bme_meas_init, sizeof(bme_meas_init)); // Sets pressure/temp oversampling and starts measurements

            __delay_cycles(160000); // ~10 ms delay for the BME to finish its measurement.

            rx_I2C(BME_ADDR, bme_raw_out, sizeof(bme_raw_out), BME_READ_REG); // Get BME climate measurements

            extract_raw_climate(); // Extract climate data from the rx and put into appropriate variables.

            compensate(); //Compensate each measurement and assign to appropriate variable.
        }
    }

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////     Interrupts      ////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma vector = PORT1_VECTOR
__interrupt void ISR_Port1(void){
    tick = 1;
    P1IFG &= ~BIT1;
}

#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void){
    static unsigned int index = 0;

    switch(__even_in_range(UCB0IV, 0x1E)){

        case 0x16: // RX BUFFER FULL
            if ((index == (i2c_length - 2))){
                UCB0CTLW0 |= UCTXSTP; // Send STOP before reading the last byte
            }

            if(index < i2c_length){
                i2c_buffer[index++] = UCB0RXBUF;
            }else{
                char garbage = UCB0RXBUF; // Dispose of any extra RX data we don't want just to be safe.
            }

            if (index == i2c_length){
                index = 0;
                transmission = 1;
            }

            break;

        case 0x18: // TX BUFFER TRANSMITTED
            if (index < i2c_length) {
                UCB0TXBUF = i2c_buffer[index++];
            } else {
                UCB0CTLW0 |= UCTXSTP; // Send STOP
                index = 0;
                transmission = 1;
            }
            break;

        default: break;
    }
}

