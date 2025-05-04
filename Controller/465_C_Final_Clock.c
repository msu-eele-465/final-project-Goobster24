#include <msp430.h>
#include <stdint.h>

/////////////////////////////////////////////////////////////////////////////
////////////////////////////     Definitions     ////////////////////////////
/////////////////////////////////////////////////////////////////////////////

//---------------- Clock Definitions ----------------
#define CLOCK_ADDR 0x68

#define CLOCK_TIME_REG 0x00
#define CLOCK_SQUARE_REG 0x0E

// These can be set at-will to change the initial
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
//---------------- End Clock Definitions ----------------

/////////////////////////////////////////////////////////////////////////////
////////////////////////////      Variables      ////////////////////////////
/////////////////////////////////////////////////////////////////////////////

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

uint32_t raw_pressure = 0;
uint32_t raw_temperature = 0;
uint32_t raw_humidity = 0;

//---------------- End BME280 Variables ----------------

//---------------- I2C Variables ----------------
char *i2c_buffer; // Point to which buffer we are writing to / reading from
volatile unsigned int i2c_length; // Length of the buffer we are writing to / reading from
volatile int transmission = 1; // Boolean for indicating an entire tranmission (tx or rx) is complete
volatile int tick = 0; // Boolean for if the 1hz tick interrupt has fired.
//---------------- End I2C Variables ----------------

/////////////////////////////////////////////////////////////////////////////
////////////////////////////      Functions      ////////////////////////////
/////////////////////////////////////////////////////////////////////////////

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

void extract_raw_climate(){
    // Seemingly complicated yet quite elegant bit shift.
    raw_pressure = ((uint32_t)bme_raw_out[0] << 12) | ((uint32_t)bme_raw_out[1] << 4) | (bme_raw_out[2] >> 4);
    raw_temperature = ((uint32_t)bme_raw_out[3] << 12) | ((uint32_t)bme_raw_out[4] << 4) | (bme_raw_out[5] >> 4);
    raw_humidity = ((uint32_t)bme_raw_out[6] << 8) | bme_raw_out[7];
}

/////////////////////////////////////////////////////////////////////////////
////////////////////////////        Main         ////////////////////////////
/////////////////////////////////////////////////////////////////////////////

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

    // Clock Initialization
    //tx_I2C(CLOCK_ADDR, clock_square_wave, sizeof(clock_square_wave)); // Enable 1hz square wave on the DS321

    tx_I2C(CLOCK_ADDR, clock_time_init, sizeof(clock_time_init)); // Set initial time

    //BME280 Initialization
    tx_I2C(BME_ADDR, bme_conf_init, sizeof(bme_conf_init)); // Configurations

    tx_I2C(BME_ADDR, bme_hum_init, sizeof(bme_hum_init)); // Set Humidity

    //tx_I2C(BME_ADDR, bme_meas_init, sizeof(bme_meas_init)); Starts measurements from the BME, so I'll save this for the loop

    // Send over all the config stuff for the BME280

    while(1){
        if(tick && transmission){
            tick = 0;

            //rx_I2C(CLOCK_ADDR, clock_current_time, sizeof(clock_current_time), CLOCK_TIME_REG); // Get the time

            tx_I2C(BME_ADDR, bme_meas_init, sizeof(bme_meas_init)); // Sets pressure/temp oversampling and starts measurements

            __delay_cycles(160000); // ~10 ms delay for the BME to finish its measurement.

            rx_I2C(BME_ADDR, bme_raw_out, sizeof(bme_raw_out), BME_READ_REG); // Get BME climate measurements

            extract_raw_climate(); // Extract climate data from the rx and put into appropriate variables.
        }
    }

    return 0;
}

/////////////////////////////////////////////////////////////////////////////
////////////////////////////     Interrupts      ////////////////////////////
/////////////////////////////////////////////////////////////////////////////

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
            if (index == (i2c_length - 2)){
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

