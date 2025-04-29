#include <msp430.h>


//---------------- Clock Definitions ----------------
#define CLOCK_ADDR 0x68

#define CLOCK_TIME_REG 0x00
#define CLOCK_SQUARE_REG 0x0E

#define DEF_SEC  0x15
#define DEF_MIN  0x56
#define DEF_HOUR 0x13
#define DEF_DAY  0x06
#define DEF_DOM  0x28
#define DEF_MON  0x04
#define DEF_YEAR 0x25
//---------------- End Clock Definitions ----------------

//---------------- Clock Variables ----------------
char clock_time_init[8] = {
    CLOCK_TIME_REG,  // First byte is the register address
    DEF_SEC, DEF_MIN, DEF_HOUR,
    DEF_DAY, DEF_DOM, DEF_MON, DEF_YEAR
};

char clock_current_time[7];// Current time and RX buffer for querying the RTC

char clock_square_wave[2] = {CLOCK_SQUARE_REG, 0x00}; // Command to enable the 1hz square wave on the clock
//---------------- End Clock Variables ----------------

//---------------- I2C Variables ----------------
char *i2c_buffer; // Point to which buffer we are writing to / reading from
volatile unsigned int i2c_length; // Length of the buffer we are writing to / reading from
volatile int transmission = 1; // Boolean for indicating an entire tranmission (tx or rx) is complete
//---------------- End I2C Variables ----------------

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

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //----------------- Setup UCBO for I2C -----------------
    UCB0CTLW0 |= UCSWRST; // put in software reset

    UCB0CTLW0 |= UCSSEL_3; // choose SMCLK
    UCB0BRW = 10; // set prescalar to 10 (1 MHz / 10 = 100 kHz)

    UCB0CTLW0 |= UCMODE_3; //put into I2C mode
    UCB0CTLW0 |= UCMST; // Set as master

    UCB0CTLW0 &= ~UCSWRST; // take B0 out of SW reset
    //-----------------END  Setup UCBO for I2C -----------------

    //-- Setup P1
    // Setup P1.3 for SCL
    P1SEL1 &= ~BIT3;
    P1SEL0 |= BIT3;

    // Setup P1.2 for SDA
    P1SEL1 &= ~BIT2;
    P1SEL0 |= BIT2;

    PM5CTL0 &= ~LOCKLPM5; // Turn on I/0
    __enable_interrupt(); // enable maskables

    tx_I2C(CLOCK_ADDR, clock_square_wave, sizeof(clock_square_wave)); // Enable 1hz square wave on the DS321

    tx_I2C(CLOCK_ADDR, clock_time_init, sizeof(clock_time_init)); // Set initial time

    while(1){
        //rx_I2C(CLOCK_ADDR, clock_current_time, sizeof(clock_current_time), CLOCK_TIME_REG);
    }


    return 0;
}

//-- ISRs
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void){
    // ISR for Tx, iterate along packet and send each byte
    static unsigned int index = 0;

    switch(__even_in_range(UCB0IV, 0x1E)){
        case 0x16:

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

        case 0x18:
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

