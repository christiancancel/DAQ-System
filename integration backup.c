#include "msp.h"
#include <stdint.h>

//make reading boolean checks easier
typedef enum boolean {false, true} bool;
//used to decide if we're turning channels on or off
typedef enum channelControl {down, up} control;

//variables for dealing with instructions coming in
char instructions[20];
char count = 0;

//sram
uint8_t sampleData[12];
int datacount;

//module settings
struct options{
    char mode;
    char period;
    char gain;
    char channels;
    char format;
    char frequency;
    char ratio;
    bool testing;
} settings;

//operation flags
bool sync = false, request = false, initialized = false;

//we currently only use high resolution mode
void setOperatingMode(char mode){
    settings.mode = mode;
}
void setSamplingPeriod(char period){
    settings.period = period;
}
void setGainFactor(char gain){        //amplification is selected in steps
    settings.gain = gain;
}
void restartConversion(){                           //sync
    sync = false;
}
void toggleChannels(control power, char channels){           //power up/down channels
    if((channels & 0xF0) == 0xE0){
        settings.channels = (power == down) ? settings.channels & (channels ^= 0x0F) : settings.channels | channels;
    }
}

//SRAM
void writeSample(int address, uint8_t sample[12]){
    P3OUT &= ~BIT3;
    address *= 12;
    EUSCI_A1->TXBUF = (uint8_t) 0x02;
    EUSCI_A1->TXBUF = (uint8_t) (address >> 16);    //8 most significant bits of address
    EUSCI_A1->TXBUF = (uint8_t) (address >> 8);
    EUSCI_A1->TXBUF = (uint8_t) (address);          //8 least significant bits of address
//    P3OUT &= ~BIT2;
    EUSCI_A1->TXBUF = sample[0];
    EUSCI_A1->TXBUF = sample[1];
    EUSCI_A1->TXBUF = sample[2];
    EUSCI_A1->TXBUF = sample[3];
    EUSCI_A1->TXBUF = sample[4];
    EUSCI_A1->TXBUF = sample[5];
    EUSCI_A1->TXBUF = sample[6];
    EUSCI_A1->TXBUF = sample[7];
    EUSCI_A1->TXBUF = sample[8];
    EUSCI_A1->TXBUF = sample[9];
    EUSCI_A1->TXBUF = sample[10];
    EUSCI_A1->TXBUF = sample[11];
    P3OUT |= BIT3;
//    P3OUT |= BIT2;
}

static int record[200];
int recordcount = 0;
char readingCheck[36];
void readSample(int address){
    short i;
    P3OUT &= ~BIT3;                     //!CS low
    EUSCI_A1->TXBUF = (uint8_t) 0x03;             //read instruction
    address *= 12;
    while(!(EUSCI_A1->IFG & BIT1));
    EUSCI_A1->TXBUF = (uint8_t) (address >> 16);    //8 most significant bits of address
    while(!(EUSCI_A1->IFG & BIT1));
    EUSCI_A1->TXBUF = (uint8_t) (address >> 8);
    while(!(EUSCI_A1->IFG & BIT1));
    EUSCI_A1->TXBUF = (uint8_t) (address);          //8 least significant bits of address
    while(!(EUSCI_A1->IFG & BIT1));
    EUSCI_A1->IFG &= ~BIT0;
    for(i=0; i<12; i++){
        EUSCI_A1->TXBUF = 0x00;
        while(!(EUSCI_A1->IFG & BIT0));     //reading sample data
        readingCheck[i] = EUSCI_A1->RXBUF;
    }
    record[recordcount++] = (int) readingCheck[0]<<16 | readingCheck[1]<<8 | readingCheck[2];
    P3OUT |= BIT3;
}

void SRAMinit(){
    //P3.3 to be used as SRAM selector (!CS)
    P3DIR |= BIT3;
    P3OUT |= BIT3;
//    P3DIR |= BIT2;
//    P3OUT |= BIT2;

    P2->SEL0 |= BIT0 | BIT1 | BIT2 | BIT3;

    EUSCI_A1->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI state machine in reset
    EUSCI_A1->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI state machine in reset
            EUSCI_A_CTLW0_MST |             // Set as SPI master
            EUSCI_A_CTLW0_SYNC |            // Set as synchronous mode
            EUSCI_A_CTLW0_CKPL |            // Set clock polarity high
            EUSCI_A_CTLW0_MSB;              // MSB first

    EUSCI_A1->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK; // SMCLK
    EUSCI_A1->BRW = 0x01;                   // /2,fBitClock = fBRCLK/(UCBRx+1).
    EUSCI_A1->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;// Initialize USCI state machine
}

void SRAMtesting(){
    SRAMinit();
    int i;
    for(i=0;i<12;i++){
        sampleData[i]=i;
    }
//    for(i=0; i<3; i++){
//        EUSCI_A1->TXBUF = (uint8_t)0x02;
//        EUSCI_A1->TXBUF = (uint8_t)0x0;
//        EUSCI_A1->TXBUF = (uint8_t)0x0;
//        EUSCI_A1->TXBUF = (uint8_t)0x0;
    while(1){
        writeSample(0,sampleData);
        for(i=0;i<200;i++);
        readSample(0);
    }

//    }
//    for(i=0;i<3;i++){

//    }
}

void startSampling(){
    SRAMinit();
    /*
     * to be implemented
     * trash the first 78 samples, since it takes 78 samples for the ADC to stabilize
     */
    datacount = 0;
    P3OUT &= ~BIT3;                     //!CS low
    EUSCI_A1->TXBUF = 0x02;             //write instruction
    EUSCI_A1->TXBUF = (uint8_t) 0;    //8 most significant bits of address
    EUSCI_A1->TXBUF = (uint8_t) 0;
    EUSCI_A1->TXBUF = (uint8_t) 0;          //8 least significant bits of address
//    EUSCI_A1->TXBUF = (uint8_t) (address >> 16);    //8 most significant bits of address
//    EUSCI_A1->TXBUF = (uint8_t) (address >> 8);
//    EUSCI_A1->TXBUF = (uint8_t) address;          //8 least significant bits of address
    P4->SEL0 |= BIT3;   //SMCLK pin output selected
}

void stopSampling(){
    P4->SEL0 &= ~BIT3;   //SMCLK pin output selected
    short i;
    for(i=0; i<200;i++){
        readSample(i);
    }
}

void transmitData(){
    request = false;
}

static uint8_t TXData;
void ADCinit(){
    P4->DIR |= BIT3;    //SMCLK output pin
    //Pin interrupt for DRDY
    P5->IES |= BIT1;                            // Enable Interrupt on Falling Edge of DRDY
    P5->IE |= BIT1;                             // Port Interrupt Enable
    P5->IFG = 0;                                // Clear Port Interrupt Flag
    P5->DIR &= ~BIT1;                           // Set Pin as Input
    NVIC->ISER[1] |= 1 << ((PORT5_IRQn) & 31);  // Enable PORT5 interrupt in NVIC module

    //SPI Initialization/Reconfiguration
    P1->SEL0 |= BIT5 | BIT6 | BIT7;             // Set P1.5, P1.6, and P1.7 as
                                                // SPI pins functionality
    EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_SWRST;      // Put eUSCI state machine in reset
    EUSCI_B0->CTLW0 = EUSCI_B_CTLW0_SWRST |     // Remain eUSCI state machine in reset
            EUSCI_B_CTLW0_MST |                 // Set as SPI master
            EUSCI_B_CTLW0_SYNC |                // Set as synchronous mode
            EUSCI_B_CTLW0_MSB;                  // MSB first

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK;// SMCLK
    EUSCI_B0->BRW = 0x00;                       // /1, fBitClock = fBRCLK/(UCBRx+1).
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;    // Initialize USCI state machine
}

void PGAMuxInit(){

    //Juan Begin
    P2->SEL0 |= BIT0 | BIT1 | BIT2 | BIT3; //set sel1 sel0 bits to select mapping of the pins
    //01, is the mapping of the SPI communication.
    P4->DIR |= BIT4;
//    P4->DIR |= BIT2;
//    P4->OUT |= BIT2;

    UCA1CTLW0 = UCSWRST;
    //set up SPI
    UCA1CTLW0 |= UCMST | //master mode
            UCMODE_2 | //4 pin slave enabled on low
            UCSYNC | //synchronous mode
            UCSSEL_2 | //set clock source as SMCLK
            //UCMSB | //to see if msb is what i have wrong
            UCSTEM |//used to generate signal for a 4-wire salve
            UCCKPH |
            UCSWRST; //software reset enable

    UCA1BRW |= 0x00;                   // /2,fBitClock = fBRCLK/(UCBRx+1).
    UCA1CTLW0 &= ~UCSWRST;             // **Initialize USCI state machine**
    //TESTING
//    changePGAAMP(0);
//    changeMUX(1);
    //Juan End
}

void muxinit(){
    P4->DIR |= BIT2;
    P4->OUT |= BIT2;
}

void changeMUX(int in){
    muxinit();

    settings.testing = (in) ? false : true;
    //in = 1, sensor
    //in = 0, tester
    if(in) P4->OUT |= BIT2;
    if(!in) P4->OUT &= ~BIT2;
}


static uint8_t TXData;
void changePGAAMP(int amp){
    PGAMuxInit();

    int temp = pgatoHex(amp);
    TXData = temp;
    P4->OUT &= ~BIT4;
    UCA1TXBUF |= (uint8_t) TXData;
    while (UCA1STATW & EUSCI_A_STATW_BUSY);
    P4->OUT |= BIT4;
}

uint8_t pgatoHex(int a){ // 8 levels, 1, 2, 3, 4, 5, 6, 7, 8
   uint8_t hex = 0x01;
   switch (a) {
    case 1: //10
        hex = (uint8_t) 0x03;
        break;
    case 2: //20
        hex = (uint8_t) 0x05;
        break;
    case 3: //30
        hex = (uint8_t) 0x07;
        break;
    case 4: //40
        hex = (uint8_t) 0x09;
        break;
    case 5: //60
        hex = (uint8_t) 0x0B;
        break;
    case 6: //80
        hex = (uint8_t) 0x0C;
        break;
    case 7: //120
        hex = (uint8_t) 0x0F;
        break;
    case 8: //157
        hex = (uint8_t) 0x11;
        break;
    default:
        hex = (uint8_t) 0x01;
        break;
   }

   return hex;
}


//i2c setup instructions
void i2cinit(){
    EUSCI_B2->CTLW0 = (uint16_t) 0x0001;                //CTLW0[0] = UCSWRST = 1b       /set so we can modify the rest of the register
    EUSCI_B2->CTLW0 &= (uint16_t) 0x7FFF;               //CTLW0[15] = UCA10 = 0b        /set own address to 7bit
    EUSCI_B2->CTLW0 &= (uint16_t) 0xF7FF;               //CTLW0[11] = UCMST = 0b        /slave
    EUSCI_B2->CTLW0 |= (uint16_t) 0x0600;               //CLTW0[10:9] = UCMODE = 11b    /I2C mode
    EUSCI_B2->BRW |= (uint16_t) 0xD;                    //(int) clk/baud rate           /baud rate=10000

    //EUSCI_B2 SDA
    P3SEL1 &= ~BIT6;
    P3SEL0 |= BIT6;
    //EUSCI_B2 SCL
    P3SEL1 &= ~BIT7;
    P3SEL0 |= BIT7;

    EUSCI_B2->I2COA0 |= 0x0460;                         //address enabled + slave module address = 110 0000

    EUSCI_B2->CTLW0 &= (uint16_t) 0xFFFE;               //UCSWRST = 0

    EUSCI_B2->IE = 0x0009;                                //interrupts enabled
    NVIC->ISER[0] = 1 << (EUSCIB2_IRQn);
}

//DMA is future work
//void DMAinit(){
//    //DMA_SW_CHTRIG = 0x4 trigger channel 2
//    //DMA_CH2_SRCCFG = 0x1 for EUUSCI_A1 TX on channel 2
//    //DMA_CTLBASE = 0 primary data structure at address 0
//    //DMA_ENASET = 0x4 enable channel 2
//    /*channel_cfg 32bit
//     * [31:30] = 00b source address increment byte
//     * [29:28] = 00b destination is byte
//     * [27:26] = 11b destination address remains the same
//     * [25:24] = 00b source data is byte
//     * [23:21] = 100b destination access is cacheable/non bufferable/non priviledged
//     * [20:18] = 110b source access is cacheable/bufferable/non priviledged
//     * [17:14] = 0010b arbitrates after 4 transfers
//     * [13:4] = 14 performs 15 dma transfers
//     * [3] = 0 related to alternate structure
//     * [2:0] = 010b = auto request, controller inserts requests for appropriate channel by itself   */
//}


//settings stored in global variables and appropriate functions called
int muxstate;
void interpretInstruction(char* instruction){
    short i; //for loop counter
    if(instruction[0] == 0xCA){         //check header
        for(i = 1; i<count; i++){
            switch(instruction[i]){
            case 0x60:                                      //mode select
                setOperatingMode(instruction[++i]);
                break;
            case 0x61:
                setSamplingPeriod(instruction[++i]);        //sample period
                break;
            case 0x62:                                      //gain select
                setGainFactor(instruction[++i]);
                changePGAAMP(settings.gain);
                break;
            case 0x63:                                      //diagnostics
                muxstate = (int) instruction[++i];
                changeMUX(muxstate);
                SRAMinit();
                break;
            case 0x64:                                      //sync
                sync = true;
                i++;
                break;
            case 0x65:                                      //power down
                toggleChannels(down, instruction[++i]);
                break;
            case 0x66:                                      //power up
                toggleChannels(up, instruction[++i]);
                break;
            case 0x68:                                      //set output format
                i++;
                break;
            case 0x9A:                                      //begin sampling
                startSampling();
                break;
            case 0x9B:
                SRAMtesting();                              //test ram write;
                break;
            case 0x6C:                                      //set clock frequency
                i++;
                break;
            case 0x6E:                                      //set SClock ratio
                i++;
                break;
            case 0x6F:                                      //data request
                request = true;
                i++;
                break;
            default:
                i++;
            }
        }
    }
}

void clockSystem(){
    CS->KEY = CS_KEY_VAL ;                  // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 48MHz
    // Select ACLK = REFO, SMCLK = MCLK = DCO
    CS->CTL1 = CS_CTL1_SELA_2 | CS_CTL1_SELS_3 | CS_CTL1_SELM__DCOCLK;
    CS->KEY = 0;                            // Lock CS module from unintended accesses
}

void main(void){
    clockSystem();

    i2cinit();

    ADCinit();

    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;       // Wake up on exit from ISR

    // Ensures SLEEPONEXIT takes effect immediately
    __DSB();

    //comes back to do nothing
    int i = 0;

    changeMUX(0);
    changePGAAMP(2);


    //SRAMtesting();
    for (i = 200; i > 0; i--);        // Delay
    startSampling();
    while (1)
    {
        __sleep();
        __no_operation();                   // For debugger
    }
}

//timer interrupt that gets conversion in memory
void TA0_0_IRQHandler(void){
    if(TIMER_A0->CCTL[0] & BIT0){
        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
    }
    TIMER_A0->CCTL[0] &= ~BIT0;
}

/*
 * Pin Interrupt in P5.1 in order to receive 96 bits
 * when ADC spits out new data.
 */
void PORT5_IRQHandler(void)
{
    if (P5IFG & BIT1)
    {
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[0] = EUSCI_B0->RXBUF;            // Store first byte of channel 1 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[1] = EUSCI_B0->RXBUF;            // Store second byte of channel 1 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[2] = EUSCI_B0->RXBUF;            // Store third byte of channel 1 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[3] = EUSCI_B0->RXBUF;            // Store first byte of channel 2 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[4] = EUSCI_B0->RXBUF;            // Store second byte of channel 2 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[5] = EUSCI_B0->RXBUF;            // Store third byte of channel 2 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[6] = EUSCI_B0->RXBUF;            // Store first byte of channel 3 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[7] = EUSCI_B0->RXBUF;            // Store second byte of channel 3 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[8] = EUSCI_B0->RXBUF;            // Store third byte of channel 3 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[9] = EUSCI_B0->RXBUF;            // Store first byte of channel 4 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[10] = EUSCI_B0->RXBUF;            // Store second byte of channel 4 in the array
        EUSCI_B0->TXBUF = TXData;                   // Transmit dummy data to generate bit clock
        while(!(EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG));// Wait until we receive entire byte to read
        sampleData[11] = EUSCI_B0->RXBUF;            // Store third byte of channel 4 in the array
    }
    writeSample(datacount, sampleData);
    if(++datacount >= 200) stopSampling();
    P5->IFG = 0;                                // Clear Port Interrupt Flag

}

//I2C interrupts
void EUSCIB2_IRQHandler(void){
    //something has been received
    if(EUSCI_B2->IFG & BIT0){
        char received;
        received = EUSCI_B2->RXBUF;
        EUSCI_B2->IFG &= ~BIT0;
        if(initialized == false && ((received & 0xF0) == 0x50)){                               //set new address if not initialized yet
            EUSCI_B2->I2COA0 = 0x8400 + received;
            initialized =  true;
            EUSCI_B2->IFG &= ~BIT3;         //turning this flag off to make sure it doesn't try to interpret instructions
        }
        else{                                           //storing instruction byte
            instructions[count++] = received;
        }

    }
    else{           //after instructions have all been stored
        interpretInstruction(instructions);
        count = 0;
        EUSCI_B2->IFG &= ~BIT3;
    }

    //status is being requested
//    else if(EUSCI_B2->IFG & BIT1){
//        EUSCI_B2->TXBUF = message;
//        EUSCI_B2->IFG &= ~BIT1;
//    }
}
