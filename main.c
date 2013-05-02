#include <msp430.h> 

/*
 * main.c
 */

#define CONNECTED_LED_ON()      (P1OUT |= BIT0)
#define CONNECTED_LED_OFF()     (P1OUT &= ~BIT0)

#define EAP_RX_BUF              UCA0RXBUF
#define EAP_TX_BUF              UCA0TXBUF

#define EAP_RX_VECTOR           USCIAB0RX_VECTOR
#define EAP_TX_VECTOR           PORT2_VECTOR

#define EAP_RX_ACK_CONFIG()     (P2DIR |= BIT0)
#define EAP_RX_ACK_SET()        (P2OUT |= BIT0)
#define EAP_RX_ACK_CLR()        (P2OUT &= ~BIT0)

#define EAP_TX_INT_CONFIG()     (P2DIR &= ~BIT1, P2IES |= BIT1, P2IFG &= BIT1, P2IE |= BIT1)
#define EAP_TX_INT_TST()        (P2IFG & BIT1)
#define EAP_TX_INT_CLR()        (P2IFG &= ~BIT1)


/// CARD READER:
#define SCL BIT4
#define SDA BIT5
//#define READ 0xA1
//#define WRITE 0xA0
unsigned char READ = 0xA1;
unsigned char WRITE = 0xA0;

#define FAILURE -1
#define SUCCESS 0

void sendByte(void);
void receiveByte(void);
void sendAck(void);
void receiveAck(void);
void start(void);
void stop(void);

unsigned char txData = 0;
unsigned char rxData = 0;
unsigned char ackFlag = 0;
unsigned char bitCounter = 0;
unsigned int address = 0; // Address of the card to be read/written
unsigned int addrRef = 0; // Reference to see which is the current block
//int block = 0;
unsigned int block = 0;

int writeChar(void);
int readChar(void);
int readCurrentChar(void);
int writeInt(void);
int readInt(void);

void readString(int sizeInBytes);
void writeString(unsigned char *theString, int sizeInBytes);

unsigned char charData = 0;
unsigned char *string;

// The string being read is stored in this array
unsigned char theReadString[127];
//unsigned char theReadString[2048];	// Eight blocks of 256 bytes -> 2048 total bytes in a card

void init(void) {

    WDTCTL = WDTPW + WDTHOLD;
    BCSCTL2 = SELM_0 + DIVM_0 + DIVS_0;
    if (CALBC1_1MHZ != 0xFF) {
        DCOCTL = 0x00;
        BCSCTL1 = CALBC1_1MHZ;      /* Set DCO to 1MHz */
        DCOCTL = CALDCO_1MHZ;
    }
    BCSCTL1 |= XT2OFF + DIVA_0;
    BCSCTL3 = XT2S_0 + LFXT1S_2 + XCAP_1;  // Setting clock to low frequency occillator

    P1DIR |= BIT0;  /* LED */
    P1DIR |= BIT6;
    CONNECTED_LED_OFF();

    UCA0CTL1 |= UCSWRST;  // Reset USCI

    P1SEL |= BIT1 + BIT2;  // Select pins for uart
    P1SEL2 |= BIT1 + BIT2;

    EAP_RX_ACK_CONFIG();
    EAP_RX_ACK_SET();

    EAP_TX_INT_CONFIG();

    UCA0CTL1 = UCSSEL_2 + UCSWRST;
    UCA0MCTL = UCBRF_0 + UCBRS_6;
    UCA0BR0 = 8; 						// 115200 baud
    UCA0CTL1 &= ~UCSWRST;

    IFG2 &= ~(UCA0RXIFG);
    IE2 |= UCA0RXIE;

//    TA1CCTL0 = CM_0 + CCIS_0 + OUTMOD_0 + CCIE;
//    TA1CCR0 = 1200;
//    TA1CTL = TASSEL_1 + ID_0 + MC_1;


    ////////////////////////////////////////////////////
    ////// INIT ADC FOR SENSOR //////////////////

    P1DIR &= ~BIT7;		//Set P1.7 as input

    ADC10CTL0 &= ~ENC;

        /*
         * Control Register 0
         *
         * ~ADC10SC -- No conversion
         * ~ENC -- Disable ADC
         * ~ADC10IFG -- Clear ADC interrupt flag
         * ~ADC10IE -- Disable ADC interrupt
         * ADC10ON -- Switch On ADC10
         * ~REFON -- Disable ADC reference generator
         * ~REF2_5V -- Set reference voltage generator = 1.5V
         * ~MSC -- Disable multiple sample and conversion
         * ~REFBURST -- Reference buffer on continuously
         * ~REFOUT -- Reference output off
         * ~ADC10SR -- Reference buffer supports up to ~200 ksps
         * ADC10SHT_0 -- 4 x ADC10CLKs
         * SREF_0 -- VR+ = VCC and VR- = VSS
         *
         * Note: ~<BIT> indicates that <BIT> has value zero
         */
        ADC10CTL0 = ADC10ON + ADC10SHT_0 + SREF_0;
        ADC10CTL1 = INCH_7;		//Select adc channel 7 (P1.7)
        /* Analog (Input) Enable Control Register 0 */
        ADC10AE0 = 0x1;

        /* enable ADC10 */
        ADC10CTL0 |= ENC;

    __enable_interrupt();
}

/*
 * ============ Serial Driver ============
 */

#include <Em_Message.h>

#pragma vector=EAP_RX_VECTOR
__interrupt void rxHandler(void) {
    uint8_t b = EAP_RX_BUF;
    if (Em_Message_addByte(b)) {
        Em_Message_dispatch();
    }
    EAP_RX_ACK_CLR();
    EAP_RX_ACK_SET();
}

#pragma vector=EAP_TX_VECTOR
__interrupt void txHandler(void) {
    if (EAP_TX_INT_TST()) {
        uint8_t b;
        if (Em_Message_getByte(&b)) {
            EAP_TX_BUF = b;
        }
        EAP_TX_INT_CLR();
    }
}

void Em_Message_startSend() {
    uint8_t b;
    if (Em_Message_getByte(&b)) {
        UCA0TXBUF = b;
    }
}

uint8_t Em_Message_lock() {
	uint8_t state = _get_interrupt_state();
	_disable_interrupt();
	return state;
}

void Em_Message_unlock(uint8_t key) {
	_set_interrupt_state(key);
}

/*
 * ============ Application Program ============
 */

#include <HeightSensorBT_v1.h>

#define COUNT_DEFAULT 5

//volatile HeightSensorBT_v1_cmd_t cmdRes = HeightSensorBT_v1_STOP_CMD;


int main(int argc, char *argv[]) {
    volatile int dummy = 0;
    init();
    HeightSensorBT_v1_run();
    while (dummy == 0) {
        /* idle */
    }
    return 0;
}

void HeightSensorBT_v1_connectHandler(void) {
    CONNECTED_LED_ON();
}

void HeightSensorBT_v1_disconnectHandler(void) {
    CONNECTED_LED_OFF();
}

int getDistance(void) {
	
	int ADC_Conversion_Result;
    // Get the height
    ADC10CTL0 |= ENC + ADC10SC;
    while ((ADC10CTL0 & ADC10IFG) == 0);
    ADC_Conversion_Result = ADC10MEM;
    ADC_Conversion_Result /= 2;

	return ADC_Conversion_Result;
}

void HeightSensorBT_v1_distance_fetch(HeightSensorBT_v1_distance_t* const output) {
    /* TODO: write resource 'distance' into 'output' */

	int dist = getDistance();
	*output = dist;
}

void HeightSensorBT_v1_led2_store(HeightSensorBT_v1_led2_t* const input) {
    /* TODO: read resource 'led2' from 'input' */

	P1DIR |= BIT6;

	if (*input == HeightSensorBT_v1_LED2_ON)
	{
		P1OUT |= BIT6;
	}
	if (*input == HeightSensorBT_v1_LED2_OFF)
	{
		P1OUT &= ~BIT6;
	}
}

void HeightSensorBT_v1_currentBlock_store(HeightSensorBT_v1_currentBlock_t* const input) {
    /* TODO: read resource 'currentBlock' from 'input' */
	block = *input;

	switch(block)
	{
	case 0:
		WRITE = 0xA0;
		READ = 0xA1;
		addrRef = 0;
		break;
	case 1:
			WRITE = 0xA2;
			READ = 0xA3;
			//addrRef = 256;
			break;
	case 2:
			WRITE = 0xA4;
			READ = 0xA5;
			//addrRef = 512;
			break;
	case 3:
			WRITE = 0xA6;
			READ = 0xA7;
			//addrRef = 768;
			break;
	case 4:
			WRITE = 0xA8;
			READ = 0xA9;
			//addrRef = 1024;
			break;
	case 5:
				WRITE = 0xAA;
				READ = 0xAB;
				//addrRef = 1280;
				break;
	case 6:
				WRITE = 0xAC;
				READ = 0xAD;
				//addrRef = 1536;
				break;
	case 7:
				WRITE = 0xAE;
				READ = 0xAF;
				//addrRef = 1792;
				break;
	}

}

void HeightSensorBT_v1_blockHalf_store(HeightSensorBT_v1_blockHalf_t* const input) {
    /* TODO: read resource 'blockHalf' from 'input' */

	if (*input == HeightSensorBT_v1_Second_Half)
	{
		//addrRef += 127;		// Add 128 to the address to write to the second half of the current block
	}
}


void HeightSensorBT_v1_cardContents_fetch(HeightSensorBT_v1_cardContents_t* const output) {
    /* TODO: write resource 'cardContents' into 'output' */
	P2OUT  |= SCL;
	P2DIR |= SCL;

	readString(127);

	P1DIR |= BIT6;
	P1OUT |= BIT6;
	//CONNECTED_LED_ON();
	memcpy((void *)output, theReadString, sizeof(theReadString));
	//CONNECTED_LED_OFF();
	P1OUT &= ~BIT6;
}

void HeightSensorBT_v1_cardContents_store(HeightSensorBT_v1_cardContents_t* const input) {
    /* TODO: read resource 'cardContents' from 'input' */
	P2OUT  |= SCL;
	P2DIR |= SCL;

	P1DIR |= BIT6;
	P1OUT |= BIT6;
	//CONNECTED_LED_ON();
	string = (unsigned char *)input;
	writeString(string, 127);
	// Changed to sizeof
	//writeString(string, sizeof(string));
	//CONNECTED_LED_OFF();
	P1OUT &= ~BIT6;
}

void writeString(unsigned char * theString, int sizeInBytes)
{
	int i;

		for(i = sizeInBytes; i > 0; i--)
		{
			address = i + addrRef; // set address to i
			charData = theString[i-1];
			writeChar(); // write char to address
			_delay_cycles(10000);
		}
}

// Function to read a string of a specified number of bytes
void readString(int sizeInBytes)
{
	int i, flag;
		for(i = sizeInBytes; i > 0; i--)
		{
			// TESTING
			address = i + addrRef;
			flag = readChar();
			if(flag == FAILURE)
			{
			   P1DIR |= BIT0;
			   P1OUT |= BIT0;
			}
			else
			{
			   P1DIR = BIT6;
			   P1OUT = BIT6;
			}
			theReadString[i-1] = rxData;
			_delay_cycles(10000);
		}
}

// Reads a byte from the card. Address must be set beforehand
// 1-> Send start
// 2-> Send device address with RW bit = 0
// 3-> Recieve ack
// 4-> Send byte address to read
// 5-> Receive ack
// 6-> Re-send start
// 7-> Send device address with RW bit = 1
// 8-> Receive ack
// 9-> Receive byte
// 10-> Send stop
int readChar(void) {
    start();
    txData = WRITE;
    sendByte();
    receiveAck();
    if(!ackFlag)
        return FAILURE;
    txData = address;
    sendByte();
    receiveAck();
    start();
    txData = READ;
    sendByte();
    receiveAck();
    if(!ackFlag)
        return FAILURE;
    receiveByte();
    ackFlag = 0;
    sendAck();
    charData = rxData;
    stop();
    return SUCCESS;
}

// Writes a byte to the card. Address must be set beforehand
// 1-> Send start
// 2-> Send device address with RW bit = 0
// 3-> Recieve ack
// 4-> Send byte address to write
// 5-> Receive ack
// 6-> Send byte
// 7-> Receive ack
// 8-> Send stop
int writeChar(void) {
    start();
    txData = WRITE;
    sendByte();
    receiveAck();
    if(!ackFlag)
        return FAILURE;
    txData = address;
    sendByte();
    receiveAck();
    if(!ackFlag)
        return FAILURE;
    txData = charData;
    sendByte();
    receiveAck();
    if(!ackFlag)
        return FAILURE;
    stop();
    return SUCCESS;
}


// send byte to slave
void sendByte(void) {
    P2DIR |= SDA;
    bitCounter = 0;
    while(bitCounter < 8) {
        (txData & BIT7) ? (P2OUT |= SDA) : (P2OUT &= ~SDA);
        P2OUT |= SCL;
        txData <<= 1;
        bitCounter++;
        P2OUT &= ~SCL;
    }
    P2OUT |= SDA;
    P2DIR &= ~SDA;
}

// receive byte from slave
void receiveByte(void) {
    bitCounter = 0;
    while(bitCounter < 8) {
        P2OUT |= SCL;
        rxData <<= 1;
        bitCounter++;
        if(P2IN & SDA) {
            rxData |= BIT0;
        }
        P2OUT &= ~SCL;
    }
}

// send master's ACK
void sendAck(void) {
    P2DIR |= SDA;
    (ackFlag) ? (P2OUT &= ~SDA) : (P2OUT |= SDA);
    P2OUT |= SCL;
    P2OUT &= ~SCL;
    P2OUT |= SDA;
    P2DIR &= ~SDA;
}

// receive slave's ACK
void receiveAck(void) {
    P2OUT |= SCL;
    (P2IN & SDA) ? (ackFlag = 0) : (ackFlag = 1);
    P2OUT &= ~SCL;
}

// start condition
void start(void) {
    P2OUT |= SCL;
    P2DIR |= SDA;
    P2OUT &= ~SDA;
    P2OUT &= ~SCL;
    P2OUT |= SDA;
    P2DIR &= ~SDA;

}

// stop condition
void stop(void) {
    P2DIR |= SDA;
    P2OUT &= ~SDA;
    P2OUT |= SCL;
    P2OUT |= SDA;
    P2DIR &= ~SDA;
}
