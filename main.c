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

void init(void) {

    WDTCTL = WDTPW + WDTHOLD;
    BCSCTL2 = SELM_0 + DIVM_0 + DIVS_0;
    if (CALBC1_1MHZ != 0xFF) {
        DCOCTL = 0x00;
        BCSCTL1 = CALBC1_1MHZ;      /* Set DCO to 1MHz */
        DCOCTL = CALDCO_1MHZ;
    }
    BCSCTL1 |= XT2OFF + DIVA_0;
    BCSCTL3 = XT2S_0 + LFXT1S_2 + XCAP_1;

    P1DIR |= BIT0;  /* LED */
    P1DIR |= BIT6;  /* CONNECTED_LED */
    CONNECTED_LED_OFF();

    UCA0CTL1 |= UCSWRST;

    P1SEL |= BIT1 + BIT2;
    P1SEL2 |= BIT1 + BIT2;

    EAP_RX_ACK_CONFIG();
    EAP_RX_ACK_SET();

    EAP_TX_INT_CONFIG();

    UCA0CTL1 = UCSSEL_2 + UCSWRST;
    UCA0MCTL = UCBRF_0 + UCBRS_6;
    UCA0BR0 = 8;
    UCA0CTL1 &= ~UCSWRST;

    IFG2 &= ~(UCA0RXIFG);
    IE2 |= UCA0RXIE;

    ////////////////////////////////////////////////////
    ////// INIT ADC FOR SENSOR //////////////////

    P1DIR &= ~BIT7;  	//Set P1.7 as input

    ADC10CTL0 &= ~ENC;

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

