#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>

#include "stm32f4_discovery.h"
#include "misc.h"  //NVIC & Such

//Need to shift because I2C library just masks with 0xFE
//and top 7-bits are used as address
#define MY_I2C_ADDRESS    0x1 << 1  

//LED code from:
//https://github.com/Malkavian/tuts/tree/master/stm/blinky
#define GREEN  LED4_PIN
#define ORANGE LED3_PIN
#define RED    LED5_PIN
#define BLUE   LED6_PIN
#define ALL_LEDS (GREEN | ORANGE | RED | BLUE) // all leds
#define LEDS_GPIO_PORT (GPIOD)
static void setup_leds(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
    /* Enable the GPIOD peripheral clock before we
     * actually setup GPIOD.
     * This function is declared in stm32f4xx_rcc.h
     * and implemented in stm32f4xx_rcc.c
     */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* which pins we select to setup
     * every pin number is mapped to a bit number */
    GPIO_InitStructure.GPIO_Pin   = ALL_LEDS;
    /* pins in output mode */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    /* high clock speed for the selected pins
     * see stm32f4xx_gpio.h for different speeds
     * (enum GPIOSpeed_TypeDef) */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    /* operating output type for the selected pins
     * see the enum GPIOOType_TypeDef in stm32f4xx_gpio.h
     * for different values.
     * PP stands for "push/pull", OD stands for "open drain"
     * google for "push pull vs open drain" */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    /* operating Pull-up/Pull down for the selected pins */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    /* Write this data into memory at the address
     * mapped to GPIO device port D, where the led pins
     * are connected */
    GPIO_Init(LEDS_GPIO_PORT, &GPIO_InitStructure);
    /* This call resolves in
     * GPIO_Init((GPIO_TypeDef *) 0X40020C00, &GPIO_InitStructure)
     * where 0X40020C00 is the memory address mapped to
     * the GPIOD port. Without the library we would have to know all
     * these memory addresses. */
}

//Initialization and general help with STM32 I2C from:
//https://github.com/devthrash/STM32F4-examples/tree/master/I2C%20Master
void init_I2C1(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* setup SCL and SDA pins
	 * You can connect I2C1 to two different
	 * pairs of pins:
	 * 1. SCL on PB6 and SDA on PB7 
	 * 2. SCL on PB8 and SDA on PB9
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB
	
	// Connect I2C1 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
	
	// configure I2C1 
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = MY_I2C_ADDRESS;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1

	//setup interrupts
	uint16_t i2c_int_flags = 
		I2C_IT_ERR | 
		I2C_IT_EVT | 
		I2C_IT_BUF;	
	I2C_ITConfig(I2C1, i2c_int_flags, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	// Configure the I2C event priority
	NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

int main(void){
	uint8_t received_data[2];
	int i;
	uint32_t curStatus = 0;
	uint8_t data;
	//Initializtion
	setup_leds();
	init_I2C1();
	//Busy Loop
	while(1){
        GPIO_ToggleBits(LEDS_GPIO_PORT, BLUE); //Do something
		for(i = 0; i < 1000000; ++i); //Really terrible delay function
	}
}

//Clear ADDR by reading SR1, then SR2
void I2C_clear_ADDR(I2C_TypeDef* I2Cx) {
	I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR);
	((void)(I2Cx->SR2));
}

//Clear STOPF by reading SR1, then writing CR1
void I2C_clear_STOPF(I2C_TypeDef* I2Cx) {
	I2C_GetFlagStatus(I2Cx, I2C_FLAG_STOPF);
	I2C_Cmd(I2Cx, ENABLE);
}

uint8_t data = 0;
void I2C1_EV_IRQHandler(void) {
        GPIO_SetBits(GPIOD, GREEN); //Show that we got here
		//Clear AF from slave-transmission end
		if(I2C_GetITStatus(I2C1, I2C_IT_AF)) {
			I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
		}
		//Big state machine response, since doesn't actually keep state
		switch(I2C_GetLastEvent(I2C1)) {
			//SLAVE
			//Receive
			case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED: //EV1
				I2C_clear_ADDR(I2C1);
				break;
			case I2C_EVENT_SLAVE_BYTE_RECEIVED: //EV2
				//Read it, so no one is waiting, clears BTF if necessary
				data = I2C_ReceiveData(I2C1);
				//Do something with it
				if(I2C_GetFlagStatus(I2C1, I2C_FLAG_DUALF)) {//Secondary Receive
				} else if(I2C_GetFlagStatus(I2C1, I2C_FLAG_GENCALL)) {//General Receive
				} else {//Normal
				}
				break;
			case I2C_EVENT_SLAVE_STOP_DETECTED: //End of receive, EV4
				I2C_clear_STOPF(I2C1);
				break;

			//Transmit
			case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED: //EV1
				I2C_clear_ADDR(I2C1);
				//Send first byte
				I2C_SendData(I2C1, ++data);
				break;
			case I2C_EVENT_SLAVE_BYTE_TRANSMITTED: //EV3
				//Determine what you want to send
				//data = 5;
				if(I2C_GetFlagStatus(I2C1, I2C_FLAG_DUALF)) {//Secondary Transmit
				} else if(I2C_GetFlagStatus(I2C1, I2C_FLAG_GENCALL)) {//General Transmit
				} else {//Normal
				}
				//Read flag and write next byte to clear BTF if present
				I2C_GetFlagStatus(I2C1, I2C_FLAG_BTF);
				I2C_SendData(I2C1, ++data);
				break;
			case I2C_EVENT_SLAVE_ACK_FAILURE://End of transmission EV3_2
				//TODO: Doesn't seem to be getting reached, so just
				//check at top-level
				I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
				break;
			//Alternative Cases for address match
			case I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED: //EV1
				break;
			case I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED: //EV1
				break;
			case I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED: //EV1
				break;


			//MASTER
			case I2C_EVENT_MASTER_MODE_SELECT: //EV5, just sent start bit
				break;
			//Receive
			case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED: //EV6, just sent addr    
				break;
			case I2C_EVENT_MASTER_BYTE_RECEIVED: //EV7
				break;
			//Transmit
			case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED: //EV6, just sent addr     
				break;
			case I2C_EVENT_MASTER_BYTE_TRANSMITTING: //EV8, about to send data
				break;
			case I2C_EVENT_MASTER_BYTE_TRANSMITTED: //EV8_2, just sent data
				break;

			//Alternative addressing stuff, not going to worry about
			case I2C_EVENT_MASTER_MODE_ADDRESS10: //EV9
				break;
			default:
				//How the FUCK did you get here?
				//I should probably raise some error, but fuck it,
				//it's late
				break;
		}
}

void I2C1_ER_IRQHandler(void) {
        GPIO_SetBits(GPIOD, RED);
		//Can't use nice switch statement, because no fxn available
		if(I2C_GetITStatus(I2C1,        I2C_IT_SMBALERT)) {
		} else if(I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT)) {
		} else if(I2C_GetITStatus(I2C1, I2C_IT_PECERR)) {
		} else if(I2C_GetITStatus(I2C1, I2C_IT_OVR)) {
			//Overrun
			//CLK stretch disabled and receiving
			//DR has not been read, b4 next byte comes in
			//effect: lose byte
			//should:clear RxNE and transmitter should retransmit

			//Underrun
			//CLK stretch disabled and I2C transmitting
			//haven't updated DR since new clock
			//effect: same byte resent
			//should: make sure discarded, and write next
		} else if(I2C_GetITStatus(I2C1, I2C_IT_AF)) {
			//Detected NACK
			//Transmitter must reset com
				//Slave: lines released
				//Master: Stop or repeated Start must must be generated
				//Master = MSL bit
			//Fixup
			I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
		} else if(I2C_GetITStatus(I2C1, I2C_IT_ARLO)) {
			//Arbitration Lost
			//Goes to slave mode, but can't ack slave address in same transfer
			//Can after repeat Start though
		} else if(I2C_GetITStatus(I2C1, I2C_IT_BERR)) {
			//Bus Error
			//In slave mode: data discarded, lines released, acts like restart
			//In master mode: current transmission continues
		}
}

