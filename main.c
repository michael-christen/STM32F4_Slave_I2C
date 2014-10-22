#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>

#include "stm32f4_discovery.h"
#include "misc.h"  //NVIC & Such

#define SLAVE_ADDRESS 0x4 // the slave address (example)
#define MY_I2C_ADDRESS    0x0

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

	/*
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
	*/
	
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
		
	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	  
	/* wait for I2C1 EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

uint32_t I2C_analyzeState(I2C_TypeDef* I2Cx) {
	uint32_t i2c_status = I2C_GetLastEvent(I2Cx);
	int numFlags = 0;
	//Large structure to do specific thing for each flag
	if(i2c_status  & I2C_FLAG_DUALF) {
		numFlags++;
	}if(i2c_status & I2C_FLAG_SMBHOST) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_SMBDEFAULT) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_GENCALL) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_TRA) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_BUSY) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_MSL) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_SMBALERT) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_TIMEOUT) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_PECERR) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_OVR) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_AF) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_ARLO) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_BERR) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_TXE) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_RXNE) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_STOPF) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_ADD10) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_BTF) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_ADDR) {

		numFlags++;
	}if(i2c_status & I2C_FLAG_SB) {

		numFlags++;
	}
	return i2c_status;
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1 
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition 
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

int main(void){
	
	setup_leds();
	init_I2C1(); // initialize I2C peripheral
	
	uint8_t received_data[2];
	int i;
	uint32_t curStatus = 0;
	uint8_t data;
	
	while(1){
        GPIO_ToggleBits(LEDS_GPIO_PORT, BLUE);
		/*
		while(!I2C_CheckEvent(I2C1, 
					I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED) 
				&&
				!I2C_CheckEvent(I2C1, 
					I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED)) {
			curStatus = I2C_analyzeState(I2C1);
		}
		*/
		//Clear ADDR flag, by reading s1 and s2
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR));
		((void)(I2C1->SR2));

		//Until get stop bit
		//while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF)) {

		//Read, implicitly clears BTF flag
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE));
		data = I2C_ReceiveData(I2C1);

		//}
		//Clear stop bit
		while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
		I2C_Cmd(I2C1, ENABLE);


		//I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
		//I2C_write(I2C1, 'a'); // write one byte to the slave
		//I2C_write(I2C1, 'b'); // write another byte to the slave
		//I2C_stop(I2C1); // stop the transmission
		
		//I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
		//received_data[0] = I2C_read_ack(I2C1); // read one byte and request another byte
		//received_data[1] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
		//I2C_stop(I2C1); // stop the transmission
		//for(i = 0; i < 1000000; ++i);
	}
}

void I2C1_EV_IRQHandler(void) {
        GPIO_SetBits(GPIOD, GREEN);
}

void I2C1_ER_IRQHandler(void) {
        GPIO_SetBits(GPIOD, RED);
}

