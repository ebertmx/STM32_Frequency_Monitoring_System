/*
 *  Created on: Nov. 21, 2022
 *      Author: Matthew Ebert; V00884117
 * 		For: ECE 355; University of Victoria
 * 		Modified: Nov. 25, 2022
 * 		
 * Description: This file contians functions to configure and use the SPI hardware on the 
 * STM32F0 Discovery board. Additional function are included to write specific patterns
 * which control the LCD display on the PBMCUSLK through a 74HC595 shift register.
 * 
 */

#include "main.h"

SPI_HandleTypeDef SPI_Handle; //Handle for configured SPI

volatile uint16_t delay_target = 0;



void LCD_Setup(){

	//Reset the LCD
	//This code replaces power cycling the LCD
	sendNibble(0x03);
	delay_ms(10);
	sendNibble(0x03);
	delay_ms(10);
	sendNibble(0x03);
	//Configure the LCD
	sendNibble(0x02);//set up for 4bit interface
	sendByte(0x28,0);//two line mode
	sendByte(0x0C,0);//Display on
	sendByte(0x06,0);//auto increment DDRAM
	sendByte(0x01,0);//clear display
}

/*
Description: Sends an 8bit integer up to 4 digits to the
LCD in as displayed in the format F:xxxxHz
*/
void sendFreq(unsigned int freq){
	char frq[8];
	//convert int to char
	sprintf(frq,"F:%4uHz", freq);

	//send to LCD on the first line
	sendLine(frq, 8, 0);
}


/*
Description: Sends an 8bit integer up to 4 digits to the
LCD in as displayed in the format R:xxxxOh
*/
void sendADC(unsigned int adcvalue){
	char adc[8];
	//convert int to char
	sprintf(adc,"R:%4uOh", adcvalue);
	//send to LCD on the second line
	sendLine(adc, 8, 1);
}



/*
Description: Sends a number of characters to a specific 
position on the LCD
*/
int sendAtPos (char *c, uint8_t len, uint8_t x, uint8_t y){
	y = (y<<6) | 0x80;
	uint8_t pos = y|x;
	sendByte(pos,0);//set up DDRAM

	for (int i = 0; i<len; i++){
			sendByte(c[i],1);
		}
		return 1;
}

/*
Description: Sends a number of characters to a specific 
line on the LCD
*/
int sendLine (char *c, uint8_t len, uint8_t line){
	line = (line<<6) | 0x80;
	sendByte(line,0);//set up DDRAM

	for (int i = 0; i<len; i++){
		//trace_printf("%c", c[i]);
		sendByte(c[i],1);
	}
	return 1;
}


/*
Description: Sends a complete byte to the LCD by sending 
the upper and lower nibbles
*/
void sendByte (uint8_t B, uint8_t RS){
	RS = RS<<6;
	uint8_t H = (B>>4) | RS;
	uint8_t L = (0x0F & B) | RS;

	//send Upper nibble
	sendNibble(H);
	send lower nibble
	sendNibble(L);
}

/*
Description: Sends a nibble to the LCD with by writing a
sequence of bits which creates a pulse on the EN bit
*/
void sendNibble (uint8_t nib){
		//trace_printf("sending: %d\n", *txbuff);
		uint8_t sequence [3];
		sequence[0] = 0x00 | nib;
		sequence[1] = 0x80 | nib;
		sequence[2] = 0x00 | nib;
	for(int i = 0; i<3;i++){
		writeByte(&sequence[i]);
	}
	//	trace_printf("sent\n");
}


/*
Description: writes a bytes through the SPI interface and sends it
to the LCD by pulsing the RCLK input (PB4)
*/
void writeByte(uint8_t *txbuff){
	while(!__HAL_SPI_GET_FLAG(&SPI_Handle, SPI_FLAG_TXE) ){}

	if(HAL_SPI_Transmit(&SPI_Handle, txbuff , 1 , HAL_MAX_DELAY) !=HAL_OK){
		trace_printf("did not send...\n");
	}
	while(!__HAL_SPI_GET_FLAG(&SPI_Handle, SPI_FLAG_TXE) ){}

	GPIOB->BSRR = lck_msk;
	delay_ms(2);//delay for LCD to process
	GPIOB->BRR = lck_msk;
	//trace_printf("sent: %d", *txbuff);
}


/*
Description: Configures the SPI interface
*/
void myLCD_Init(void){

	myTIM3_Init();


	//Enable clock for SPI1
	RCC->APB2ENR|= RCC_APB2ENR_SPI1EN;
	//enable clock for GPIOB
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	//Configure PB3 and PB5 for SCLK and MOSI
	//Set as alternate function mode
	GPIOB->MODER |= GPIO_MODER_MODER3_1;
	GPIOB->MODER |= GPIO_MODER_MODER5_1;

	//Configure SPI handle
	//use SPI
	SPI_Handle.Instance = SPI1;
	//Output only
	SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;
	//set as master
	SPI_Handle.Init.Mode = SPI_MODE_MASTER;
	//8 bit transmit data size
	SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
	//set polarity low
	SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
	//set clock phase
	SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI_Handle.Init.NSS = SPI_NSS_SOFT;
	//set to lowest baud rate
	SPI_Handle.Init.BaudRatePrescaler =SPI_BAUDRATEPRESCALER_256 ;
	//set bit order
	SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_Handle.Init.CRCPolynomial=7;

	//Initalize SPI with configuration
	HAL_SPI_MspInit(&SPI_Handle);
	if( HAL_SPI_Init(&SPI_Handle) !=HAL_OK){
		trace_printf("FAIL");
	}
	__HAL_SPI_ENABLE(&SPI_Handle);

	myGPIOB_Init();//setup RCLK pin
}


/*
Description: Set up PB4 as an output pin used for RCLK
*/
void myGPIOB_Init(){
	GPIOB->MODER |= (GPIO_MODER_MODER4_0);
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_4);
	/* Ensure high-speed mode for PC8 and PC9 */
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4);
	/* Ensure no pull-up/pull-down for PC8 and PC9 */
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
	GPIOB->BSRR = lck_msk;
}


/*
Description: Delays a number of ms using TIM3
*/
void delay_ms(int delay){

	delay_target = 0;
	TIM3->CR1 |= TIM_CR1_CEN;//start timer
	while (delay_target < delay);//wait for delay
	TIM3->CR1 &= ~(TIM_CR1_CEN);//stop timer

}


/*
Description: Used exclusively for the delay_ms function
*/
void myTIM3_Init()
{
	/* Enable clock for TIM3 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/* Configure TIM3: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM3->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM3->PSC = myTIM3_PRESCALER;
	/* Set auto-reloaded delay */
	TIM3->ARR = myTIM3_PERIOD;

	/* Update timer registers */
	TIM3->EGR = ((uint16_t)0x0001);

	/* Assign TIM3 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM3_IRQn, 0);
	// Same as: NVIC->IP[3] = ((uint32_t)0x00FFFFFF);

	/* Enable TIM3 interrupts in NVIC */
	NVIC_EnableIRQ(TIM3_IRQn);
	// Same as: NVIC->ISER[0] = ((uint32_t)0x00008000) */

	/* Enable update interrupt generation */
	TIM3->DIER |= TIM_DIER_UIE;
	/* Start counting timer pulses */
	TIM3->CR1 |= TIM_CR1_CEN;
}


/*
Description: Used exclusively for the delay_ms function.
when TIM3 is running this is run every 1 ms.
*/
void TIM3_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM3->SR & TIM_SR_UIF) != 0)
	{
		/* Read current PC output and isolate PC8 and PC9 bits */
		delay_target++;
		//trace_printf("%d",delay_target);
		TIM3->SR &= ~(TIM_SR_UIF);	/* Clear update interrupt flag */
		TIM3->CR1 |= TIM_CR1_CEN;	/* Restart stopped timer */
	}

}


//debugging
void printBin (uint8_t num){
	trace_printf("bin = ");
	for(int i = 0; i<8;i++){
		trace_printf("%ud", num>>(7-i));
	}
	trace_printf("bin = ");
}



