
/*
 *  Created on: Nov. 21, 2022
 *      Author: Matthew Ebert; V00884117
 * 		For: ECE 355; University of Victoria
 * 		Modified: Nov. 25, 2022
 * 		
 * Description: This program completes 4 tasks using an STM32F0 Discovery board.
 * 1. Read an analog voltage from a potentiometer
 * 2. Output an analog voltage via the onboard DAC
 * 3. Calculate the frequency of an input sqaurewave
 * 4. Display the frequency of the input wave and the resistance of the potentiometer
 * 		on an LCD via a SPI interface
 * 
 * This file requires the source LCD.c and main.h to run. It also uses the HAL SPI library.
 */

#include "main.h"


volatile int timerTriggered = 0;// flag to indicate first or second rising edge
const unsigned int clockFrequency = 48000000; //board clock frequency
volatile char WRITEFLAG = 0;//flag to indicate if an LCD write is in progress
volatile int frequency=0;//the calculated frequency
volatile unsigned int signalPeriod=0;//the calculated period

int main(int argc, char* argv[])
{

	SystemClock48MHz();//Set the board clock to 48MHz
	trace_puts("Hello World!");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);


	myGPIOA_Init();//enable clock for GPIOA pins
	myGPIOC_Init();//not used

	myDAC_Init();
	myADC_Init();
	myLCD_Init(); //Set up SPI interface
	LCD_Setup(); //Reset and configure the LCD


	myEXTI_Init(); //Configure and Start EXTI on PA1
	myTIM2_Init(); //Configure onboard timer for frequency timing

	uint16_t valueADC = 0; //store last read ADC value
	float Rvalue = 0; //stores calculated resistance value
	uint16_t Res = 0; //stores Rvalue as integer
	//int count = 0;//debuging
	while(1){
		//count ++;

		//wait for adc to finish conversion
		while((ADC1->ISR & ADC_ISR_EOC) ==0){
			continue;
		}
		//save converted value from ADC
		valueADC = ADC1->DR;

		//set DAC 
		DAC->DHR12R1 = valueADC;
		//calculate resistance
		Rvalue = ((float)valueADC)*(1.221);
		Res = Rvalue;//convert to uint16_t

		/*this delay gives the interrupt time to complete frequency 
		calculations if the frequency is very low (<100Hz)*/
		delay_ms(10);

		//if new frequency available
		if(WRITEFLAG){

			sendFreq(frequency);//write frequency to LCD
			sendADC(Res);//write resistance to LCD
			delay_ms(100);//delay for LCD to set refresh rate
			WRITEFLAG = 0;//indicate finished writing
			EXTI->IMR |= (1<<1);//re-enable interrupt on PA1

		}
	}
}



/*
DESCRIPTION: initialized the ADC on PA5
*/
void myADC_Init(){

	//Calibrate ADC
	//configure PA5 as Analog
	GPIOA->MODER |= (GPIO_MODER_MODER5_0 | GPIO_MODER_MODER5_1);
	//Clear ADC_CR
	ADC1->CR = 0x00000000;
	//enable clock for ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	//enable ADC
	ADC1->CR |= ADC_CR_ADEN;
	//sample rate
	ADC1->SMPR |= ADC_SMPR_SMP_0|ADC_SMPR_SMP_1|ADC_SMPR_SMP_2;
	//Channel Select
	ADC1->CHSELR |= ADC_CHSELR_CHSEL5;
	//Configure Resolution
	ADC1->CFGR1 &= ~(ADC_CFGR1_RES_0|ADC_CFGR1_RES_1);
	//Configure Alignment
	ADC1->CFGR1 &= ~(ADC_CFGR1_ALIGN);
	//Configure overrun Management
	ADC1->CFGR1 |= (ADC_CFGR1_OVRMOD);
	//Enable continuous conversion
	ADC1->CFGR1 |= (ADC_CFGR1_CONT);

	//wait for adc to be ready
	while(((ADC1->ISR)& ADC_ISR_ADRDY)==0){
		continue;
	}
	trace_printf("ADC Configured Successfully\n");
	//start ADC
	ADC1->CR |= ADC_CR_ADSTART;
}


/*
DESCRIPTION: initialized the DAC on PA4
*/
void myDAC_Init(){
	//configure PA4 as Analog
	GPIOA->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER4_1);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
	//enable DAC clock
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	//enable DAC
	DAC->CR |= DAC_CR_EN1;
	//Disable high impedence
	DAC->CR &= ~(DAC_CR_BOFF1);

	//trace_printf("DAC Configured Successfully\n");

}
/*
DESCRIPTION: Enables clock for GPIOA pins
*/
void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

}


/*
DESCRIPTION: Initilizes the onboard timer used for frequency calculations
*/
void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = ((uint16_t)0x008C);
	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;
	/* Update timer registers */
	TIM2->EGR = ((uint16_t)0x0001);
	/* Assign TIM2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn, 0);
	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);
	/* Enable update interrupt generation */
	TIM2->DIER |= TIM_DIER_UIE;


}


void myEXTI_Init()
{
	/* Map EXTI2 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= 0x0000FF0F;
	/* EXTI2 line interrupts: set rising-edge trigger */
	EXTI->RTSR |= (1<<1);

	/* Unmask interrupts from EXTI2 line */
	EXTI->IMR |= (1<<1);

	/* Assign EXTI2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI2 interrupts in NVIC */
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}


/*
DESCRIPTION: Runs everytime the TIM2 counter overflows
*/
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}

}


/*
DESCRIPTION: Runs everytime a rising edge appears on PA1
*/
void EXTI0_1_IRQHandler()
{
	// Declare/initialize your local variables here...

	/* Check if EXTI2 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//
		// 1. If this is the first edge:
		if (timerTriggered == 0){
			//Clear count register (TIM2->CNT).
			TIM2->CNT = 0;
			//Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;
			timerTriggered = 1;
		}else{		//    Else (this is the second edge):

			//Stop timer (TIM2->CR1).
			TIM2->CR1 &= ~(TIM_CR1_CEN);
			//	- Calculate signal period and frequency.

			if(!WRITEFLAG){
				if(TIM2->CNT<clockFrequency){
				//calculate values
					frequency = clockFrequency / TIM2->CNT;
					signalPeriod=TIM2->CNT / 48;//scaled by 1000000

				}
				if(TIM2->CNT>clockFrequency){
				//calculate values
					unsigned int temp = TIM2->CNT/1000000;
					frequency = 4800 / temp;//scaled by 100000
					signalPeriod = TIM2->CNT / clockFrequency;

				}
				WRITEFLAG = 1;//Indicate frequency is ready to write

				//disable interrupt to save processing time and wait for
				//LCD write.
				EXTI->IMR &= ~(1<<1);

			}
			//reset edge flag
			timerTriggered = 0;
		}
		//reset interrupt pending flag
		EXTI->PR |= EXTI_PR_PR1;
		//
	}
}




/*
DESCRIPTION: Not used
*/
void myGPIOC_Init()
{
	/* Enable clock for GPIOC peripheral */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	/* Configure PC8 and PC9 as outputs */
	GPIOC->MODER |= (GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0);
	/* Ensure push-pull mode selected for PC8 and PC9 */
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	/* Ensure high-speed mode for PC8 and PC9 */
	GPIOC->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9);
	/* Ensure no pull-up/pull-down for PC8 and PC9 */
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR8 | GPIO_PUPDR_PUPDR9);
}




void SystemClock48MHz( void )
{
//
// Disable the PLL
//
    RCC->CR &= ~(RCC_CR_PLLON);
//
// Wait for the PLL to unlock
//
    while (( RCC->CR & RCC_CR_PLLRDY ) != 0 );
//
// Configure the PLL for a 48MHz system clock
//
    RCC->CFGR = 0x00280000;

//
// Enable the PLL
//
    RCC->CR |= RCC_CR_PLLON;

//
// Wait for the PLL to lock
//
    while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );

//
// Switch the processor to the PLL clock source
//
    RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;

//
// Update the system with the new clock frequency
//
    SystemCoreClockUpdate();

}



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
