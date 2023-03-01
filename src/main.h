
/*
 *  Created on: Nov. 21, 2022
 *      Author: Matthew Ebert; V00884117
 * 		For: ECE 355; University of Victoria
 * 		Modified: Nov. 25, 2022
 * 		
 * Description: Header file for main.c and LDC.c for ECE355 project
 * 

 */


#ifndef MAIN_H_
#define MAIN_H_

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
#include "stm32f0xx_hal_spi.h"


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)


/* Clock prescaler for TIM3 timer: no prescaling */
#define myTIM3_PRESCALER ((uint16_t)0x0000)
/* Delay count for TIM3 timer: 1 ms at 48 MHz */
#define myTIM3_PERIOD ((uint32_t)48000)
//msk for LCK pin for LCD

#define lck_msk 0x0010;

void SystemClock48MHz( void );


void myGPIOA_Init(void);
void myGPIOC_Init(void);


void myGPIOB_Init();
void myTIM3_Init(void);
void myLCD_Init(void);
void delay_ms(int delay);

void sendByte (uint8_t B, uint8_t RS);
void LCD_Setup();
void writeByte(uint8_t * txbuff);
void sendNibble(uint8_t nib);
void printBin (uint8_t num);
int sendLine (char *c, uint8_t len, uint8_t line);
int sendAtPos (char *c, uint8_t len, uint8_t x, uint8_t y);
void sendFreq(unsigned int freq);
void sendADC(unsigned int adc);

void myADC_Init();
void myDAC_Init();


void myTIM2_Init();
void myEXTI_Init();

void TIM2_IRQHandler();
void EXTI0_1_IRQHandler();


#endif /* MAIN_H_ */
