//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: CENG 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)


void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void mySPI1_Init(void);
void myLCD_Init(void);
void mySPI1_SendData(unsigned char data);
void myLCD_SendInstruction(unsigned char data);
void myLCD_SendChar(unsigned char data);
void wait(uint32_t);
void myDAC_ADC_Init(void);

// Your global variables...
static float clock_Frequency = 48000000;

unsigned char setRise =0;
int count=0;
float frequency =0;
float resistance=0;
unsigned char data= 0x30;
int main(int argc, char* argv[])
{

	//trace_printf("This is a Lab Project Test\n");
	//trace_printf("System clock: %u Hz\n", SystemCoreClock);

	wait(100);//wait so trace print doesn't interfere

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		/* Initialize I/O port PB */
	myDAC_ADC_Init();		/* Initialize DAC and ADC */
	wait(5);
	mySPI1_Init();		/* Initialize SPI1 */
	myLCD_Init();		/*Initialize LCD Display*/
	wait(5);

//Display Something
	//Set address
	myLCD_SendInstruction(0x80);
	wait(4);
	myLCD_SendChar('F');
	wait(4);
	myLCD_SendChar(':');
	wait(4);
	myLCD_SendChar('0');
	wait(4);
	myLCD_SendChar('0');
	wait(4);
	myLCD_SendChar('0');
	wait(4);
	myLCD_SendChar('0');
	wait(4);
	myLCD_SendChar('H');
	wait(4);
	myLCD_SendChar('z');
	wait(4);

	myTIM2_Init();	  /* Initialize timer TIM2 */
	myEXTI_Init();	  /* Initialize EXTI */

	while(1);

	return 0;
}

void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//PA1
	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

//PA4 (DAC)
	/* Configure PA4 as analog */
	//GPIOA->MODER
	GPIOA->MODER |= (0x300);
	/* Ensure no pull-up/pull-down for PA4 */
	GPIOA->PUPDR |= ~(GPIO_PUPDR_PUPDR4);
	/* low speed mode for PA4 */
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4);

//PA6 (ADC)
	/* Configure PA6 as analog */
	//GPIOA->MODER
	GPIOA->MODER |= (0x3000);
	/* Ensure no pull-up/pull-down for PA6*/
	GPIOA->PUPDR |= ~(GPIO_PUPDR_PUPDR6);
	/* low speed mode for PA6 */
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR6);
}

void myGPIOB_Init()
{
//PB3 and 5
	/* Enable clock for GPIOB peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	/* Configure PB3/5 as AF */
	// Relevant register: GPIOA->MODER
	GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER5_1);

	/* Ensure no pull-up/pull-down for PB3, PB5 */
	// Relevant register: GPIOA->PUPDR
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR5 );
//PB4
	/*Configure PB4 as Output*/
	// Relevant register: GPIOA->MODER
	GPIOB->MODER |= (GPIO_MODER_MODER4_0);

	/* Ensure no pull-up/pull-down for PB4 */
	// Relevant register: GPIOA->PUPDR
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);

	/* Ensure push-pull mode for PB4*/
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_4);

	/* Ensure high-speed mode for PB4 */
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4);

}

void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= 0x01;
	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 |= 0x008C;
	/* Set clock prescaler value */
	TIM2->PSC |= myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR |= myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR |= 0x01;
	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn,0);
	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);
	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER=0x01;
}

void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0]=0x80;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR =0x02;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR =0x02;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn,0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	EXTI->PR=1;
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR =0x00;

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 = TIM_CR1_CEN;
	}
}


void mySPI1_Init(){
	//SPI1 Structures
	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
	wait(5);
}

void myLCD_Init(void){

	//Set 4-bit interface
	//myLCD_SendInstruction(0x20);
	mySPI1_SendData(0x2);
	mySPI1_SendData(0x82);
	mySPI1_SendData(0x2);
	wait(4);

	//Set Function
	myLCD_SendInstruction(0x28);
	wait(4);

	//Turn on displayed
	myLCD_SendInstruction(0x0F);
	wait(4);

	//Entry mode
	myLCD_SendInstruction(0x06);
	wait(4);

	//Clear Display
	myLCD_SendInstruction(0x01);
	wait(4);

}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	// Your local variables...

	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//
		// 1. If this is the first edge:
		if(setRise==0){
			setRise=1;
		//	- Clear count register (TIM2->CNT).
			TIM2->CNT=0x0;
		//	- Start timer (TIM2->CR1).
			TIM2->CR1 = TIM_CR1_CEN;
		}
		//    Else (this is the second edge):
		else{
		//	- Stop timer (TIM2->CR1).
			TIM2->CR1 = 0x0000;
		//	- Read out count register (TIM2->CNT).
			count=(int)TIM2->CNT;
		//	- Calculate signal period and frequency.
			frequency= (float)(clock_Frequency/(float)count);
			resistance = ADC1->DR;
		//	- Print calculated values to the console.
		//	  NOTE: Function trace_printf does not work
		//	  with floating-point numbers: you must use
		//	  "unsigned int" type to print your signal
		//	  period and frequency.
			trace_printf("Frequency: %.8f, Resistance: %.2f\n",frequency,resistance);
			setRise=0;
		}
		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR=0x2;

	}
}
void mySPI1_SendData(unsigned char data){
	/* Force your LCK signal to 0 */
	GPIOB->BRR = GPIO_Pin_4;

	/* Wait until SPI1 is ready (TXE = 1 or BSY = 0) */
	while((SPI1->SR & 0x80)!=0);

	/* Assumption: your data holds 8 bits to be sent */
	SPI_SendData8(SPI1, data);

	/* Wait until SPI1 is not busy (BSY = 0) */
	while((SPI1->SR & 0x80)!=0);

	/* Force your LCK signal to 1 */
	GPIOB->BSRR = GPIO_Pin_4;
}

void myLCD_SendInstruction(unsigned char data){
	//Split byte into Low and High 4 bits
	unsigned char H = data>>4;
	unsigned char L = data & 0x0F;
	//Send LCD data via pulse
	//High
	mySPI1_SendData(H);
	mySPI1_SendData(0x80|H);
	mySPI1_SendData(H);
	wait(4);

	//Low if L!=0
	mySPI1_SendData(L);
	mySPI1_SendData(0x80|L);
	mySPI1_SendData(L);
	wait(4);
}

//Sends a char to be displayed on LCD
void myLCD_SendChar(unsigned char data){
	//Split byte into Low and High 4 bits
	unsigned char H = data>>4;
	unsigned char L = data & 0x0F;
	//Send LCD data via pulse
	//High
	mySPI1_SendData(0x40 | H);
	mySPI1_SendData(0xC0 | H);
	mySPI1_SendData(0x40 | H);
	wait(4);
	//lLow
	mySPI1_SendData(0x40 | L);
	mySPI1_SendData(0xC0 | L);
	mySPI1_SendData(0x40 | L);
	wait(4);
}

void myDAC_ADC_Init(){
	DAC->CR |= 0x1;
	ADC1->CR |=0x1;
}

void wait(uint32_t mag){
	static volatile uint32_t i;
	for(i=0;i<mag*50000;i++);
	i=0;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------



