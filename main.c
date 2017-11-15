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
void mySPI1_SendData(unsigned char data);
void myLCD_SendData(unsigned char data, unsigned char RS);

//SPI1 Structs
SPI_InitTypeDef SPI_InitStructInfo;
SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

// Your global variables...
static float clock_Frequency = 48000000;

unsigned char setRise =0;
int count=0;
float frequency =0;
float period=0;
unsigned char data= 0x30;
main(int argc, char* argv[])
{

	trace_printf("This is a Lab Project Test\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		/* Initialize I/O port PB */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	mySPI1_Init();		/* Initialize SPI1 */

	//Set 4-bit interfacing
	myLCD_SendData(0x20,1);
	//Set Function
	myLCD_SendData(0x28,1);
	//Turn on displayed
	myLCD_SendData(0x0C,1);
	//Entry mode
	myLCD_SendData(0x06,1);
	//Clear Display
	myLCD_SendData(0x01,1);

	trace_printf("Intialized LCD\n");
	//Display Something
	//Set address
	myLCD_SendData(0x80,0);
	//Display Word
	myLCD_SendData(72,0);
	trace_printf("Displayed H on LCD\n");

	while(1);

	return 0;

}


void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	/* Configure PA1 as input */

	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

	/*Configure PA9 as output*/
	// Relevant register: GPIOA->MODER
		GPIOA->MODER |= (GPIO_MODER_MODER9_0);

		/* Ensure no pull-up/pull-down for PA9 */
		// Relevant register: GPIOA->PUPDR
		GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9);

		/* Ensure push-pull mode for PA9*/
		GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_9);

		/* Ensure high-speed mode for PA9 */
		GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9);

}

void myGPIOB_Init()
{
	/* Enable clock for GPIOB peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	/* Configure PB3/5 as output */

	// Relevant register: GPIOA->MODER
	GPIOB->MODER |= (GPIO_MODER_MODER3_0 | GPIO_MODER_MODER5_0);

	/* Ensure no pull-up/pull-down for PB3, PB5 */
	// Relevant register: GPIOA->PUPDR
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR5 );

	/* Ensure push-pull mode for PB3, BC5 */
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_3 |
	GPIO_OTYPER_OT_5);

	/* Ensure high-speed mode for PB3, PB5 */
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR3 |
	GPIO_OSPEEDER_OSPEEDR5);

}


void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR = 0x01;
	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = 0x008C;
	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = 0x01;
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
	RCC->APB2ENR=0x1000;

	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct->SPI_BaudRatePrescaler = 0;
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct->SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, SPI_InitStruct);

	SPI_Cmd(SPI1, ENABLE);
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
			period= 1.0/frequency;
		//	- Print calculated values to the console.
		//	  NOTE: Function trace_printf does not work
		//	  with floating-point numbers: you must use
		//	  "unsigned int" type to print your signal
		//	  period and frequency.
			trace_printf("Period: %.8f, Frequency: %.3f\n",period,frequency);
			setRise=0;
		}
		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR=0x2;

	}
}
void mySPI1_SendData(unsigned char data){
	/* Force your LCK signal to 0 */
	GPIOC->BRR = ((uint16_t)0x1000);
	/* Wait until SPI1 is ready (TXE = 1 or BSY = 0) */
	while(!(SPI1->SR & 0x02) ||(SPI1->SR & 0x80));
	/* Assumption: your data holds 8 bits to be sent */
	SPI_SendData8(SPI1, data);
	/* Wait until SPI1 is not busy (BSY = 0) */
	while(SPI1->SR & 0x80);
	/* Force your LCK signal to 1 */
	GPIOC->BSRR = ((uint16_t)0x1000);
}

//Sends 1 byte of data to LCD( Either comand or code to be displayed)
void myLCD_SendData(unsigned char data, unsigned char RS){
	//Split byte into Low and High 4 bits
	unsigned char H = data>>4;
	unsigned char L = data & 0x0F;
	//Send LCD data via pulse
	//High
	mySPI1_SendData(H|(RS<<6));
	mySPI1_SendData((H | 0x80)|(RS<<6));
	mySPI1_SendData(H |(RS<<6));
	//Low if L!=0
	if(L!=0){
		mySPI1_SendData(L|(RS<<6));
		mySPI1_SendData((L | 0x80)|(RS<<6));
		mySPI1_SendData(L|(RS<<6));
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

