
main(int argc, char* argv[])
{

	trace_printf("This is Part 2 of Introductory Lab...\n");
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
	
	//Display Something
	//Set address
	myLCD_SendData(0x80,0);
	//Display Word
	myLCD_SendData(72);
	
	while(1);
	
	return 0;

}
//Initialize SPI1
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

//Sends Data to SPI
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

