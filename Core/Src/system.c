#include "system.h"


void X_System_Init(void){

	RCC->APB2ENR.bit.AFIOEN = RCC_APB2ENR_AFIOEN__AFIO_Clock_Enable;

	AFIO->MAPR.bit.SWJ_CFG = AFIO_MAPR_SWJ_CFG__JTAG_Disable_SW_Enable;


}
void X_Clock_Init(void){

	volatile uint32_t counter=0;
	RCC->CR.bit.HSEBYP = RCC_CR_HSEBYP__OscillatorNotBypassed;


	RCC->CR.bit.HSEON = RCC_CR_HSEON__HSE_ON;

	do{
		counter++;
	}while( !(RCC->CR.bit.HSERDY) && (counter<0x500)  );

	if(RCC->CR.bit.HSERDY){

		FLASH->ACR.bit.PRFTBE= FLASH_ACR_PRFTBE__Prefetch_Enable;
		FLASH->ACR.bit.LATENCY = FLASH_ACR_LATENCY__TwoWait_48MHz_SYSCLK_72MHz;

		RCC->CFGR.bit.HPRE = RCC_CFGR_HPRE__SYSCLK_NotDivided;

		RCC->CFGR.bit.PPRE1 = RCC_CFGR_PPRE1__HCLK_DividedBy_2;

		RCC->CFGR.bit.PPRE2 = RCC_CFGR_PPRE2__HCLK_NotDivided;

		RCC->CFGR.bit.ADCPRE = RCC_CFGR_ADCPRE__PCLK2_DividedBy_6;

		RCC->CFGR.bit.USBPRE = RCC_CFGR_USBPRE__PLL_Is_Divided_1p5;

		RCC->CFGR.bit.PLLXTPRE = RCC_CFGR_PLLXTPRE__HSE_NotDivided;

		RCC->CFGR.bit.PLLSRC = RCC_CFGR_PLLSRC__HSE;

		RCC->CFGR.bit.PLLMUL = RCC_CFGR_PLLMUL__InputClockx9;

		RCC->CR.bit.PLLON = RCC_CR_PLLON__PLL_ON;

		while( !(RCC->CR.bit.PLLRDY) );

		RCC->CFGR.bit.SW = RCC_CFGR_SW__PLL;

		while( RCC->CFGR.bit.SWS != RCC_CFGR_SWS__SwitchStatus_PLL );





















	}


}



