 //******   Includes
#include <stm32f103c8_bitfield.h>
#include <stm32f103c8_BitDescription.h>


int main(void) {


	X_System_Init();
	X_Clock_Init();

	RCC->APB2ENR.bit.IOPAEN = RCC_APB2ENR_IOPAEN__GPIOA_Clock_Enable;

	GPIOA->CRH.bit.MODE8 = GPIO_CRH_MODEx__Output_50M_AF_PP;


	RCC->CFGR.bit.MCO = RCC_CFGR_MCO__SYSCLK;
		while(1){


		}
}







