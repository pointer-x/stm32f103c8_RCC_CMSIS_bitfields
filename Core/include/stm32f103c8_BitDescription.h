/**
  ******************************************************************************
  * @file    stm32f103_BitDescription.h
  * @brief   stm32f103_BitDescription.h
  ******************************************************************************
  *
  * This file contains definitions for all values that are going to be written
  * or read from register's bit fields.
  *
  * How definitions are designed?
  * #define [peripheral name]_[register name]_[bit name]__[description]     [value]
  * example:
  * #define RCC_APB2ENR_IOPAEN__GPIOA_Clock_Enable     0x1U
  * When this definition is written in bit IOPAEN of APB2ENR register of RCC
  * peripheral, its going to enable GPIOA peripheral clock.
  *
  * Guid to read macro names:
  * for decimal point in macro name,"p" character is used. example : 1p5 = 1.5
  * List of abbreviations:
  * IntrptEnbl      Interrupt Enable
  * IntrptDsbl      Interrupt Disable
  * IntrptFlgSt     Interrupt Flag is Set
  * IntrptFlgRst    Interrupt Flag is Reset
  * IntrptFlgClr    Interrupt Flag Clear
  * FlgClr          Flag Clear
  * Cmplt           Complete
  * Discont         discontinuous
  * Simul           simultaneous
  * Conv            Conversion
  * Synchronous     Sync
  * ASynchronous    Async
  *
  ******************************************************************************
  */

#pragma once

// CORTEX Preemption Priority Group
#define NVIC_PRIORITYGROUP_0__0_pre_4_sub               0x7U
#define NVIC_PRIORITYGROUP_1__1_pre_3_sub               0x6U
#define NVIC_PRIORITYGROUP_2__2_pre_2_sub               0x5U
#define NVIC_PRIORITYGROUP_3__3_pre_1_sub               0x4U
#define NVIC_PRIORITYGROUP_4__4_pre_0_sub               0x3U

#define FLAG_CLEAR										0x0

// Vector Table base offset field.This value must be a multiple of 0x200.
#define VECT_TAB_OFFSET                                 0x0U

// Time out for HSE start up
#define HSE_STARTUP_TIMEOUT                             0x500U

/******************************************************************************
 ******     FLASH
******************************************************************************/

// FLASH : ACR
#define FLASH_ACR_LATENCY__ZeroWait_0_SYSCLK_24MHz      0x0U
#define FLASH_ACR_LATENCY__OneWait_24MHz_SYSCLK_48MHz   0x1U
#define FLASH_ACR_LATENCY__TwoWait_48MHz_SYSCLK_72MHz   0x2U

#define FLASH_ACR_HLFCYA__HalfCycle_Disable              0x0U
#define FLASH_ACR_HLFCYA__HalfCycle_Enable               0x1U

#define FLASH_ACR_PRFTBE__Prefetch_Disable              0x0U
#define FLASH_ACR_PRFTBE__Prefetch_Enable               0x1U

#define FLASH_ACR_PRFTBS__PrefetchStatus_Disabled       0x0U
#define FLASH_ACR_PRFTBS__PrefetchStatus_Enabled        0x1U

// FLASH : KEYR
#define FLASH_KEYR_FKEYR__UnlockFlash_1                 0x45670123U
#define FLASH_KEYR_FKEYR__UnlockFlash_2                 0xCDEF89ABU

// FLASH : OPTKEYR
#define FLASH_OPTKEYR_OPTKEYR__UnlockOptionBytes_1      0x45670123U
#define FLASH_OPTKEYR_OPTKEYR__UnlockOptionBytes_2      0xCDEF89ABU

// FLASH : SR
#define FLASH_SR_BSY__OperationInProgress               0x1U
#define FLASH_SR_BSY__OperationFinished                 0x0U

#define FLASH_SR_PGERR__ProgrammingErrorOccurred        0x1U
#define FLASH_SR_PGERR__No_ProgrammingError             0x0U
#define FLASH_SR_PGERR__Clear_PGERR                     0x1U

#define FLASH_SR_WRPRTERR__WriteProtectionErrorOccurred 0x1U
#define FLASH_SR_WRPRTERR__No_WriteProtectionError      0x0U
#define FLASH_SR_WRPRTERR__Clear_WRPRTERR               0x1U

#define FLASH_SR_EOP__OperationCmplt                    0x1U
#define FLASH_SR_EOP__OperationNotCmplt                 0x0U
#define FLASH_SR_EOP__Clear_EOP                         0x1U

// FLASH : CR
#define FLASH_CR_PG__FlashProgrammingChosen             0x1U

#define FLASH_CR_PER__PageErase                         0x1U

#define FLASH_CR_MER__MassErase                         0x1U

#define FLASH_CR_OPTPG__OptionByteProgrammingChosen     0x1U

#define FLASH_CR_OPTER__OptionByteErase                 0x1U

#define FLASH_CR_START__StartErase                      0x1U

#define FLASH_CR_LOCK__LockFlash                        0x1U

#define FLASH_CR_LOCK__OptionbytesWrite_Enable          0x1U
#define FLASH_CR_LOCK__OptionbytesWrite_Disable         0x0U

#define FLASH_CR_ERRIE__Error_IntrptDsbl                0x0U
#define FLASH_CR_ERRIE__Error_IntrptEnbl                0x1U

#define FLASH_CR_EOPIE__EndOfOperatoin_IntrptDsbl       0x0U
#define FLASH_CR_EOPIE__EndOfOperatoin_IntrptEnbl       0x1U

// FLASH : OBR
#define FLASH_OBR_OPTERR__NoOptionByteError             0x0U
#define FLASH_OBR_OPTERR__OptionByteErrorOccurred       0x1U

#define FLASH_OBR_RDPRT__ReadProtection_Enabled         0x1U
#define FLASH_OBR_RDPRT__ReadProtection_Disabled        0x0U

#define FLASH_OBR_WDG_SW__HardwareWatchdog              0x0U
#define FLASH_OBR_WDG_SW__SoftwareWatchdog              0x1U

#define FLASH_OBR_nRST_STOP__ResetGeneratedEnteringStop 0x0U
#define FLASH_OBR_nRST_STOP__NoResetGenerated           0x1U

#define FLASH_OBR_nRST_STDBY__Reset_Generated_Entering_Standby   0x0U
#define FLASH_OBR_nRST_STDBY__NoResetGenerated                   0x1U

// FLASH : WRPR
#define FLASH_WRPR_WRP__WriteProtectionActive           0x0U
#define FLASH_WRPR_WRP__WriteProtectionNotActive        0x1U

/******************************************************************************
 ******     CRC
******************************************************************************/
#define CRC_CR_RESET__ResetCRC                          0x1U

/******************************************************************************
 ******     PWR
******************************************************************************/

// PWR : CR
#define PWR_CR_DBP__BackupDomainAccess_Disable          0x1U
#define PWR_CR_DBP__BackupDomainAccess_Enable           0x0U

#define PWR_CR_PLS__PVD_Level_2p2V                      0x0U
#define PWR_CR_PLS__PVD_Level_2p3V                      0x1U
#define PWR_CR_PLS__PVD_Level_2p4V                      0x2U
#define PWR_CR_PLS__PVD_Level_2p5V                      0x3U
#define PWR_CR_PLS__PVD_Level_2p6V                      0x4U
#define PWR_CR_PLS__PVD_Level_2p7V                      0x5U
#define PWR_CR_PLS__PVD_Level_2p8V                      0x6U
#define PWR_CR_PLS__PVD_Level_2p9V                      0x7U

#define PWR_CR_PVDE__PowerVoltageDetector_Disable       0x0U
#define PWR_CR_PVDE__PowerVoltageDetector_Enable        0x1U

#define PWR_CR_CSBF__Standby_FlgClr                     0x1U

#define PWR_CR_CWUF__Wakeup_FlgClr                      0x1U

#define PWR_CR_PDDS__StopMode_WhenDeepSleep             0x0U
#define PWR_CR_PDDS__StandbyMode_whenDeepSleep          0x1U

#define PWR_CR_LPDS__Regulator_On_InStopMode            0x0U
#define PWR_CR_LPDS__Regulator_LowPower_InStopMode      0x1U

// PWR : CSR
#define PWR_CSR_WUF__NO_WakeUpEvent                     0x0U
#define PWR_CSR_WUF__WakeUpEventReceived                0x1U

#define PWR_CSR_SBF__NotBeenInStandbyMode               0x0U
#define PWR_CSR_SBF__BeenInStandbyMode                  0x1U

#define PWR_CSR_PVDO__VddHigherThanThreshold            0x0U
#define PWR_CSR_PVDO__VddLowerThanThreshold             0x1U

#define PWR_CSR_EWUP__WakeupPin_IsUsedAsGPIO            0x0U
#define PWR_CSR_EWUP__WakeupPin_IsUsedAsWakeup          0x1U

/******************************************************************************
 ******     BKP
******************************************************************************/

// BKP : RTCCR
#define BKP_RTCCR_CCO__Outputs_RTC_ClockDiv64OnTAMPER   0x1U

#define BKP_RTCCR_ASOE__RTC_Output_Disable              0x0U
#define BKP_RTCCR_ASOE__RTC_Output_Enable               0x1U

#define BKP_RTCCR_ASOS__AlarmPulse                      0x0U
#define BKP_RTCCR_ASOS__SecondPulse                     0x1U

// BKP : CR
#define BKP_CR_TPE__TamperPin_FreeForGPIO               0x0U
#define BKP_CR_TPE__TamperPin_Enable                    0x1U

#define BKP_CR_TPAL__HighLeverOnTamperResetRegisters    0x0U
#define BKP_CR_TPAL__LowLeverOnTamperResetRegisters     0x1U

// BKP : CSR
#define BKP_CSR_CTE__TamperEventClear                   0x1U

#define BKP_CSR_CTI__TamperInterruptClear               0x1U

#define BKP_CSR_TPIE__Tamper_IntrptDsbl                 0x0U
#define BKP_CSR_TPIE__Tamper_IntrptEnbl                 0x1U

#define BKP_CSR_TEF__NoTamperEvent                      0x0U
#define BKP_CSR_TEF__TamperEventOccurred                0x1U

#define BKP_CSR_TIF__NoTamperInterrupt                  0x0U
#define BKP_CSR_TIF__TamperInterruptOccurred            0x1U

/******************************************************************************
 ******     RCC
******************************************************************************/

// RCC : CR
#define RCC_CR_PLLRDY__PLL_unLocked                     0x0U
#define RCC_CR_PLLRDY__PLL_Locked                       0x1U

#define RCC_CR_PLLON__PLL_OFF                           0x0U
#define RCC_CR_PLLON__PLL_ON                            0x1U

#define RCC_CR_CSSON__ClockDetector_OFF                 0x0U
#define RCC_CR_CSSON__ClockDetector_ON                  0x1U

#define RCC_CR_HSEBYP__OscillatorNotBypassed            0x0U
#define RCC_CR_HSEBYP__OscillatorBypassed               0x1U

#define RCC_CR_HSERDY__HSE_NotReady                     0x0U
#define RCC_CR_HSERDY__HSE_Ready                        0x1U

#define RCC_CR_HSEON__HSE_OFF                           0x0U
#define RCC_CR_HSEON__HSE_ON                            0x1U

#define RCC_CR_HSIRDY__HSI_NotReady                     0x0U
#define RCC_CR_HSIRDY__HSI_Ready                        0x1U

#define RCC_CR_HSION__HSI_OFF                           0x0U
#define RCC_CR_HSION__HSI_ON                            0x1U

// RCC : CFGR
#define RCC_CFGR_MCO__NoClock                           0x0U
#define RCC_CFGR_MCO__SYSCLK                            0x4U
#define RCC_CFGR_MCO__HSI                               0x5U
#define RCC_CFGR_MCO__HSE                               0x6U
#define RCC_CFGR_MCO__PLL_DividedBy2                    0x7U

#define RCC_CFGR_USBPRE__PLL_Is_Divided_1p5             0x0U
#define RCC_CFGR_USBPRE__PLL_Not_Divided                0x1U

#define RCC_CFGR_PLLMUL__InputClockx2                   0x0U
#define RCC_CFGR_PLLMUL__InputClockx3                   0x1U
#define RCC_CFGR_PLLMUL__InputClockx4                   0x2U
#define RCC_CFGR_PLLMUL__InputClockx5                   0x3U
#define RCC_CFGR_PLLMUL__InputClockx6                   0x4U
#define RCC_CFGR_PLLMUL__InputClockx7                   0x5U
#define RCC_CFGR_PLLMUL__InputClockx8                   0x6U
#define RCC_CFGR_PLLMUL__InputClockx9                   0x7U
#define RCC_CFGR_PLLMUL__InputClockx7                   0x5U
#define RCC_CFGR_PLLMUL__InputClockx8                   0x6U
#define RCC_CFGR_PLLMUL__InputClockx9                   0x7U
#define RCC_CFGR_PLLMUL__InputClockx10                  0x8U
#define RCC_CFGR_PLLMUL__InputClockx11                  0x9U
#define RCC_CFGR_PLLMUL__InputClockx12                  0xAU
#define RCC_CFGR_PLLMUL__InputClockx13                  0xBU
#define RCC_CFGR_PLLMUL__InputClockx14                  0xCU
#define RCC_CFGR_PLLMUL__InputClockx15                  0xDU
#define RCC_CFGR_PLLMUL__InputClockx16                  0xEU

#define RCC_CFGR_PLLXTPRE__HSE_NotDivided               0x0U
#define RCC_CFGR_PLLXTPRE__HSE_Dividedby_2              0x1U

#define RCC_CFGR_PLLSRC__HSI_DividedBy_2                0x0U
#define RCC_CFGR_PLLSRC__HSE                            0x1U

#define RCC_CFGR_ADCPRE__PCLK2_DividedBy_2              0x0U
#define RCC_CFGR_ADCPRE__PCLK2_DividedBy_4              0x1U
#define RCC_CFGR_ADCPRE__PCLK2_DividedBy_6              0x2U
#define RCC_CFGR_ADCPRE__PCLK2_DividedBy_8              0x3U

#define RCC_CFGR_PPRE2__HCLK_NotDivided                 0x0U
#define RCC_CFGR_PPRE2__HCLK_DividedBy_2                0x4U
#define RCC_CFGR_PPRE2__HCLK_DividedBy_4                0x5U
#define RCC_CFGR_PPRE2__HCLK_DividedBy_8                0x6U
#define RCC_CFGR_PPRE2__HCLK_DividedBy_16               0x7U

#define RCC_CFGR_PPRE1__HCLK_NotDivided                 0x0U
#define RCC_CFGR_PPRE1__HCLK_DividedBy_2                0x4U
#define RCC_CFGR_PPRE1__HCLK_DividedBy_4                0x5U
#define RCC_CFGR_PPRE1__HCLK_DividedBy_8                0x6U
#define RCC_CFGR_PPRE1__HCLK_DividedBy_16               0x7U

#define RCC_CFGR_HPRE__SYSCLK_NotDivided                0x0U
#define RCC_CFGR_HPRE__SYSCLK_DividedBy_2               0x8U
#define RCC_CFGR_HPRE__SYSCLK_DividedBy_4               0x9U
#define RCC_CFGR_HPRE__SYSCLK_DividedBy_8               0xAU
#define RCC_CFGR_HPRE__SYSCLK_DividedBy_16              0xBU
#define RCC_CFGR_HPRE__SYSCLK_DividedBy_64              0xCU
#define RCC_CFGR_HPRE__SYSCLK_DividedBy_128             0xDU
#define RCC_CFGR_HPRE__SYSCLK_DividedBy_256             0xEU
#define RCC_CFGR_HPRE__SYSCLK_DividedBy_512             0xFU

#define RCC_CFGR_SWS__SwitchStatus_HSI                  0x0U
#define RCC_CFGR_SWS__SwitchStatus_HSE                  0x1U
#define RCC_CFGR_SWS__SwitchStatus_PLL                  0x2U

#define RCC_CFGR_SW__HSI                                0x0U
#define RCC_CFGR_SW__HSE                                0x1U
#define RCC_CFGR_SW__PLL                                0x2U

// RCC : CIR
#define RCC_CIR_LSIRDYF__LSI_Ready_IntrptFlgRst         0x0U
#define RCC_CIR_LSIRDYF__LSI_Ready_IntrptFlgSt          0x1U

#define RCC_CIR_LSERDYF__LSE_Ready_IntrptFlgRst         0x0U
#define RCC_CIR_LSERDYF__LSE_Ready_IntrptFlgSt          0x1U

#define RCC_CIR_HSIRDYF__HSI_Ready_IntrptFlgRst         0x0U
#define RCC_CIR_HSIRDYF__HSI_Ready_IntrptFlgSt          0x1U

#define RCC_CIR_HSERDYF__HSE_Ready_IntrptFlgRst         0x0U
#define RCC_CIR_HSERDYF__HSE_Ready_IntrptFlgSt          0x1U

#define RCC_CIR_PLLRDYF__PLL_Ready_IntrptFlgRst         0x0U
#define RCC_CIR_PLLRDYF__PLL_Ready_IntrptFlgSt          0x1U

#define RCC_CIR_CSSF__CCS_Ready_IntrptFlgRst            0x0U
#define RCC_CIR_CSSF__CSS_Ready_IntrptFlgSt             0x1U

#define RCC_CIR_LSIRDYIE__LSIRDY_IntrptDsbl             0x0U
#define RCC_CIR_LSIRDYIE__LSIRDY_IntrptEnbl             0x1U

#define RCC_CIR_LSERDYIE__LSERDY_IntrptDsbl             0x0U
#define RCC_CIR_LSERDYIE__LSERDY_IntrptEnbl             0x1U

#define RCC_CIR_HSIRDYIE__HSIRDY_IntrptDsbl             0x0U
#define RCC_CIR_HSIRDYIE__HSIRDY_IntrptEnbl             0x1U

#define RCC_CIR_HSERDYIE__HSERDY_IntrptDsbl             0x0U
#define RCC_CIR_HSERDYIE__HSERDY_IntrptEnbl             0x1U

#define RCC_CIR_PLLRDYIE__PLLRDY_IntrptDsbl             0x0U
#define RCC_CIR_PLLRDYIE__PLLRDY_IntrptEnbl             0x1U

#define RCC_CIR_LSIRDYC__LSIRDYF_FlgClr                 0x1U

#define RCC_CIR_LSERDYC__LSERDYF_FlgClr                 0x1U

#define RCC_CIR_HSIRDYC__HSIRDYF_FlgClr                 0x1U

#define RCC_CIR_HSERDYC__HSERDYF_FlgClr                 0x1U

#define RCC_CIR_PLLRDYC__PLLRDYF_FlgClr                 0x1U

#define RCC_CIR_CSSC__CSSF_FlgClr                       0x1U

// RCC : APB2RSTR
#define RCC_APB2RSTR_AFIORST__Reset_AFIO                0x1U

#define RCC_APB2RSTR_IOPARST__Reset_GPIOA               0x1U

#define RCC_APB2RSTR_IOPBRST__Reset_GPIOB               0x1U

#define RCC_APB2RSTR_IOPCRST__Reset_GPIOC               0x1U

#define RCC_APB2RSTR_IOPDRST__Reset_GPIOD               0x1U

#define RCC_APB2RSTR_IOPERST__Reset_GPIOE               0x1U

#define RCC_APB2RSTR_IOPFRST__Reset_GPIOF               0x1U

#define RCC_APB2RSTR_IOPGRST__Reset_GPIOG               0x1U

#define RCC_APB2RSTR_ADC1RST__Reset_ADC1                0x1U

#define RCC_APB2RSTR_ADC2RST__Reset_ADC2                0x1U

#define RCC_APB2RSTR_TIM1RST__Reset_TIM1                0x1U

#define RCC_APB2RSTR_SPI1RST__Reset_SPI1                0x1U

#define RCC_APB2RSTR_TIM8RST__Reset_TIM8                0x1U

#define RCC_APB2RSTR_USART1RST__Reset_USART1            0x1U

#define RCC_APB2RSTR_ADC3RST__Reset_ADC3                0x1U

#define RCC_APB2RSTR_TIM9RST__Reset_TIM9                0x1U

#define RCC_APB2RSTR_TIM10RST__Reset_TIM10              0x1U

#define RCC_APB2RSTR_TIM11RST__Reset_TIM11              0x1U

// RCC : APB1RSTR
#define RCC_APB1RSTR_TIM2RST__Reset_TIM2                0x1U

#define RCC_APB1RSTR_TIM3RST__Reset_TIM3                0x1U

#define RCC_APB1RSTR_TIM4RST__Reset_TIM4                0x1U

#define RCC_APB1RSTR_TIM5RST__Reset_TIM5                0x1U

#define RCC_APB1RSTR_TIM6RST__Reset_TIM6                0x1U

#define RCC_APB1RSTR_TIM7RST__Reset_TIM7                0x1U

#define RCC_APB1RSTR_TIM12RST__Reset_TIM12              0x1U

#define RCC_APB1RSTR_TIM13RST__Reset_TIM13              0x1U

#define RCC_APB1RSTR_TIM14RST__Reset_TIM14              0x1U

#define RCC_APB1RSTR_WWDGRST__Reset_WWDG                0x1U

#define RCC_APB1RSTR_SPI2RST__Reset_SPI2                0x1U

#define RCC_APB1RSTR_SPI3RST__Reset_SPI3                0x1U

#define RCC_APB1RSTR_USART2RST__Reset_USART2            0x1U

#define RCC_APB1RSTR_USART3RST__Reset_USART3            0x1U

#define RCC_APB1RSTR_UART4RST__Reset_USART4             0x1U

#define RCC_APB1RSTR_UART5RST__Reset_USART5             0x1U

#define RCC_APB1RSTR_I2C1RST__Reset_I2C1                0x1U

#define RCC_APB1RSTR_I2C2RST__Reset_I2C2                0x1U

#define RCC_APB1RSTR_USBRST__Reset_USB                  0x1U

#define RCC_APB1RSTR_CANRST__Reset_CAN                  0x1U

#define RCC_APB1RSTR_BKPRST__Reset_BKP                  0x1U

#define RCC_APB1RSTR_PWRRST__Reset_PWR                  0x1U

#define RCC_APB1RSTR_DACRST__Reset_DAC                  0x1U

// RCC : AHBENR
#define RCC_AHBENR_DMA1EN__DMA1_Clock_Enable            0x1U
#define RCC_AHBENR_DMA1EN__DMA1_Clock_Disable           0x0U

#define RCC_AHBENR_DMA2EN__DMA2_Clock_Enable            0x1U
#define RCC_AHBENR_DMA2EN__DMA2_Clock_Disable           0x0U

#define RCC_AHBENR_SRAMEN__SRAM_Clock_Enable            0x1U
#define RCC_AHBENR_SRAMEN__SRAM_Clock_Disable           0x0U

#define RCC_AHBENR_FLITFENEN__FLITFEN_Clock_Enable      0x1U
#define RCC_AHBENR_FLITFENEN__FLITFEN_Clock_Disable     0x0U

#define RCC_AHBENR_CRCEN__CRC_Clock_Enable              0x1U
#define RCC_AHBENR_CRCEN__CRC_Clock_Disable             0x0U

#define RCC_AHBENR_FSMCEN__FSMC_Clock_Enable            0x1U
#define RCC_AHBENR_FSMCEN__FSMC_Clock_Disable           0x0U

#define RCC_AHBENR_SDIOEN__SDIO_Clock_Enable            0x1U
#define RCC_AHBENR_SDIOEN__SDIO_Clock_Disable           0x0U

// RCC : APB2ENR
#define RCC_APB2ENR_AFIOEN__AFIO_Clock_Enable           0x1U
#define RCC_APB2ENR_AFIOEN__AFIO_Clock_Disable          0x0U

#define RCC_APB2ENR_IOPAEN__GPIOA_Clock_Enable          0x1U
#define RCC_APB2ENR_IOPAEN__GPIOA_Clock_Disable         0x0U

#define RCC_APB2ENR_IOPBEN__GPIOB_Clock_Enable          0x1U
#define RCC_APB2ENR_IOPBEN__GPIOB_Clock_Disable         0x0U

#define RCC_APB2ENR_IOPCEN__GPIOC_Clock_Enable          0x1U
#define RCC_APB2ENR_IOPCEN__GPIOC_Clock_Disable         0x0U

#define RCC_APB2ENR_IOPDEN__GPIOD_Clock_Enable          0x1U
#define RCC_APB2ENR_IOPDEN__GPIOD_Clock_Disable         0x0U

#define RCC_APB2ENR_IOPEEN__GPIOE_Clock_Enable          0x1U
#define RCC_APB2ENR_IOPEEN__GPIOE_Clock_Disable         0x0U

#define RCC_APB2ENR_IOPFEN__GPIOF_Clock_Enable          0x1U
#define RCC_APB2ENR_IOPFEN__GPIOF_Clock_Disable         0x0U

#define RCC_APB2ENR_IOPGEN__GPIOG_Clock_Enable          0x1U
#define RCC_APB2ENR_IOPGEN__GPIOG_Clock_Disable         0x0U

#define RCC_APB2ENR_ADC1EN__ADC1_Clock_Enable           0x1U
#define RCC_APB2ENR_ADC1EN__ADC1_Clock_Disable          0x0U

#define RCC_APB2ENR_ADC2EN__ADC2_Clock_Enable           0x1U
#define RCC_APB2ENR_ADC2EN__ADC2_Clock_Disable          0x0U

#define RCC_APB2ENR_TIM1EN__TIM1_Clock_Enable           0x1U
#define RCC_APB2ENR_TIM1EN__TIM1_Clock_Disable          0x0U

#define RCC_APB2ENR_SPI1EN__SPI1_Clock_Enable           0x1U
#define RCC_APB2ENR_SPI1EN__SPI1_Clock_Disable          0x0U

#define RCC_APB2ENR_TIM8EN__TIM8_Clock_Enable           0x1U
#define RCC_APB2ENR_TIM8EN__TIM8_Clock_Disable          0x0U

#define RCC_APB2ENR_USART1EN__USART1_Clock_Enable       0x1U
#define RCC_APB2ENR_USART1EN__USART1_Clock_Disable      0x0U

#define RCC_APB2ENR_ADC3EN__ADC3_Clock_Enable           0x1U
#define RCC_APB2ENR_ADC3EN__ADC3_Clock_Disable          0x0U

#define RCC_APB2ENR_TIM9EN__TIM9_Clock_Enable           0x1U
#define RCC_APB2ENR_TIM9EN__TIM9_Clock_Disable          0x0U

#define RCC_APB2ENR_TIM10EN__TIM10_Clock_Enable         0x1U
#define RCC_APB2ENR_TIM10EN__TIM10_Clock_Disable        0x0U

#define RCC_APB2ENR_TIM11EN__TIM11_Clock_Enable         0x1U
#define RCC_APB2ENR_TIM11EN__TIM11_Clock_Disable        0x0U

// RCC : APB1ENR
#define RCC_APB1ENR_TIM2EN__TIM2_Clock_Enable           0x1U
#define RCC_APB1ENR_TIM2EN__TIM2_Clock_Disable          0x0U

#define RCC_APB1ENR_TIM3EN__TIM3_Clock_Enable           0x1U
#define RCC_APB1ENR_TIM3EN__TIM3_Clock_Disable          0x0U

#define RCC_APB1ENR_TIM4EN__TIM4_Clock_Enable           0x1U
#define RCC_APB1ENR_TIM4EN__TIM4_Clock_Disable          0x0U

#define RCC_APB1ENR_TIM5EN__TIM5_Clock_Enable           0x1U
#define RCC_APB1ENR_TIM5EN__TIM5_Clock_Disable          0x0U

#define RCC_APB1ENR_TIM6EN__TIM6_Clock_Enable           0x1U
#define RCC_APB1ENR_TIM6EN__TIM6_Clock_Disable          0x0U

#define RCC_APB1ENR_TIM7EN__TIM7_Clock_Enable           0x1U
#define RCC_APB1ENR_TIM7EN__TIM7_Clock_Disable          0x0U

#define RCC_APB1ENR_TIM12EN__TIM12_Clock_Enable         0x1U
#define RCC_APB1ENR_TIM12EN__TIM12_Clock_Disable        0x0U

#define RCC_APB1ENR_TIM13EN__TIM13_Clock_Enable         0x1U
#define RCC_APB1ENR_TIM13EN__TIM13_Clock_Disable        0x0U

#define RCC_APB1ENR_TIM14EN__TIM14_Clock_Enable         0x1U
#define RCC_APB1ENR_TIM14EN__TIM14_Clock_Disable        0x0U

#define RCC_APB1ENR_WWDGEN__WWDG_Clock_Enable           0x1U
#define RCC_APB1ENR_WWDGEN__WWDG_Clock_Disable          0x0U

#define RCC_APB1ENR_SPI2EN__SPI2_Clock_Enable           0x1U
#define RCC_APB1ENR_SPI2EN__SPI2_Clock_Disable          0x0U

#define RCC_APB1ENR_SPI3EN__SPI3_Clock_Enable           0x1U
#define RCC_APB1ENR_SPI3EN__SPI3_Clock_Disable          0x0U

#define RCC_APB1ENR_USART2EN__USART2_Clock_Enable       0x1U
#define RCC_APB1ENR_USART2EN__USART2_Clock_Disable      0x0U

#define RCC_APB1ENR_USART3EN__USART3_Clock_Enable       0x1U
#define RCC_APB1ENR_USART3EN__USART3_Clock_Disable      0x0U

#define RCC_APB1ENR_UART4EN__UART4_Clock_Enable         0x1U
#define RCC_APB1ENR_UART4EN__UART4_Clock_Disable        0x0U

#define RCC_APB1ENR_UART5EN__UART5_Clock_Enable         0x1U
#define RCC_APB1ENR_UART5EN__UART5_Clock_Disable        0x0U

#define RCC_APB1ENR_I2C1EN__I2C1_Clock_Enable           0x1U
#define RCC_APB1ENR_I2C1EN__I2C1_Clock_Disable          0x0U

#define RCC_APB1ENR_I2C2EN__I2C2_Clock_Enable           0x1U
#define RCC_APB1ENR_I2C2EN__I2C2_Clock_Disable          0x0U

#define RCC_APB1ENR_USBEN__USB_Clock_Enable             0x1U
#define RCC_APB1ENR_USBEN__USB_Clock_Disable            0x0U

#define RCC_APB1ENR_CANEN__CAN_Clock_Enable             0x1U
#define RCC_APB1ENR_CANEN__CAN_Clock_Disable            0x0U

#define RCC_APB1ENR_BKPEN__BKP_Clock_Enable             0x1U
#define RCC_APB1ENR_BKPEN__BKP_Clock_Disable            0x0U

#define RCC_APB1ENR_PWREN__PWR_Clock_Enable             0x1U
#define RCC_APB1ENR_PWREN__PWR_Clock_Disable            0x0U

#define RCC_APB1ENR_DACEN__DAC_Clock_Enable             0x1U
#define RCC_APB1ENR_DACEN__DAC_Clock_Disable            0x0U

// RCC : BDCR
#define RCC_BDCR_LSEON__External_32kHz_OFF              0x0U
#define RCC_BDCR_LSEON__External_32kHz_ON               0x1U

#define RCC_BDCR_LSERDY__External_32kHz_NotReady        0x0U
#define RCC_BDCR_LSERDY__External_32kHz_Ready           0x1U

#define RCC_BDCR_LSEBYP__External_32kHz_NotBypassed     0x0U
#define RCC_BDCR_LSEBYP__External_32kHz_Bypassed        0x1U

#define RCC_BDCR_RTCSEL__RTC_Source_NoClock             0x0U
#define RCC_BDCR_RTCSEL__RTC_Source_LSE                 0x1U
#define RCC_BDCR_RTCSEL__RTC_Source_LSI                 0x2U
#define RCC_BDCR_RTCSEL__RTC_Source_HSEdiv128           0x3U

#define RCC_BDCR_RTCEN__RTC_Clock_Disable               0x0U
#define RCC_BDCR_RTCEN__RTC_Clock_Enable                0x1U

#define RCC_BDCR_BDRST__Reset_NotActive                 0x0U
#define RCC_BDCR_BDRST__BackUp_Reset                    0x1U

// RCC : CSR
#define RCC_CSR_LSION__Internal_40kHz_OFF               0x0U
#define RCC_CSR_LSION__Internal_40kHz_ON                0x1U

#define RCC_CSR_LSIRDY__Internal_40kHz_NotReady         0x0U
#define RCC_CSR_LSIRDY__Internal_40kHz_Ready            0x1U

#define RCC_CSR_RMVF__Clear_ResetFlag                   0x1U

#define RCC_CSR_PINRSTF__No_NRST_Reset                  0x0U
#define RCC_CSR_PINRSTF__NRST_ResetOccurred             0x1U

#define RCC_CSR_PORRSTF__No_PORorPDR_Reset              0x0U
#define RCC_CSR_PORRSTF__PORorPDR_ResetOccurred         0x1U

#define RCC_CSR_SFTRSTF__No_Software_Reset              0x0U
#define RCC_CSR_SFTRSTF__Software_ResetOccurred         0x1U

#define RCC_CSR_IWDGRSTF__No_IWDG_Reset                 0x0U
#define RCC_CSR_IWDGRSTF__IWDG_ResetOccurred            0x1U

#define RCC_CSR_WWDGRSTF__No_WWDG_Reset                 0x0U
#define RCC_CSR_WWDGRSTF__WWDG_ResetOccurred            0x1U

#define RCC_CSR_LPWRRSTF__No_LowPower_Reset             0x0U
#define RCC_CSR_LPWRRSTF__LowPower_ResetOccurred        0x1U

/******************************************************************************
 ******     GPIO
******************************************************************************/

// GPIO Pins Mask definitions
#define GPIO_PIN_0_MSK                                  0x1U
#define GPIO_PIN_1_MSK                                  0x2U
#define GPIO_PIN_2_MSK                                  0x4U
#define GPIO_PIN_3_MSK                                  0x8U
#define GPIO_PIN_4_MSK                                  0x10U
#define GPIO_PIN_5_MSK                                  0x20U
#define GPIO_PIN_6_MSK                                  0x40U
#define GPIO_PIN_7_MSK                                  0x80U
#define GPIO_PIN_8_MSK                                  0x100U
#define GPIO_PIN_9_MSK                                  0x200U
#define GPIO_PIN_10_MSK                                 0x400U
#define GPIO_PIN_11_MSK                                 0x800U
#define GPIO_PIN_12_MSK                                 0x1000U
#define GPIO_PIN_13_MSK                                 0x2000U
#define GPIO_PIN_14_MSK                                 0x4000U
#define GPIO_PIN_15_MSK                                 0x8000U
#define GPIO_PIN_All_MSK                                0xFFFFU

// GPIO Pins Number definitions
#define GPIO_PIN_0_NUM                                  0x0U
#define GPIO_PIN_1_NUM                                  0x1U
#define GPIO_PIN_2_NUM                                  0x2U
#define GPIO_PIN_3_NUM                                  0x3U
#define GPIO_PIN_4_NUM                                  0x4U
#define GPIO_PIN_5_NUM                                  0x5U
#define GPIO_PIN_6_NUM                                  0x6U
#define GPIO_PIN_7_NUM                                  0x7U
#define GPIO_PIN_8_NUM                                  0x8U
#define GPIO_PIN_9_NUM                                  0x9U
#define GPIO_PIN_10_NUM                                 0xAU
#define GPIO_PIN_11_NUM                                 0xBU
#define GPIO_PIN_12_NUM                                 0xCU
#define GPIO_PIN_13_NUM                                 0xDU
#define GPIO_PIN_14_NUM                                 0xEU
#define GPIO_PIN_15_NUM                                 0xFU

// GPIO : CRL
#define GPIO_CRL_MODEx__Output_2M_GP_PP                 0x2U
#define GPIO_CRL_MODEx__Output_2M_GP_OD                 0x6U
#define GPIO_CRL_MODEx__Output_2M_AF_PP                 0xAU
#define GPIO_CRL_MODEx__Output_2M_AF_OD                 0xEU
#define GPIO_CRL_MODEx__Output_10M_GP_PP                0x1U
#define GPIO_CRL_MODEx__Output_10M_GP_OD                0x5U
#define GPIO_CRL_MODEx__Output_10M_AF_PP                0x9U
#define GPIO_CRL_MODEx__Output_10M_AF_OD                0xDU
#define GPIO_CRL_MODEx__Output_50M_GP_PP                0x3U
#define GPIO_CRL_MODEx__Output_50M_GP_OD                0x7U
#define GPIO_CRL_MODEx__Output_50M_AF_PP                0xBU
#define GPIO_CRL_MODEx__Output_50M_AF_OD                0xFU
#define GPIO_CRL_MODEx__Input_Float                     0x4U
#define GPIO_CRL_MODEx__Input_PuPd                      0x8U
#define GPIO_CRL_MODEx__Input_Analog                    0x0U

// GPIO : CRH
#define GPIO_CRH_MODEx__Output_2M_GP_PP                 0x2U
#define GPIO_CRH_MODEx__Output_2M_GP_OD                 0x6U
#define GPIO_CRH_MODEx__Output_2M_AF_PP                 0xAU
#define GPIO_CRH_MODEx__Output_2M_AF_OD                 0xEU
#define GPIO_CRH_MODEx__Output_10M_GP_PP                0x1U
#define GPIO_CRH_MODEx__Output_10M_GP_OD                0x5U
#define GPIO_CRH_MODEx__Output_10M_AF_PP                0x9U
#define GPIO_CRH_MODEx__Output_10M_AF_OD                0xDU
#define GPIO_CRH_MODEx__Output_50M_GP_PP                0x3U
#define GPIO_CRH_MODEx__Output_50M_GP_OD                0x7U
#define GPIO_CRH_MODEx__Output_50M_AF_PP                0xBU
#define GPIO_CRH_MODEx__Output_50M_AF_OD                0xFU
#define GPIO_CRH_MODEx__Input_Float                     0x4U
#define GPIO_CRH_MODEx__Input_PuPd                      0x8U
#define GPIO_CRH_MODEx__Input_Analog                    0x0U

// GPIO : IDR
#define GPIO_IDR_IDRx__PinIsReset                       0x0U
#define GPIO_IDR_IDRx__PinIsSet                         0x1U

// GPIO : ODR
#define GPIO_ODR_ODRx__PullUp                           0x1U
#define GPIO_ODR_ODRx__PullDown                         0x0U
#define GPIO_ODR_ODRx__PinSet                           0x1U
#define GPIO_ODR_ODRx__PinReset                         0x0U

// GPIO : BSRR
#define GPIO_BSRR_BSx__PinSet                           0x1U
#define GPIO_BSRR_BRx__PinReset                         0x1U

// GPIO : BRR
#define GPIO_BRR_BRx__PinReset                          0x1U

// GPIO : LCKR
#define GPIO_LCKR_LCKy__PortConfigNotLocked             0x0U
#define GPIO_LCKR_LCKy__PortConfigLocked                0x1U

#define GPIO_LCKR_LCKK__PortConfigLockKeyNotActive      0x0U
#define GPIO_LCKR_LCKK__PortConfigLockKeyActive         0x1U

/******************************************************************************
 ******     AFIO
******************************************************************************/

// AFIO : EVCR
#define AFIO_EVCR_EVOE__CortexEventOut_Disable          0x0U
#define AFIO_EVCR_EVOE__CortexEventOut_Enable           0x1U

#define AFIO_EVCR_PORT__CortexEventOut_PA               0x0U
#define AFIO_EVCR_PORT__CortexEventOut_PB               0x1U
#define AFIO_EVCR_PORT__CortexEventOut_PC               0x2U
#define AFIO_EVCR_PORT__CortexEventOut_PD               0x3U
#define AFIO_EVCR_PORT__CortexEventOut_PE               0x4U

#define AFIO_EVCR_PIN__CortexEventOut_Px0               0x0U
#define AFIO_EVCR_PIN__CortexEventOut_Px1               0x1U
#define AFIO_EVCR_PIN__CortexEventOut_Px2               0x2U
#define AFIO_EVCR_PIN__CortexEventOut_Px3               0x3U
#define AFIO_EVCR_PIN__CortexEventOut_Px4               0x4U
#define AFIO_EVCR_PIN__CortexEventOut_Px5               0x5U
#define AFIO_EVCR_PIN__CortexEventOut_Px6               0x6U
#define AFIO_EVCR_PIN__CortexEventOut_Px7               0x7U
#define AFIO_EVCR_PIN__CortexEventOut_Px8               0x8U
#define AFIO_EVCR_PIN__CortexEventOut_Px9               0x9U
#define AFIO_EVCR_PIN__CortexEventOut_Px10              0xAU
#define AFIO_EVCR_PIN__CortexEventOut_Px11              0xBU
#define AFIO_EVCR_PIN__CortexEventOut_Px12              0xCU
#define AFIO_EVCR_PIN__CortexEventOut_Px13              0xDU
#define AFIO_EVCR_PIN__CortexEventOut_Px14              0xEU
#define AFIO_EVCR_PIN__CortexEventOut_Px15              0xFU

// AFIO : MAPR
#define AFIO_MAPR_SPI1_REMAP__NoReMap_NSS_PA4_SCK_PA5_MISO_PA6_MOSI_PA7   0x0U
#define AFIO_MAPR_SPI1_REMAP__ReMap_NSS_PA15_SCK_PB3_MISO_PB4_MOSI_PB5    0x1U

#define AFIO_MAPR_I2C1_REMAP__NoReMap_SCL_PB6_SDA_PB7   0x0U
#define AFIO_MAPR_I2C1_REMAP__ReMap_SCL_PB8_SDA_PB9     0x1U

#define AFIO_MAPR_USART1_REMAP__NoReMap_TX_PA9_RX_PA10  0x0U
#define AFIO_MAPR_USART1_REMAP__ReMap_TX_PB6_RX_PB7     0x1U

#define AFIO_MAPR_USART2_REMAP__NoReMap_CTS_PA0_RTS_PA1_TX_PA2_RX_PA3_CK_PA4            0x0U
#define AFIO_MAPR_USART2_REMAP__ReMap_CTS_PD3_RTS_PD4_TX_PD5_RX_PD6_CK_PD7              0x1U

#define AFIO_MAPR_USART3_REMAP__NoReMap_TX_PB10_RX_PB11_CK_PB12_CTS_PB13_RTS_PB14       0x0U
#define AFIO_MAPR_USART3_REMAP__PartialReMap_TX_PC10_RX_PC11_CK_PC12_CTS_PB13_RTS_PB14  0x1U
#define AFIO_MAPR_USART3_REMAP__FullReMap_TX_PD8_RX_PD9_CK_PD10_CTS_PD11_RTS_PD12       0x3U

#define AFIO_MAPR_TIM1_REMAP__NoReMap_ETR_PA12_CH1_PA8_CH2_PA9_CH3_PA10_CH4_PA11_BKIN_PB12_CH1N_PB13_CH2N_PB14_CH3N_PB15    0x0U
#define AFIO_MAPR_TIM1_REMAP__PartialReMap_ETR_PA12_CH1_PA8_CH2_PA9_CH3_PA10_CH4_PA11_BKIN_PA6_CH1N_PA7_CH2N_PB0_CH3N_PB1   0x1U
#define AFIO_MAPR_TIM1_REMAP__FullReMap_ETR_PE7_CH1_PE9_CH2_PE11_CH3_PE13_CH4_PE14_BKIN_PE15_CH1N_PE8_CH2N_PE10_CH3N_PE12   0x3U

#define AFIO_MAPR_TIM2_REMAP__NoReMap_CH1_ETR_PA0_CH2_PA1_CH3_PA2_CH4_PA3               0x0U
#define AFIO_MAPR_TIM2_REMAP__PartialReMap_CH1_ETR_PA15_CH2_PB3_CH3_PA2_CH4_PA3         0x1U
#define AFIO_MAPR_TIM2_REMAP__PartialReMap_CH1_ETR_PA0_CH2_PA1_CH3_PB10_CH4_PB11        0x2U
#define AFIO_MAPR_TIM2_REMAP__FullReMap_CH1_ETR_PA15_CH2_PB3_CH3_PB10_CH4_PB11          0x3U

#define AFIO_MAPR_TIM3_REMAP__NoReMap_ETR_PE0_CH1_PA6_CH2_PA7_CH3_PB0_CH4_PB1           0x0U
#define AFIO_MAPR_TIM3_REMAP__PartialReMap_ETR_PE0_CH1_PB4_CH2_PB5_CH3_PB0_CH4_PB1      0x2U
#define AFIO_MAPR_TIM3_REMAP__FullReMap_ETR_PE0_CH1_PC6_CH2_PC7_CH3_PC8_CH4_PC9         0x3U

#define AFIO_MAPR_TIM4_REMAP__NoReMap_ETR_PE0_CH1_PB6_CH2_PB7_CH3_PB8_CH4_PB9           0x0U
#define AFIO_MAPR_TIM4_REMAP__FullReMap_ETR_PE0_CH1_PD12_CH2_PD13_CH3_PD14_CH4_PD15     0x1U

#define AFIO_MAPR_CAN_REMAP__RX_PA11_TX_PA12            0x0U
#define AFIO_MAPR_CAN_REMAP__RX_PB8_TX_PB9              0x2U
#define AFIO_MAPR_CAN_REMAP__RX_PD0_TX_PD1              0x3U

#define AFIO_MAPR_PD01_REMAP__NoReMap                   0x0U
#define AFIO_MAPR_PD01_REMAP__PD0_OSC_IN_PD1_OSC_OUT    0x1U

#define AFIO_MAPR_TIM5CH4_IREMAP__CH4_PA0               0x0U
#define AFIO_MAPR_TIM4CH4_IREMAP__CH4_LSI               0x1U

#define AFIO_MAPR_ADC1_ETRGINJ__ADC1_Injected_Trigger_EXTI15      0x0U
#define AFIO_MAPR_ADC1_ETRGINJ__ADC1_Injected_Trigger_TIM8_CH4    0x1U

#define AFIO_MAPR_ADC1_ETRGREG__ADC1_Regular_Trigger_EXTI11       0x0U
#define AFIO_MAPR_ADC1_ETRGREG__ADC1_Regular_Trigger_TIM8_TRGO    0x1U

#define AFIO_MAPR_ADC2_ETRGINJ__ADC2_Injected_Trigger_EXTI15      0x0U
#define AFIO_MAPR_ADC2_ETRGINJ__ADC2_Injected_Trigger_TIM8_CH4    0x1U

#define AFIO_MAPR_ADC2_ETRGREG__ADC2_Regular_Trigger_EXTI11       0x0U
#define AFIO_MAPR_ADC2_ETRGREG__ADC2_Regular_Trigger_TIM8_TRGO    0x1U

#define AFIO_MAPR_SWJ_CFG__FullSWJ_ResetState           0x0U
#define AFIO_MAPR_SWJ_CFG__FullSWJ_Without_NJTRST       0x1U
#define AFIO_MAPR_SWJ_CFG__JTAG_Disable_SW_Enable       0x2U
#define AFIO_MAPR_SWJ_CFG__JTAG_Disable_SW_Disable      0x4U

// AFIO : EXTICR1
#define AFIO_EXTICR1_EXTI0__PA0                         0x0U
#define AFIO_EXTICR1_EXTI0__PB0                         0x1U
#define AFIO_EXTICR1_EXTI0__PC0                         0x2U
#define AFIO_EXTICR1_EXTI0__PD0                         0x3U
#define AFIO_EXTICR1_EXTI0__PE0                         0x4U
#define AFIO_EXTICR1_EXTI0__PF0                         0x5U
#define AFIO_EXTICR1_EXTI0__PG0                         0x6U

#define AFIO_EXTICR1_EXTI1__PA1                         0x0U
#define AFIO_EXTICR1_EXTI1__PB1                         0x1U
#define AFIO_EXTICR1_EXTI1__PC1                         0x2U
#define AFIO_EXTICR1_EXTI1__PD1                         0x3U
#define AFIO_EXTICR1_EXTI1__PE1                         0x4U
#define AFIO_EXTICR1_EXTI1__PF1                         0x5U
#define AFIO_EXTICR1_EXTI1__PG1                         0x6U

#define AFIO_EXTICR1_EXTI2__PA2                         0x0U
#define AFIO_EXTICR1_EXTI2__PB2                         0x1U
#define AFIO_EXTICR1_EXTI2__PC2                         0x2U
#define AFIO_EXTICR1_EXTI2__PD2                         0x3U
#define AFIO_EXTICR1_EXTI2__PE2                         0x4U
#define AFIO_EXTICR1_EXTI2__PF2                         0x5U
#define AFIO_EXTICR1_EXTI2__PG2                         0x6U

#define AFIO_EXTICR1_EXTI3__PA3                         0x0U
#define AFIO_EXTICR1_EXTI3__PB3                         0x1U
#define AFIO_EXTICR1_EXTI3__PC3                         0x2U
#define AFIO_EXTICR1_EXTI3__PD3                         0x3U
#define AFIO_EXTICR1_EXTI3__PE3                         0x4U
#define AFIO_EXTICR1_EXTI3__PF3                         0x5U
#define AFIO_EXTICR1_EXTI3__PG3                         0x6U

// AFIO : EXTICR2
#define AFIO_EXTICR2_EXTI4__PA4                         0x0U
#define AFIO_EXTICR2_EXTI4__PB4                         0x1U
#define AFIO_EXTICR2_EXTI4__PC4                         0x2U
#define AFIO_EXTICR2_EXTI4__PD4                         0x3U
#define AFIO_EXTICR2_EXTI4__PE4                         0x4U
#define AFIO_EXTICR2_EXTI4__PF4                         0x5U
#define AFIO_EXTICR2_EXTI4__PG4                         0x6U

#define AFIO_EXTICR2_EXTI5__PA5                         0x0U
#define AFIO_EXTICR2_EXTI5__PB5                         0x1U
#define AFIO_EXTICR2_EXTI5__PC5                         0x2U
#define AFIO_EXTICR2_EXTI5__PD5                         0x3U
#define AFIO_EXTICR2_EXTI5__PE5                         0x4U
#define AFIO_EXTICR2_EXTI5__PF5                         0x5U
#define AFIO_EXTICR2_EXTI5__PG5                         0x6U

#define AFIO_EXTICR2_EXTI6__PA6                         0x0U
#define AFIO_EXTICR2_EXTI6__PB6                         0x1U
#define AFIO_EXTICR2_EXTI6__PC6                         0x2U
#define AFIO_EXTICR2_EXTI6__PD6                         0x3U
#define AFIO_EXTICR2_EXTI6__PE6                         0x4U
#define AFIO_EXTICR2_EXTI6__PF6                         0x5U
#define AFIO_EXTICR2_EXTI6__PG6                         0x6U

#define AFIO_EXTICR2_EXTI7__PA7                         0x0U
#define AFIO_EXTICR2_EXTI7__PB7                         0x1U
#define AFIO_EXTICR2_EXTI7__PC7                         0x2U
#define AFIO_EXTICR2_EXTI7__PD7                         0x3U
#define AFIO_EXTICR2_EXTI7__PE7                         0x4U
#define AFIO_EXTICR2_EXTI7__PF7                         0x5U
#define AFIO_EXTICR2_EXTI7__PG7                         0x6U

// AFIO : EXTICR3
#define AFIO_EXTICR3_EXTI8__PA8                         0x0U
#define AFIO_EXTICR3_EXTI8__PB8                         0x1U
#define AFIO_EXTICR3_EXTI8__PC8                         0x2U
#define AFIO_EXTICR3_EXTI8__PD8                         0x3U
#define AFIO_EXTICR3_EXTI8__PE8                         0x4U
#define AFIO_EXTICR3_EXTI8__PF8                         0x5U
#define AFIO_EXTICR3_EXTI8__PG8                         0x6U

#define AFIO_EXTICR3_EXTI9__PA9                         0x0U
#define AFIO_EXTICR3_EXTI9__PB9                         0x1U
#define AFIO_EXTICR3_EXTI9__PC9                         0x2U
#define AFIO_EXTICR3_EXTI9__PD9                         0x3U
#define AFIO_EXTICR3_EXTI9__PE9                         0x4U
#define AFIO_EXTICR3_EXTI9__PF9                         0x5U
#define AFIO_EXTICR3_EXTI9__PG9                         0x6U

#define AFIO_EXTICR3_EXTI10__PA10                       0x0U
#define AFIO_EXTICR3_EXTI10__PB10                       0x1U
#define AFIO_EXTICR3_EXTI10__PC10                       0x2U
#define AFIO_EXTICR3_EXTI10__PD10                       0x3U
#define AFIO_EXTICR3_EXTI10__PE10                       0x4U
#define AFIO_EXTICR3_EXTI10__PF10                       0x5U
#define AFIO_EXTICR3_EXTI10__PG10                       0x6U

#define AFIO_EXTICR3_EXTI11__PA11                       0x0U
#define AFIO_EXTICR3_EXTI11__PB11                       0x1U
#define AFIO_EXTICR3_EXTI11__PC11                       0x2U
#define AFIO_EXTICR3_EXTI11__PD11                       0x3U
#define AFIO_EXTICR3_EXTI11__PE11                       0x4U
#define AFIO_EXTICR3_EXTI11__PF11                       0x5U
#define AFIO_EXTICR3_EXTI11__PG11                       0x6U

// AFIO : EXTICR4
#define AFIO_EXTICR4_EXTI12__PA12                       0x0U
#define AFIO_EXTICR4_EXTI12__PB12                       0x1U
#define AFIO_EXTICR4_EXTI12__PC12                       0x2U
#define AFIO_EXTICR4_EXTI12__PD12                       0x3U
#define AFIO_EXTICR4_EXTI12__PE12                       0x4U
#define AFIO_EXTICR4_EXTI12__PF12                       0x5U
#define AFIO_EXTICR4_EXTI12__PG12                       0x6U

#define AFIO_EXTICR4_EXTI13__PA13                       0x0U
#define AFIO_EXTICR4_EXTI13__PB13                       0x1U
#define AFIO_EXTICR4_EXTI13__PC13                       0x2U
#define AFIO_EXTICR4_EXTI13__PD13                       0x3U
#define AFIO_EXTICR4_EXTI13__PE13                       0x4U
#define AFIO_EXTICR4_EXTI13__PF13                       0x5U
#define AFIO_EXTICR4_EXTI13__PG13                       0x6U

#define AFIO_EXTICR4_EXTI14__PA14                       0x0U
#define AFIO_EXTICR4_EXTI14__PB14                       0x1U
#define AFIO_EXTICR4_EXTI14__PC14                       0x2U
#define AFIO_EXTICR4_EXTI14__PD14                       0x3U
#define AFIO_EXTICR4_EXTI14__PE14                       0x4U
#define AFIO_EXTICR4_EXTI14__PF14                       0x5U
#define AFIO_EXTICR4_EXTI14__PG14                       0x6U

#define AFIO_EXTICR4_EXTI15__PA15                       0x0U
#define AFIO_EXTICR4_EXTI15__PB15                       0x1U
#define AFIO_EXTICR4_EXTI15__PC15                       0x2U
#define AFIO_EXTICR4_EXTI15__PD15                       0x3U
#define AFIO_EXTICR4_EXTI15__PE15                       0x4U
#define AFIO_EXTICR4_EXTI15__PF15                       0x5U
#define AFIO_EXTICR4_EXTI15__PG15                       0x6U

// AFIO : MAPR2
#define AFIO_MAPR2_TIM9_REMAP__NoReMap_CH1_PA2_CH2_PA3  0x0U
#define AFIO_MAPR2_TIM9_REMAP__ReMap_CH1_PE5_CH2_PE6    0x1U

#define AFIO_MAPR2_TIM10_REMAP__NoReMap_CH1_PB8         0x0U
#define AFIO_MAPR2_TIM10_REMAP__ReMap_CH1_PF6           0x1U

#define AFIO_MAPR2_TIM11_REMAP__NoReMap_CH1_PB9         0x0U
#define AFIO_MAPR2_TIM11_REMAP__ReMap_CH1_PF7           0x1U

#define AFIO_MAPR2_TIM13_REMAP__NoReMap_CH1_PA6         0x0U
#define AFIO_MAPR2_TIM13_REMAP__ReMap_CH1_PF8           0x1U

#define AFIO_MAPR2_TIM14_REMAP__NoReMap_CH1_PA7         0x0U
#define AFIO_MAPR2_TIM14_REMAP__ReMap_CH1_PF9           0x1U

#define AFIO_MAPR2_FSMC_NADV__NADV_ConnectedToOutput    0x0U
#define AFIO_MAPR2_FSMC_NADV__NADV_NotConnected         0x1U

/******************************************************************************
 ******     EXTI
******************************************************************************/

// EXTI : IMR
#define EXTI_IMR_MRx__IntrptEnbl                        0x1U
#define EXTI_IMR_MRx__IntrptDsbl                        0x0U

// EXTI : EMR
#define EXTI_EMR_MRx__Event_Enable                      0x1U
#define EXTI_EMR_MRx__Event_Disable                     0x0U

// EXTI : PTSR
#define EXTI_RTSR_TRx__RisingTrigger_Enalbe             0x1U
#define EXTI_RTSR_TRx__RisingTrigger_Disable            0x0U

// EXTI : FTSR
#define EXTI_FTSR_TRx__FallingTrigger_Enable            0x1U
#define EXTI_FTSR_TRx__FallingTrigger_Disable           0x0U

// EXTI : SWIER
#define EXTI_SWIER_SWIERx__Generate_Interrupt           0x1U
#define EXTI_SWIER_SWIERx__ReadyToGenerate_Interrupt    0x0U

// EXTI : PR
#define EXTI_PR_PRx__No_TriggerRequest                  0x0U
#define EXTI_PR_PRx__InterruptPending                   0x1U
#define EXTI_PR_PRx__ClearPending                       0x1U

/******************************************************************************
 ******     ADC
******************************************************************************/

// ADC : SR
#define ADC_SR_AWD__AWD_FlgClr                          0x0U
#define ADC_SR_AWD__WatchdogEvent_NotOccurred           0x0U
#define ADC_SR_AWD__WatchdogEvent_Occurred              0x1U

#define ADC_SR_EOC__EOC_FlgClr                          0x0U
#define ADC_SR_EOC__RegularConv_NotCmplt                0x0U
#define ADC_SR_EOC__RegularConv_Cmplt                   0x1U

#define ADC_SR_JEOC__JEOC_FlgClr                        0x0U
#define ADC_SR_JEOC__InjectedConv_NotCmplt              0x0U
#define ADC_SR_JEOC__InjectedConv_Cmplt                 0x1U

#define ADC_SR_JSTRT__JSTART_FlgClr                     0x0U
#define ADC_SR_JSTRT__InjectedChannel_NotStarted        0x0U
#define ADC_SR_JSTRT__InjectedChannel_Started           0x1U

#define ADC_SR_START__START_FlgClr                      0x0U
#define ADC_SR_START__RegularChannel_NotStarted         0x0U
#define ADC_SR_START__RegularChannel_Started            0x1U

// ADC : CR1
#define ADC_CR1_AWDCH__WatchdogOn_CH0                   0x0U
#define ADC_CR1_AWDCH__WatchdogOn_CH1                   0x1U
#define ADC_CR1_AWDCH__WatchdogOn_CH2                   0x2U
#define ADC_CR1_AWDCH__WatchdogOn_CH3                   0x3U
#define ADC_CR1_AWDCH__WatchdogOn_CH4                   0x4U
#define ADC_CR1_AWDCH__WatchdogOn_CH5                   0x5U
#define ADC_CR1_AWDCH__WatchdogOn_CH6                   0x6U
#define ADC_CR1_AWDCH__WatchdogOn_CH7                   0x7U
#define ADC_CR1_AWDCH__WatchdogOn_CH8                   0x8U
#define ADC_CR1_AWDCH__WatchdogOn_CH9                   0x9U
#define ADC_CR1_AWDCH__WatchdogOn_CH10                  0xAU
#define ADC_CR1_AWDCH__WatchdogOn_CH11                  0xBU
#define ADC_CR1_AWDCH__WatchdogOn_CH12                  0xCU
#define ADC_CR1_AWDCH__WatchdogOn_CH13                  0xDU
#define ADC_CR1_AWDCH__WatchdogOn_CH14                  0xEU
#define ADC_CR1_AWDCH__WatchdogOn_CH15                  0xFU
#define ADC_CR1_AWDCH__WatchdogOn_CH16                  0x10U
#define ADC_CR1_AWDCH__WatchdogOn_CH17                  0x11U

#define ADC_CR1_EOCIE__Regular_EndOfConv_IntrptEnbl     0x0U
#define ADC_CR1_EOCIE__Regular_EndOfConv_IntrptDsbl     0x1U

#define ADC_CR1_AWDIE__AnalogWatchDog_IntrptDsbl        0x0U
#define ADC_CR1_AWDIE__AnalogWatchDog_IntrptEnbl        0x1U

#define ADC_CR1_JEOCIE__Injected_EndOfConv_IntrptDsbl   0x0U
#define ADC_CR1_JEOCIE__Injected_EndOfConv_IntrptEnbl   0x1U

#define ADC_CR1_SCAN__ScanMode_Disable                  0x0U
#define ADC_CR1_SCAN__ScanMode_Enable                   0x1U

#define ADC_CR1_AWDSGL__AnalogWatchDog_AllChannels      0x0U
#define ADC_CR1_AWDSGL__AnalogWatchDog_SingleChannel    0x1U

#define ADC_CR1_JAUTO__AutomaticInjectedConv_Disable    0x0U
#define ADC_CR1_JAUTO__AutomaticInjectedConv_Enable     0x1U

#define ADC_CR1_DISCEN__Regular_DiscontMode_Disable     0x0U
#define ADC_CR1_DISCEN__Regular_DiscontMode_Enable      0x1U

#define ADC_CR1_JDISCEN__Injected_DiscontMode_Disable   0x0U
#define ADC_CR1_JDISCEN__Injected_DiscontMode_Enable    0x1U

#define ADC_CR1_DISCNUM__DiscontMode_ChannelCount_1     0x0U
#define ADC_CR1_DISCNUM__DiscontMode_ChannelCount_2     0x1U
#define ADC_CR1_DISCNUM__DiscontMode_ChannelCount_3     0x2U
#define ADC_CR1_DISCNUM__DiscontMode_ChannelCount_4     0x3U
#define ADC_CR1_DISCNUM__DiscontMode_ChannelCount_5     0x4U
#define ADC_CR1_DISCNUM__DiscontMode_ChannelCount_6     0x5U
#define ADC_CR1_DISCNUM__DiscontMode_ChannelCount_7     0x6U
#define ADC_CR1_DISCNUM__DiscontMode_ChannelCount_8     0x7U

#define ADC_CR1_DUALMOD__IndependentMode                0x0U
#define ADC_CR1_DUALMOD__RegularSimul_InjectedSimul     0x1U
#define ADC_CR1_DUALMOD__RegularSimul_AlternateTrigger  0x2U
#define ADC_CR1_DUALMOD__InjectedSimul_FastInterleaved  0x3U
#define ADC_CR1_DUALMOD__InjectedSimul_SlowInterleaved  0x4U
#define ADC_CR1_DUALMOD__InjectedSimulModeOnly          0x5U
#define ADC_CR1_DUALMOD__RegularSimulModeOnly           0x6U
#define ADC_CR1_DUALMOD__FastInterleavedModeOnly        0x7U
#define ADC_CR1_DUALMOD__SlowInterleavedModeOnly        0x8U
#define ADC_CR1_DUALMOD__AlternateTriggerModeOnly       0x9U

#define ADC_CR1_JAWDEN__Injected_AnalogWatchDog_Disable 0x0U
#define ADC_CR1_JAWDEN__Injected_AnalogWatchDog_Enable  0x1U

#define ADC_CR1_AWDEN__Regular_AnalogWatchDog_Disable   0x0U
#define ADC_CR1_AWDEN__Regular_AnalogWatchDog_Enable    0x1U

// ADC : CR2
#define ADC_CR2_ADON__ADC_Disable_PowerDownMode         0x0U
#define ADC_CR2_ADON__ADC_Enable_StartConv              0x1U

#define ADC_CR2_CONT__SingleConvMode                    0x0U
#define ADC_CR2_CONT__ContinuousConvMode                0x1U

#define ADC_CR2_CAL__CalibrationCmpltd                  0x0U
#define ADC_CR2_CAL__EnableCalibration                  0x1U

#define ADC_CR2_RSTCAL__CalibrationRegisterInitialized  0x0U
#define ADC_CR2_RSTCAL__InitializeCalibrationRegister   0x1U

#define ADC_CR2_DMA__DMA_Disable                        0x0U
#define ADC_CR2_DMA__DMA_Enable                         0x1U

#define ADC_CR2_ALIGN__Alignment_Right                  0x0U
#define ADC_CR2_ALIGN__Alignment_Left                   0x1U

#define ADC1_CR2_JEXTSEL__InjectedEvent_TIM1_TRGO       0x0U
#define ADC1_CR2_JEXTSEL__InjectedEvent_TIM1_CC4        0x1U
#define ADC1_CR2_JEXTSEL__InjectedEvent_TIM2_TRGO       0x2U
#define ADC1_CR2_JEXTSEL__InjectedEvent_TIM2_CC1        0x3U
#define ADC1_CR2_JEXTSEL__InjectedEvent_TIM3_CC4        0x4U
#define ADC1_CR2_JEXTSEL__InjectedEvent_TIM4_TRGO       0x5U
#define ADC1_CR2_JEXTSEL__InjectedEvent_EXTI15_TIM8_CC4 0x6U
#define ADC1_CR2_JEXTSEL__InjectedEvent_JSWSTART        0x7U

#define ADC2_CR2_JEXTSEL__InjectedEvent_TIM1_TRGO       0x0U
#define ADC2_CR2_JEXTSEL__InjectedEvent_TIM1_CC4        0x1U
#define ADC2_CR2_JEXTSEL__InjectedEvent_TIM2_TRGO       0x2U
#define ADC2_CR2_JEXTSEL__InjectedEvent_TIM2_CC1        0x3U
#define ADC2_CR2_JEXTSEL__InjectedEvent_TIM3_CC4        0x4U
#define ADC2_CR2_JEXTSEL__InjectedEvent_TIM4_TRGO       0x5U
#define ADC2_CR2_JEXTSEL__InjectedEvent_EXTI15_TIM8_CC4 0x6U
#define ADC2_CR2_JEXTSEL__InjectedEvent_JSWSTART        0x7U

#define ADC3_CR2_JEXTSEL__InjectedEvent_TIM1_TRGO       0x0U
#define ADC3_CR2_JEXTSEL__InjectedEvent_TIM1_CC4        0x1U
#define ADC3_CR2_JEXTSEL__InjectedEvent_TIM4_CC3        0x2U
#define ADC3_CR2_JEXTSEL__InjectedEvent_TIM8_CC2        0x3U
#define ADC3_CR2_JEXTSEL__InjectedEvent_TIM8_CC4        0x4U
#define ADC3_CR2_JEXTSEL__InjectedEvent_TIM5_TRGO       0x5U
#define ADC3_CR2_JEXTSEL__InjectedEvent_TIM5_CC4        0x6U
#define ADC3_CR2_JEXTSEL__InjectedEvent_JSWSTART        0x7U

#define ADC_CR2_JEXTTRIG__InjectedConvOnEvent_Disable   0x0U
#define ADC_CR2_JEXTTRIG__InjectedConvOnEvent_Enable    0x1U

#define ADC1_CR2_EXTSEL__RegularEvent_TIM1_CC1          0x0U
#define ADC1_CR2_EXTSEL__RegularEvent_TIM1_CC2          0x1U
#define ADC1_CR2_EXTSEL__RegularEvent_TIM1_CC3          0x2U
#define ADC1_CR2_EXTSEL__RegularEvent_TIM2_CC2          0x3U
#define ADC1_CR2_EXTSEL__RegularEvent_TIM3_TRGO         0x4U
#define ADC1_CR2_EXTSEL__RegularEvent_TIM4_CC4          0x5U
#define ADC1_CR2_EXTSEL__RegularEvent_EXTI11_TIM8_TRGO  0x6U
#define ADC1_CR2_EXTSEL__RegularEvent_SWSTART           0x7U

#define ADC2_CR2_EXTSEL__RegularEvent_TIM1_CC1          0x0U
#define ADC2_CR2_EXTSEL__RegularEvent_TIM1_CC2          0x1U
#define ADC2_CR2_EXTSEL__RegularEvent_TIM1_CC3          0x2U
#define ADC2_CR2_EXTSEL__RegularEvent_TIM2_CC2          0x3U
#define ADC2_CR2_EXTSEL__RegularEvent_TIM3_TRGO         0x4U
#define ADC2_CR2_EXTSEL__RegularEvent_TIM4_CC4          0x5U
#define ADC2_CR2_EXTSEL__RegularEvent_EXTI11_TIM8_TRGO  0x6U
#define ADC2_CR2_EXTSEL__RegularEvent_SWSTART           0x7U

#define ADC3_CR2_EXTSEL__RegularEvent_TIM3_CC1          0x0U
#define ADC3_CR2_EXTSEL__RegularEvent_TIM2_CC3          0x1U
#define ADC3_CR2_EXTSEL__RegularEvent_TIM1_CC3          0x2U
#define ADC3_CR2_EXTSEL__RegularEvent_TIM8_CC1          0x3U
#define ADC3_CR2_EXTSEL__RegularEvent_TIM8_TRGO         0x4U
#define ADC3_CR2_EXTSEL__RegularEvent_TIM5_CC1          0x5U
#define ADC3_CR2_EXTSEL__RegularEvent_TIM5_CC3          0x6U
#define ADC3_CR2_EXTSEL__RegularEvent_SWSTART           0x7U

#define ADC_CR2_EXTTRIG__RegularConvOnEvent_Disable     0x0U
#define ADC_CR2_EXTTRIG__RegularConvOnEvent_Enable      0x1U

#define ADC_CR2_JSWSTART__Injected_ResetState           0x0U
#define ADC_CR2_JSWSTART__Injected_StartConversion      0x1U

#define ADC_CR2_SWSTART__Regular_ResetState             0x0U
#define ADC_CR2_SWSTART__Regular_StartConversion        0x1U

#define ADC_CR2_TSVREFE__TMPsensor_Vref_Disable         0x0U
#define ADC_CR2_TSVREFE__TMPsensor_Vref_Enable          0x1U

// ADC : SMPR1
#define ADC_SMPR1_SMP10__CH10_SampleTime_1p5_cycle      0x0U
#define ADC_SMPR1_SMP10__CH10_SampleTime_7p5_cycle      0x1U
#define ADC_SMPR1_SMP10__CH10_SampleTime_13p5_cycle     0x2U
#define ADC_SMPR1_SMP10__CH10_SampleTime_28p5_cycle     0x3U
#define ADC_SMPR1_SMP10__CH10_SampleTime_41p5_cycle     0x4U
#define ADC_SMPR1_SMP10__CH10_SampleTime_55p5_cycle     0x5U
#define ADC_SMPR1_SMP10__CH10_SampleTime_71p5_cycle     0x6U
#define ADC_SMPR1_SMP10__CH10_SampleTime_239p5_cycle    0x7U

#define ADC_SMPR1_SMP11__CH11_SampleTime_1p5_cycle      0x0U
#define ADC_SMPR1_SMP11__CH11_SampleTime_7p5_cycle      0x1U
#define ADC_SMPR1_SMP11__CH11_SampleTime_13p5_cycle     0x2U
#define ADC_SMPR1_SMP11__CH11_SampleTime_28p5_cycle     0x3U
#define ADC_SMPR1_SMP11__CH11_SampleTime_41p5_cycle     0x4U
#define ADC_SMPR1_SMP11__CH11_SampleTime_55p5_cycle     0x5U
#define ADC_SMPR1_SMP11__CH11_SampleTime_71p5_cycle     0x6U
#define ADC_SMPR1_SMP11__CH11_SampleTime_239p5_cycle    0x7U

#define ADC_SMPR1_SMP12__CH12_SampleTime_1p5_cycle      0x0U
#define ADC_SMPR1_SMP12__CH12_SampleTime_7p5_cycle      0x1U
#define ADC_SMPR1_SMP12__CH12_SampleTime_13p5_cycle     0x2U
#define ADC_SMPR1_SMP12__CH12_SampleTime_28p5_cycle     0x3U
#define ADC_SMPR1_SMP12__CH12_SampleTime_41p5_cycle     0x4U
#define ADC_SMPR1_SMP12__CH12_SampleTime_55p5_cycle     0x5U
#define ADC_SMPR1_SMP12__CH12_SampleTime_71p5_cycle     0x6U
#define ADC_SMPR1_SMP12__CH12_SampleTime_239p5_cycle    0x7U

#define ADC_SMPR1_SMP13__CH13_SampleTime_1p5_cycle      0x0U
#define ADC_SMPR1_SMP13__CH13_SampleTime_7p5_cycle      0x1U
#define ADC_SMPR1_SMP13__CH13_SampleTime_13p5_cycle     0x2U
#define ADC_SMPR1_SMP13__CH13_SampleTime_28p5_cycle     0x3U
#define ADC_SMPR1_SMP13__CH13_SampleTime_41p5_cycle     0x4U
#define ADC_SMPR1_SMP13__CH13_SampleTime_55p5_cycle     0x5U
#define ADC_SMPR1_SMP13__CH13_SampleTime_71p5_cycle     0x6U
#define ADC_SMPR1_SMP13__CH13_SampleTime_239p5_cycle    0x7U

#define ADC_SMPR1_SMP14__CH14_SampleTime_1p5_cycle      0x0U
#define ADC_SMPR1_SMP14__CH14_SampleTime_7p5_cycle      0x1U
#define ADC_SMPR1_SMP14__CH14_SampleTime_13p5_cycle     0x2U
#define ADC_SMPR1_SMP14__CH14_SampleTime_28p5_cycle     0x3U
#define ADC_SMPR1_SMP14__CH14_SampleTime_41p5_cycle     0x4U
#define ADC_SMPR1_SMP14__CH14_SampleTime_55p5_cycle     0x5U
#define ADC_SMPR1_SMP14__CH14_SampleTime_71p5_cycle     0x6U
#define ADC_SMPR1_SMP14__CH14_SampleTime_239p5_cycle    0x7U

#define ADC_SMPR1_SMP15__CH15_SampleTime_1p5_cycle      0x0U
#define ADC_SMPR1_SMP15__CH15_SampleTime_7p5_cycle      0x1U
#define ADC_SMPR1_SMP15__CH15_SampleTime_13p5_cycle     0x2U
#define ADC_SMPR1_SMP15__CH15_SampleTime_28p5_cycle     0x3U
#define ADC_SMPR1_SMP15__CH15_SampleTime_41p5_cycle     0x4U
#define ADC_SMPR1_SMP15__CH15_SampleTime_55p5_cycle     0x5U
#define ADC_SMPR1_SMP15__CH15_SampleTime_71p5_cycle     0x6U
#define ADC_SMPR1_SMP15__CH15_SampleTime_239p5_cycle    0x7U

#define ADC_SMPR1_SMP16__CH16_SampleTime_1p5_cycle      0x0U
#define ADC_SMPR1_SMP16__CH16_SampleTime_7p5_cycle      0x1U
#define ADC_SMPR1_SMP16__CH16_SampleTime_13p5_cycle     0x2U
#define ADC_SMPR1_SMP16__CH16_SampleTime_28p5_cycle     0x3U
#define ADC_SMPR1_SMP16__CH16_SampleTime_41p5_cycle     0x4U
#define ADC_SMPR1_SMP16__CH16_SampleTime_55p5_cycle     0x5U
#define ADC_SMPR1_SMP16__CH16_SampleTime_71p5_cycle     0x6U
#define ADC_SMPR1_SMP16__CH16_SampleTime_239p5_cycle    0x7U

#define ADC_SMPR1_SMP17__CH17_SampleTime_1p5_cycle      0x0U
#define ADC_SMPR1_SMP17__CH17_SampleTime_7p5_cycle      0x1U
#define ADC_SMPR1_SMP17__CH17_SampleTime_13p5_cycle     0x2U
#define ADC_SMPR1_SMP17__CH17_SampleTime_28p5_cycle     0x3U
#define ADC_SMPR1_SMP17__CH17_SampleTime_41p5_cycle     0x4U
#define ADC_SMPR1_SMP17__CH17_SampleTime_55p5_cycle     0x5U
#define ADC_SMPR1_SMP17__CH17_SampleTime_71p5_cycle     0x6U
#define ADC_SMPR1_SMP17__CH17_SampleTime_239p5_cycle    0x7U

// ADC : SMPR2
#define ADC_SMPR2_SMP0__CH0_SampleTime_1p5_cycle        0x0U
#define ADC_SMPR2_SMP0__CH0_SampleTime_7p5_cycle        0x1U
#define ADC_SMPR2_SMP0__CH0_SampleTime_13p5_cycle       0x2U
#define ADC_SMPR2_SMP0__CH0_SampleTime_28p5_cycle       0x3U
#define ADC_SMPR2_SMP0__CH0_SampleTime_41p5_cycle       0x4U
#define ADC_SMPR2_SMP0__CH0_SampleTime_55p5_cycle       0x5U
#define ADC_SMPR2_SMP0__CH0_SampleTime_71p5_cycle       0x6U
#define ADC_SMPR2_SMP0__CH0_SampleTime_239p5_cycle      0x7U

#define ADC_SMPR2_SMP1__CH1_SampleTime_1p5_cycle        0x0U
#define ADC_SMPR2_SMP1__CH1_SampleTime_7p5_cycle        0x1U
#define ADC_SMPR2_SMP1__CH1_SampleTime_13p5_cycle       0x2U
#define ADC_SMPR2_SMP1__CH1_SampleTime_28p5_cycle       0x3U
#define ADC_SMPR2_SMP1__CH1_SampleTime_41p5_cycle       0x4U
#define ADC_SMPR2_SMP1__CH1_SampleTime_55p5_cycle       0x5U
#define ADC_SMPR2_SMP1__CH1_SampleTime_71p5_cycle       0x6U
#define ADC_SMPR2_SMP1__CH1_SampleTime_239p5_cycle      0x7U

#define ADC_SMPR2_SMP2__CH2_SampleTime_1p5_cycle        0x0U
#define ADC_SMPR2_SMP2__CH2_SampleTime_7p5_cycle        0x1U
#define ADC_SMPR2_SMP2__CH2_SampleTime_13p5_cycle       0x2U
#define ADC_SMPR2_SMP2__CH2_SampleTime_28p5_cycle       0x3U
#define ADC_SMPR2_SMP2__CH2_SampleTime_41p5_cycle       0x4U
#define ADC_SMPR2_SMP2__CH2_SampleTime_55p5_cycle       0x5U
#define ADC_SMPR2_SMP2__CH2_SampleTime_71p5_cycle       0x6U
#define ADC_SMPR2_SMP2__CH2_SampleTime_239p5_cycle      0x7U

#define ADC_SMPR2_SMP3__CH3_SampleTime_1p5_cycle        0x0U
#define ADC_SMPR2_SMP3__CH3_SampleTime_7p5_cycle        0x1U
#define ADC_SMPR2_SMP3__CH3_SampleTime_13p5_cycle       0x2U
#define ADC_SMPR2_SMP3__CH3_SampleTime_28p5_cycle       0x3U
#define ADC_SMPR2_SMP3__CH3_SampleTime_41p5_cycle       0x4U
#define ADC_SMPR2_SMP3__CH3_SampleTime_55p5_cycle       0x5U
#define ADC_SMPR2_SMP3__CH3_SampleTime_71p5_cycle       0x6U
#define ADC_SMPR2_SMP3__CH3_SampleTime_239p5_cycle      0x7U

#define ADC_SMPR2_SMP4__CH4_SampleTime_1p5_cycle        0x0U
#define ADC_SMPR2_SMP4__CH4_SampleTime_7p5_cycle        0x1U
#define ADC_SMPR2_SMP4__CH4_SampleTime_13p5_cycle       0x2U
#define ADC_SMPR2_SMP4__CH4_SampleTime_28p5_cycle       0x3U
#define ADC_SMPR2_SMP4__CH4_SampleTime_41p5_cycle       0x4U
#define ADC_SMPR2_SMP4__CH4_SampleTime_55p5_cycle       0x5U
#define ADC_SMPR2_SMP4__CH4_SampleTime_71p5_cycle       0x6U
#define ADC_SMPR2_SMP4__CH4_SampleTime_239p5_cycle      0x7U

#define ADC_SMPR2_SMP5__CH5_SampleTime_1p5_cycle        0x0U
#define ADC_SMPR2_SMP5__CH5_SampleTime_7p5_cycle        0x1U
#define ADC_SMPR2_SMP5__CH5_SampleTime_13p5_cycle       0x2U
#define ADC_SMPR2_SMP5__CH5_SampleTime_28p5_cycle       0x3U
#define ADC_SMPR2_SMP5__CH5_SampleTime_41p5_cycle       0x4U
#define ADC_SMPR2_SMP5__CH5_SampleTime_55p5_cycle       0x5U
#define ADC_SMPR2_SMP5__CH5_SampleTime_71p5_cycle       0x6U
#define ADC_SMPR2_SMP5__CH5_SampleTime_239p5_cycle      0x7U

#define ADC_SMPR2_SMP6__CH6_SampleTime_1p5_cycle        0x0U
#define ADC_SMPR2_SMP6__CH6_SampleTime_7p5_cycle        0x1U
#define ADC_SMPR2_SMP6__CH6_SampleTime_13p5_cycle       0x2U
#define ADC_SMPR2_SMP6__CH6_SampleTime_28p5_cycle       0x3U
#define ADC_SMPR2_SMP6__CH6_SampleTime_41p5_cycle       0x4U
#define ADC_SMPR2_SMP6__CH6_SampleTime_55p5_cycle       0x5U
#define ADC_SMPR2_SMP6__CH6_SampleTime_71p5_cycle       0x6U
#define ADC_SMPR2_SMP6__CH6_SampleTime_239p5_cycle      0x7U

#define ADC_SMPR2_SMP7__CH7_SampleTime_1p5_cycle        0x0U
#define ADC_SMPR2_SMP7__CH7_SampleTime_7p5_cycle        0x1U
#define ADC_SMPR2_SMP7__CH7_SampleTime_13p5_cycle       0x2U
#define ADC_SMPR2_SMP7__CH7_SampleTime_28p5_cycle       0x3U
#define ADC_SMPR2_SMP7__CH7_SampleTime_41p5_cycle       0x4U
#define ADC_SMPR2_SMP7__CH7_SampleTime_55p5_cycle       0x5U
#define ADC_SMPR2_SMP7__CH7_SampleTime_71p5_cycle       0x6U
#define ADC_SMPR2_SMP7__CH7_SampleTime_239p5_cycle      0x7U

#define ADC_SMPR2_SMP8__CH8_SampleTime_1p5_cycle        0x0U
#define ADC_SMPR2_SMP8__CH8_SampleTime_7p5_cycle        0x1U
#define ADC_SMPR2_SMP8__CH8_SampleTime_13p5_cycle       0x2U
#define ADC_SMPR2_SMP8__CH8_SampleTime_28p5_cycle       0x3U
#define ADC_SMPR2_SMP8__CH8_SampleTime_41p5_cycle       0x4U
#define ADC_SMPR2_SMP8__CH8_SampleTime_55p5_cycle       0x5U
#define ADC_SMPR2_SMP8__CH8_SampleTime_71p5_cycle       0x6U
#define ADC_SMPR2_SMP8__CH8_SampleTime_239p5_cycle      0x7U

#define ADC_SMPR2_SMP9__CH9_SampleTime_1p5_cycle        0x0U
#define ADC_SMPR2_SMP9__CH9_SampleTime_7p5_cycle        0x1U
#define ADC_SMPR2_SMP9__CH9_SampleTime_13p5_cycle       0x2U
#define ADC_SMPR2_SMP9__CH9_SampleTime_28p5_cycle       0x3U
#define ADC_SMPR2_SMP9__CH9_SampleTime_41p5_cycle       0x4U
#define ADC_SMPR2_SMP9__CH9_SampleTime_55p5_cycle       0x5U
#define ADC_SMPR2_SMP9__CH9_SampleTime_71p5_cycle       0x6U
#define ADC_SMPR2_SMP9__CH9_SampleTime_239p5_cycle      0x7U

// ADC : SQR1
#define ADC_SQR1_L__Regular_Sequence_Length_1_conv      0x0U
#define ADC_SQR1_L__Regular_Sequence_Length_2_conv      0x1U
#define ADC_SQR1_L__Regular_Sequence_Length_3_conv      0x2U
#define ADC_SQR1_L__Regular_Sequence_Length_4_conv      0x3U
#define ADC_SQR1_L__Regular_Sequence_Length_5_conv      0x4U
#define ADC_SQR1_L__Regular_Sequence_Length_6_conv      0x5U
#define ADC_SQR1_L__Regular_Sequence_Length_7_conv      0x6U
#define ADC_SQR1_L__Regular_Sequence_Length_8_conv      0x7U
#define ADC_SQR1_L__Regular_Sequence_Length_9_conv      0x8U
#define ADC_SQR1_L__Regular_Sequence_Length_10_conv     0x9U
#define ADC_SQR1_L__Regular_Sequence_Length_11_conv     0xAU
#define ADC_SQR1_L__Regular_Sequence_Length_12_conv     0xBU
#define ADC_SQR1_L__Regular_Sequence_Length_13_conv     0xCU
#define ADC_SQR1_L__Regular_Sequence_Length_14_conv     0xDU
#define ADC_SQR1_L__Regular_Sequence_Length_15_conv     0xEU
#define ADC_SQR1_L__Regular_Sequence_Length_16_conv     0xFU

#define ADC_SQR1_SQ16__16th_CH0                         0x0U
#define ADC_SQR1_SQ16__16th_CH1                         0x1U
#define ADC_SQR1_SQ16__16th_CH2                         0x2U
#define ADC_SQR1_SQ16__16th_CH3                         0x3U
#define ADC_SQR1_SQ16__16th_CH4                         0x4U
#define ADC_SQR1_SQ16__16th_CH5                         0x5U
#define ADC_SQR1_SQ16__16th_CH6                         0x6U
#define ADC_SQR1_SQ16__16th_CH7                         0x7U
#define ADC_SQR1_SQ16__16th_CH8                         0x8U
#define ADC_SQR1_SQ16__16th_CH9                         0x9U
#define ADC_SQR1_SQ16__16th_CH10                        0xAU
#define ADC_SQR1_SQ16__16th_CH11                        0xBU
#define ADC_SQR1_SQ16__16th_CH12                        0xCU
#define ADC_SQR1_SQ16__16th_CH13                        0xDU
#define ADC_SQR1_SQ16__16th_CH14                        0xEU
#define ADC_SQR1_SQ16__16th_CH15                        0xFU
#define ADC_SQR1_SQ16__16th_CH16                        0x10U
#define ADC_SQR1_SQ16__16th_CH17                        0x11U

#define ADC_SQR1_SQ15__15th_CH0                         0x0U
#define ADC_SQR1_SQ15__15th_CH1                         0x1U
#define ADC_SQR1_SQ15__15th_CH2                         0x2U
#define ADC_SQR1_SQ15__15th_CH3                         0x3U
#define ADC_SQR1_SQ15__15th_CH4                         0x4U
#define ADC_SQR1_SQ15__15th_CH5                         0x5U
#define ADC_SQR1_SQ15__15th_CH6                         0x6U
#define ADC_SQR1_SQ15__15th_CH7                         0x7U
#define ADC_SQR1_SQ15__15th_CH8                         0x8U
#define ADC_SQR1_SQ15__15th_CH9                         0x9U
#define ADC_SQR1_SQ15__15th_CH10                        0xAU
#define ADC_SQR1_SQ15__15th_CH11                        0xBU
#define ADC_SQR1_SQ15__15th_CH12                        0xCU
#define ADC_SQR1_SQ15__15th_CH13                        0xDU
#define ADC_SQR1_SQ15__15th_CH14                        0xEU
#define ADC_SQR1_SQ15__15th_CH15                        0xFU
#define ADC_SQR1_SQ15__15th_CH16                        0x10U
#define ADC_SQR1_SQ15__15th_CH17                        0x11U

#define ADC_SQR1_SQ14__14th_CH0                         0x0U
#define ADC_SQR1_SQ14__14th_CH1                         0x1U
#define ADC_SQR1_SQ14__14th_CH2                         0x2U
#define ADC_SQR1_SQ14__14th_CH3                         0x3U
#define ADC_SQR1_SQ14__14th_CH4                         0x4U
#define ADC_SQR1_SQ14__14th_CH5                         0x5U
#define ADC_SQR1_SQ14__14th_CH6                         0x6U
#define ADC_SQR1_SQ14__14th_CH7                         0x7U
#define ADC_SQR1_SQ14__14th_CH8                         0x8U
#define ADC_SQR1_SQ14__14th_CH9                         0x9U
#define ADC_SQR1_SQ14__14th_CH10                        0xAU
#define ADC_SQR1_SQ14__14th_CH11                        0xBU
#define ADC_SQR1_SQ14__14th_CH12                        0xCU
#define ADC_SQR1_SQ14__14th_CH13                        0xDU
#define ADC_SQR1_SQ14__14th_CH14                        0xEU
#define ADC_SQR1_SQ14__14th_CH15                        0xFU
#define ADC_SQR1_SQ14__14th_CH16                        0x10U
#define ADC_SQR1_SQ14__14th_CH17                        0x11U

#define ADC_SQR1_SQ13__13th_CH0                         0x0U
#define ADC_SQR1_SQ13__13th_CH1                         0x1U
#define ADC_SQR1_SQ13__13th_CH2                         0x2U
#define ADC_SQR1_SQ13__13th_CH3                         0x3U
#define ADC_SQR1_SQ13__13th_CH4                         0x4U
#define ADC_SQR1_SQ13__13th_CH5                         0x5U
#define ADC_SQR1_SQ13__13th_CH6                         0x6U
#define ADC_SQR1_SQ13__13th_CH7                         0x7U
#define ADC_SQR1_SQ13__13th_CH8                         0x8U
#define ADC_SQR1_SQ13__13th_CH9                         0x9U
#define ADC_SQR1_SQ13__13th_CH10                        0xAU
#define ADC_SQR1_SQ13__13th_CH11                        0xBU
#define ADC_SQR1_SQ13__13th_CH12                        0xCU
#define ADC_SQR1_SQ13__13th_CH13                        0xDU
#define ADC_SQR1_SQ13__13th_CH14                        0xEU
#define ADC_SQR1_SQ13__13th_CH15                        0xFU
#define ADC_SQR1_SQ13__13th_CH16                        0x10U
#define ADC_SQR1_SQ13__13th_CH17                        0x11U

// ADC : SQR2
#define ADC_SQR2_SQ12__12th_CH0                         0x0U
#define ADC_SQR2_SQ12__12th_CH1                         0x1U
#define ADC_SQR2_SQ12__12th_CH2                         0x2U
#define ADC_SQR2_SQ12__12th_CH3                         0x3U
#define ADC_SQR2_SQ12__12th_CH4                         0x4U
#define ADC_SQR2_SQ12__12th_CH5                         0x5U
#define ADC_SQR2_SQ12__12th_CH6                         0x6U
#define ADC_SQR2_SQ12__12th_CH7                         0x7U
#define ADC_SQR2_SQ12__12th_CH8                         0x8U
#define ADC_SQR2_SQ12__12th_CH9                         0x9U
#define ADC_SQR2_SQ12__12th_CH10                        0xAU
#define ADC_SQR2_SQ12__12th_CH11                        0xBU
#define ADC_SQR2_SQ12__12th_CH12                        0xCU
#define ADC_SQR2_SQ12__12th_CH13                        0xDU
#define ADC_SQR2_SQ12__12th_CH14                        0xEU
#define ADC_SQR2_SQ12__12th_CH15                        0xFU
#define ADC_SQR2_SQ12__12th_CH16                        0x10U
#define ADC_SQR2_SQ12__12th_CH17                        0x11U

#define ADC_SQR2_SQ11__11th_CH0                         0x0U
#define ADC_SQR2_SQ11__11th_CH1                         0x1U
#define ADC_SQR2_SQ11__11th_CH2                         0x2U
#define ADC_SQR2_SQ11__11th_CH3                         0x3U
#define ADC_SQR2_SQ11__11th_CH4                         0x4U
#define ADC_SQR2_SQ11__11th_CH5                         0x5U
#define ADC_SQR2_SQ11__11th_CH6                         0x6U
#define ADC_SQR2_SQ11__11th_CH7                         0x7U
#define ADC_SQR2_SQ11__11th_CH8                         0x8U
#define ADC_SQR2_SQ11__11th_CH9                         0x9U
#define ADC_SQR2_SQ11__11th_CH10                        0xAU
#define ADC_SQR2_SQ11__11th_CH11                        0xBU
#define ADC_SQR2_SQ11__11th_CH12                        0xCU
#define ADC_SQR2_SQ11__11th_CH13                        0xDU
#define ADC_SQR2_SQ11__11th_CH14                        0xEU
#define ADC_SQR2_SQ11__11th_CH15                        0xFU
#define ADC_SQR2_SQ11__11th_CH16                        0x10U
#define ADC_SQR2_SQ11__11th_CH17                        0x11U

#define ADC_SQR2_SQ10__10th_CH0                         0x0U
#define ADC_SQR2_SQ10__10th_CH1                         0x1U
#define ADC_SQR2_SQ10__10th_CH2                         0x2U
#define ADC_SQR2_SQ10__10th_CH3                         0x3U
#define ADC_SQR2_SQ10__10th_CH4                         0x4U
#define ADC_SQR2_SQ10__10th_CH5                         0x5U
#define ADC_SQR2_SQ10__10th_CH6                         0x6U
#define ADC_SQR2_SQ10__10th_CH7                         0x7U
#define ADC_SQR2_SQ10__10th_CH8                         0x8U
#define ADC_SQR2_SQ10__10th_CH9                         0x9U
#define ADC_SQR2_SQ10__10th_CH10                        0xAU
#define ADC_SQR2_SQ10__10th_CH11                        0xBU
#define ADC_SQR2_SQ10__10th_CH12                        0xCU
#define ADC_SQR2_SQ10__10th_CH13                        0xDU
#define ADC_SQR2_SQ10__10th_CH14                        0xEU
#define ADC_SQR2_SQ10__10th_CH15                        0xFU
#define ADC_SQR2_SQ10__10th_CH16                        0x10U
#define ADC_SQR2_SQ10__10th_CH17                        0x11U

#define ADC_SQR2_SQ9__9th_CH0                           0x0U
#define ADC_SQR2_SQ9__9th_CH1                           0x1U
#define ADC_SQR2_SQ9__9th_CH2                           0x2U
#define ADC_SQR2_SQ9__9th_CH3                           0x3U
#define ADC_SQR2_SQ9__9th_CH4                           0x4U
#define ADC_SQR2_SQ9__9th_CH5                           0x5U
#define ADC_SQR2_SQ9__9th_CH6                           0x6U
#define ADC_SQR2_SQ9__9th_CH7                           0x7U
#define ADC_SQR2_SQ9__9th_CH8                           0x8U
#define ADC_SQR2_SQ9__9th_CH9                           0x9U
#define ADC_SQR2_SQ9__9th_CH10                          0xAU
#define ADC_SQR2_SQ9__9th_CH11                          0xBU
#define ADC_SQR2_SQ9__9th_CH12                          0xCU
#define ADC_SQR2_SQ9__9th_CH13                          0xDU
#define ADC_SQR2_SQ9__9th_CH14                          0xEU
#define ADC_SQR2_SQ9__9th_CH15                          0xFU
#define ADC_SQR2_SQ9__9th_CH16                          0x10U
#define ADC_SQR2_SQ9__9th_CH17                          0x11U

#define ADC_SQR2_SQ8__8th_CH0                           0x0U
#define ADC_SQR2_SQ8__8th_CH1                           0x1U
#define ADC_SQR2_SQ8__8th_CH2                           0x2U
#define ADC_SQR2_SQ8__8th_CH3                           0x3U
#define ADC_SQR2_SQ8__8th_CH4                           0x4U
#define ADC_SQR2_SQ8__8th_CH5                           0x5U
#define ADC_SQR2_SQ8__8th_CH6                           0x6U
#define ADC_SQR2_SQ8__8th_CH7                           0x7U
#define ADC_SQR2_SQ8__8th_CH8                           0x8U
#define ADC_SQR2_SQ8__8th_CH9                           0x9U
#define ADC_SQR2_SQ8__8th_CH10                          0xAU
#define ADC_SQR2_SQ8__8th_CH11                          0xBU
#define ADC_SQR2_SQ8__8th_CH12                          0xCU
#define ADC_SQR2_SQ8__8th_CH13                          0xDU
#define ADC_SQR2_SQ8__8th_CH14                          0xEU
#define ADC_SQR2_SQ8__8th_CH15                          0xFU
#define ADC_SQR2_SQ8__8th_CH16                          0x10U
#define ADC_SQR2_SQ8__8th_CH17                          0x11U

#define ADC_SQR2_SQ7__7th_CH0                           0x0U
#define ADC_SQR2_SQ7__7th_CH1                           0x1U
#define ADC_SQR2_SQ7__7th_CH2                           0x2U
#define ADC_SQR2_SQ7__7th_CH3                           0x3U
#define ADC_SQR2_SQ7__7th_CH4                           0x4U
#define ADC_SQR2_SQ7__7th_CH5                           0x5U
#define ADC_SQR2_SQ7__7th_CH6                           0x6U
#define ADC_SQR2_SQ7__7th_CH7                           0x7U
#define ADC_SQR2_SQ7__7th_CH8                           0x8U
#define ADC_SQR2_SQ7__7th_CH9                           0x9U
#define ADC_SQR2_SQ7__7th_CH10                          0xAU
#define ADC_SQR2_SQ7__7th_CH11                          0xBU
#define ADC_SQR2_SQ7__7th_CH12                          0xCU
#define ADC_SQR2_SQ7__7th_CH13                          0xDU
#define ADC_SQR2_SQ7__7th_CH14                          0xEU
#define ADC_SQR2_SQ7__7th_CH15                          0xFU
#define ADC_SQR2_SQ7__7th_CH16                          0x10U
#define ADC_SQR2_SQ7__7th_CH17                          0x11U

// ADC : SQR3
#define ADC_SQR3_SQ6__6th_CH0                           0x0U
#define ADC_SQR3_SQ6__6th_CH1                           0x1U
#define ADC_SQR3_SQ6__6th_CH2                           0x2U
#define ADC_SQR3_SQ6__6th_CH3                           0x3U
#define ADC_SQR3_SQ6__6th_CH4                           0x4U
#define ADC_SQR3_SQ6__6th_CH5                           0x5U
#define ADC_SQR3_SQ6__6th_CH6                           0x6U
#define ADC_SQR3_SQ6__6th_CH7                           0x7U
#define ADC_SQR3_SQ6__6th_CH8                           0x8U
#define ADC_SQR3_SQ6__6th_CH9                           0x9U
#define ADC_SQR3_SQ6__6th_CH10                          0xAU
#define ADC_SQR3_SQ6__6th_CH11                          0xBU
#define ADC_SQR3_SQ6__6th_CH12                          0xCU
#define ADC_SQR3_SQ6__6th_CH13                          0xDU
#define ADC_SQR3_SQ6__6th_CH14                          0xEU
#define ADC_SQR3_SQ6__6th_CH15                          0xFU
#define ADC_SQR3_SQ6__6th_CH16                          0x10U
#define ADC_SQR3_SQ6__6th_CH17                          0x11U

#define ADC_SQR3_SQ5__5th_CH0                           0x0U
#define ADC_SQR3_SQ5__5th_CH1                           0x1U
#define ADC_SQR3_SQ5__5th_CH2                           0x2U
#define ADC_SQR3_SQ5__5th_CH3                           0x3U
#define ADC_SQR3_SQ5__5th_CH4                           0x4U
#define ADC_SQR3_SQ5__5th_CH5                           0x5U
#define ADC_SQR3_SQ5__5th_CH6                           0x6U
#define ADC_SQR3_SQ5__5th_CH7                           0x7U
#define ADC_SQR3_SQ5__5th_CH8                           0x8U
#define ADC_SQR3_SQ5__5th_CH9                           0x9U
#define ADC_SQR3_SQ5__5th_CH10                          0xAU
#define ADC_SQR3_SQ5__5th_CH11                          0xBU
#define ADC_SQR3_SQ5__5th_CH12                          0xCU
#define ADC_SQR3_SQ5__5th_CH13                          0xDU
#define ADC_SQR3_SQ5__5th_CH14                          0xEU
#define ADC_SQR3_SQ5__5th_CH15                          0xFU
#define ADC_SQR3_SQ5__5th_CH16                          0x10U
#define ADC_SQR3_SQ5__5th_CH17                          0x11U

#define ADC_SQR3_SQ4__4th_CH0                           0x0U
#define ADC_SQR3_SQ4__4th_CH1                           0x1U
#define ADC_SQR3_SQ4__4th_CH2                           0x2U
#define ADC_SQR3_SQ4__4th_CH3                           0x3U
#define ADC_SQR3_SQ4__4th_CH4                           0x4U
#define ADC_SQR3_SQ4__4th_CH5                           0x5U
#define ADC_SQR3_SQ4__4th_CH6                           0x6U
#define ADC_SQR3_SQ4__4th_CH7                           0x7U
#define ADC_SQR3_SQ4__4th_CH8                           0x8U
#define ADC_SQR3_SQ4__4th_CH9                           0x9U
#define ADC_SQR3_SQ4__4th_CH10                          0xAU
#define ADC_SQR3_SQ4__4th_CH11                          0xBU
#define ADC_SQR3_SQ4__4th_CH12                          0xCU
#define ADC_SQR3_SQ4__4th_CH13                          0xDU
#define ADC_SQR3_SQ4__4th_CH14                          0xEU
#define ADC_SQR3_SQ4__4th_CH15                          0xFU
#define ADC_SQR3_SQ4__4th_CH16                          0x10U
#define ADC_SQR3_SQ4__4th_CH17                          0x11U

#define ADC_SQR3_SQ3__3rd_CH0                           0x0U
#define ADC_SQR3_SQ3__3rd_CH1                           0x1U
#define ADC_SQR3_SQ3__3rd_CH2                           0x2U
#define ADC_SQR3_SQ3__3rd_CH3                           0x3U
#define ADC_SQR3_SQ3__3rd_CH4                           0x4U
#define ADC_SQR3_SQ3__3rd_CH5                           0x5U
#define ADC_SQR3_SQ3__3rd_CH6                           0x6U
#define ADC_SQR3_SQ3__3rd_CH7                           0x7U
#define ADC_SQR3_SQ3__3rd_CH8                           0x8U
#define ADC_SQR3_SQ3__3rd_CH9                           0x9U
#define ADC_SQR3_SQ3__3rd_CH10                          0xAU
#define ADC_SQR3_SQ3__3rd_CH11                          0xBU
#define ADC_SQR3_SQ3__3rd_CH12                          0xCU
#define ADC_SQR3_SQ3__3rd_CH13                          0xDU
#define ADC_SQR3_SQ3__3rd_CH14                          0xEU
#define ADC_SQR3_SQ3__3rd_CH15                          0xFU
#define ADC_SQR3_SQ3__3rd_CH16                          0x10U
#define ADC_SQR3_SQ3__3rd_CH17                          0x11U

#define ADC_SQR3_SQ2__2nd_CH0                           0x0U
#define ADC_SQR3_SQ2__2nd_CH1                           0x1U
#define ADC_SQR3_SQ2__2nd_CH2                           0x2U
#define ADC_SQR3_SQ2__2nd_CH3                           0x3U
#define ADC_SQR3_SQ2__2nd_CH4                           0x4U
#define ADC_SQR3_SQ2__2nd_CH5                           0x5U
#define ADC_SQR3_SQ2__2nd_CH6                           0x6U
#define ADC_SQR3_SQ2__2nd_CH7                           0x7U
#define ADC_SQR3_SQ2__2nd_CH8                           0x8U
#define ADC_SQR3_SQ2__2nd_CH9                           0x9U
#define ADC_SQR3_SQ2__2nd_CH10                          0xAU
#define ADC_SQR3_SQ2__2nd_CH11                          0xBU
#define ADC_SQR3_SQ2__2nd_CH12                          0xCU
#define ADC_SQR3_SQ2__2nd_CH13                          0xDU
#define ADC_SQR3_SQ2__2nd_CH14                          0xEU
#define ADC_SQR3_SQ2__2nd_CH15                          0xFU
#define ADC_SQR3_SQ2__2nd_CH16                          0x10U
#define ADC_SQR3_SQ2__2nd_CH17                          0x11U

#define ADC_SQR3_SQ1__1st_CH0                           0x0U
#define ADC_SQR3_SQ1__1st_CH1                           0x1U
#define ADC_SQR3_SQ1__1st_CH2                           0x2U
#define ADC_SQR3_SQ1__1st_CH3                           0x3U
#define ADC_SQR3_SQ1__1st_CH4                           0x4U
#define ADC_SQR3_SQ1__1st_CH5                           0x5U
#define ADC_SQR3_SQ1__1st_CH6                           0x6U
#define ADC_SQR3_SQ1__1st_CH7                           0x7U
#define ADC_SQR3_SQ1__1st_CH8                           0x8U
#define ADC_SQR3_SQ1__1st_CH9                           0x9U
#define ADC_SQR3_SQ1__1st_CH10                          0xAU
#define ADC_SQR3_SQ1__1st_CH11                          0xBU
#define ADC_SQR3_SQ1__1st_CH12                          0xCU
#define ADC_SQR3_SQ1__1st_CH13                          0xDU
#define ADC_SQR3_SQ1__1st_CH14                          0xEU
#define ADC_SQR3_SQ1__1st_CH15                          0xFU
#define ADC_SQR3_SQ1__1st_CH16                          0x10U
#define ADC_SQR3_SQ1__1st_CH17                          0x11U

// ADC : JSQR
#define ADC_JSQR_JL__Injected_Sequence_Length_1_Conv    0x0U
#define ADC_JSQR_JL__Injected_Sequence_Length_2_Conv    0x1U
#define ADC_JSQR_JL__Injected_Sequence_Length_3_Conv    0x2U
#define ADC_JSQR_JL__Injected_Sequence_Length_4_Conv    0x3U

#define ADC_JSQR_JSQ4__4th_CH0                          0x0U
#define ADC_JSQR_JSQ4__4th_CH1                          0x1U
#define ADC_JSQR_JSQ4__4th_CH2                          0x2U
#define ADC_JSQR_JSQ4__4th_CH3                          0x3U
#define ADC_JSQR_JSQ4__4th_CH4                          0x4U
#define ADC_JSQR_JSQ4__4th_CH5                          0x5U
#define ADC_JSQR_JSQ4__4th_CH6                          0x6U
#define ADC_JSQR_JSQ4__4th_CH7                          0x7U
#define ADC_JSQR_JSQ4__4th_CH8                          0x8U
#define ADC_JSQR_JSQ4__4th_CH9                          0x9U
#define ADC_JSQR_JSQ4__4th_CH10                         0xAU
#define ADC_JSQR_JSQ4__4th_CH11                         0xBU
#define ADC_JSQR_JSQ4__4th_CH12                         0xCU
#define ADC_JSQR_JSQ4__4th_CH13                         0xDU
#define ADC_JSQR_JSQ4__4th_CH14                         0xEU
#define ADC_JSQR_JSQ4__4th_CH15                         0xFU
#define ADC_JSQR_JSQ4__4th_CH16                         0x10U
#define ADC_JSQR_JSQ4__4th_CH17                         0x11U

#define ADC_JSQR_JSQ3__3rd_CH0                          0x0U
#define ADC_JSQR_JSQ3__3rd_CH1                          0x1U
#define ADC_JSQR_JSQ3__3rd_CH2                          0x2U
#define ADC_JSQR_JSQ3__3rd_CH3                          0x3U
#define ADC_JSQR_JSQ3__3rd_CH4                          0x4U
#define ADC_JSQR_JSQ3__3rd_CH5                          0x5U
#define ADC_JSQR_JSQ3__3rd_CH6                          0x6U
#define ADC_JSQR_JSQ3__3rd_CH7                          0x7U
#define ADC_JSQR_JSQ3__3rd_CH8                          0x8U
#define ADC_JSQR_JSQ3__3rd_CH9                          0x9U
#define ADC_JSQR_JSQ3__3rd_CH10                         0xAU
#define ADC_JSQR_JSQ3__3rd_CH11                         0xBU
#define ADC_JSQR_JSQ3__3rd_CH12                         0xCU
#define ADC_JSQR_JSQ3__3rd_CH13                         0xDU
#define ADC_JSQR_JSQ3__3rd_CH14                         0xEU
#define ADC_JSQR_JSQ3__3rd_CH15                         0xFU
#define ADC_JSQR_JSQ3__3rd_CH16                         0x10U
#define ADC_JSQR_JSQ3__3rd_CH17                         0x11U

#define ADC_JSQR_JSQ2__2nd_CH0                          0x0U
#define ADC_JSQR_JSQ2__2nd_CH1                          0x1U
#define ADC_JSQR_JSQ2__2nd_CH2                          0x2U
#define ADC_JSQR_JSQ2__2nd_CH3                          0x3U
#define ADC_JSQR_JSQ2__2nd_CH4                          0x4U
#define ADC_JSQR_JSQ2__2nd_CH5                          0x5U
#define ADC_JSQR_JSQ2__2nd_CH6                          0x6U
#define ADC_JSQR_JSQ2__2nd_CH7                          0x7U
#define ADC_JSQR_JSQ2__2nd_CH8                          0x8U
#define ADC_JSQR_JSQ2__2nd_CH9                          0x9U
#define ADC_JSQR_JSQ2__2nd_CH10                         0xAU
#define ADC_JSQR_JSQ2__2nd_CH11                         0xBU
#define ADC_JSQR_JSQ2__2nd_CH12                         0xCU
#define ADC_JSQR_JSQ2__2nd_CH13                         0xDU
#define ADC_JSQR_JSQ2__2nd_CH14                         0xEU
#define ADC_JSQR_JSQ2__2nd_CH15                         0xFU
#define ADC_JSQR_JSQ2__2nd_CH16                         0x10U
#define ADC_JSQR_JSQ2__2nd_CH17                         0x11U

#define ADC_JSQR_JSQ1__1st_CH0                          0x0U
#define ADC_JSQR_JSQ1__1st_CH1                          0x1U
#define ADC_JSQR_JSQ1__1st_CH2                          0x2U
#define ADC_JSQR_JSQ1__1st_CH3                          0x3U
#define ADC_JSQR_JSQ1__1st_CH4                          0x4U
#define ADC_JSQR_JSQ1__1st_CH5                          0x5U
#define ADC_JSQR_JSQ1__1st_CH6                          0x6U
#define ADC_JSQR_JSQ1__1st_CH7                          0x7U
#define ADC_JSQR_JSQ1__1st_CH8                          0x8U
#define ADC_JSQR_JSQ1__1st_CH9                          0x9U
#define ADC_JSQR_JSQ1__1st_CH10                         0xAU
#define ADC_JSQR_JSQ1__1st_CH11                         0xBU
#define ADC_JSQR_JSQ1__1st_CH12                         0xCU
#define ADC_JSQR_JSQ1__1st_CH13                         0xDU
#define ADC_JSQR_JSQ1__1st_CH14                         0xEU
#define ADC_JSQR_JSQ1__1st_CH15                         0xFU
#define ADC_JSQR_JSQ1__1st_CH16                         0x10U
#define ADC_JSQR_JSQ1__1st_CH17                         0x11U

/*****************************************************************************
 ******     DMA
******************************************************************************/

// DMA : ISR
#define DMA_ISR_GIF1__CH1_No_TC_HT_TE_Events            0x0U
#define DMA_ISR_GIF1__CH1_TC_HT_TE_EventOccurred        0x1U

#define DMA_ISR_TCIF1__CH1_TransferCmpltEvent           0x0U
#define DMA_ISR_TCIF1__CH1_TransferCmpltEventOccurred   0x1U

#define DMA_ISR_HTIF1__CH1_NoHalfTransferEvent          0x0U
#define DMA_ISR_HTIF1__CH1_HalfTransferEventOccurred    0x1U

#define DMA_ISR_TEIF1__CH1_NoTransferError              0x0U
#define DMA_ISR_TEIF1__CH1_TransferErrorOccurred        0x1U

#define DMA_ISR_GIF2__CH2_No_TC_HT_TE_Events            0x0U
#define DMA_ISR_GIF2__CH2_TC_HT_TE_EventOccurred        0x1U

#define DMA_ISR_TCIF2__CH2_TransferCmpltEvent           0x0U
#define DMA_ISR_TCIF2__CH2_TransferCmpltEventOccurred   0x1U

#define DMA_ISR_HTIF2__CH2_NoHalfTransferEvent          0x0U
#define DMA_ISR_HTIF2__CH2_HalfTransferEventOccurred    0x1U

#define DMA_ISR_TEIF2__CH2_NoTransferError              0x0U
#define DMA_ISR_TEIF2__CH2_TransferErrorOccurred        0x1U

#define DMA_ISR_GIF3__CH3_No_TC_HT_TE_Events            0x0U
#define DMA_ISR_GIF3__CH3_TC_HT_TE_EventOccurred        0x1U

#define DMA_ISR_TCIF3__CH3_TransferCmpltEvent           0x0U
#define DMA_ISR_TCIF3__CH3_TransferCmpltEventOccurred   0x1U

#define DMA_ISR_HTIF3__CH3_NoHalfTransferEvent          0x0U
#define DMA_ISR_HTIF3__CH3_HalfTransferEventOccurred    0x1U

#define DMA_ISR_TEIF3__CH3_NoTransferError              0x0U
#define DMA_ISR_TEIF3__CH3_TransferErrorOccurred        0x1U

#define DMA_ISR_GIF4__CH4_No_TC_HT_TE_Events            0x0U
#define DMA_ISR_GIF4__CH4_TC_HT_TE_EventOccurred        0x1U

#define DMA_ISR_TCIF4__CH4_TransferCmpltEvent           0x0U
#define DMA_ISR_TCIF4__CH4_TransferCmpltEventOccurred   0x1U

#define DMA_ISR_HTIF4__CH4_NoHalfTransferEvent          0x0U
#define DMA_ISR_HTIF4__CH4_HalfTransferEventOccurred    0x1U

#define DMA_ISR_TEIF4__CH4_NoTransferError              0x0U
#define DMA_ISR_TEIF4__CH4_TransferErrorOccurred        0x1U

#define DMA_ISR_GIF5__CH5_No_TC_HT_TE_Events            0x0U
#define DMA_ISR_GIF5__CH5_TC_HT_TE_EventOccurred        0x1U

#define DMA_ISR_TCIF5__CH5_TransferCmpltEvent           0x0U
#define DMA_ISR_TCIF5__CH5_TransferCmpltEventOccurred   0x1U

#define DMA_ISR_HTIF1__CH1_NoHalfTransferEvent          0x0U
#define DMA_ISR_HTIF1__CH1_HalfTransferEventOccurred    0x1U

#define DMA_ISR_TEIF5__CH5_NoTransferError              0x0U
#define DMA_ISR_TEIF5__CH5_TransferErrorOccurred        0x1U

#define DMA_ISR_GIF6__CH6_No_TC_HT_TE_Events            0x0U
#define DMA_ISR_GIF6__CH6_TC_HT_TE_EventOccurred        0x1U

#define DMA_ISR_TCIF6__CH6_TransferCmpltEvent           0x0U
#define DMA_ISR_TCIF6__CH6_TransferCmpltEventOccurred   0x1U

#define DMA_ISR_HTIF6__CH6_NoHalfTransferEvent          0x0U
#define DMA_ISR_HTIF6__CH6_HalfTransferEventOccurred    0x1U

#define DMA_ISR_TEIF6__CH6_NoTransferError              0x0U
#define DMA_ISR_TEIF6__CH6_TransferErrorOccurred        0x1U

#define DMA_ISR_GIF7__CH7_No_TC_HT_TE_Events            0x0U
#define DMA_ISR_GIF7__CH7_TC_HT_TE_EventOccurred        0x1U

#define DMA_ISR_TCIF7__CH7_TransferCmpltEvent           0x0U
#define DMA_ISR_TCIF7__CH7_TransferCmpltEventOccurred   0x1U

#define DMA_ISR_HTIF7__CH7_NoHalfTransferEvent          0x0U
#define DMA_ISR_HTIF7__CH7_HalfTransferEventOccurred    0x1U

#define DMA_ISR_TEIF7__CH7_NoTransferError              0x0U
#define DMA_ISR_TEIF7__CH7_TransferErrorOccurred        0x1U


// DMA : IFCR
#define DMA_IFCR_CGIF1__GIF_TEIF_HTIF_TCIF_FlgClr       0x1U
#define DMA_IFCR_CTCIF1__TCIF_FlgClr                    0x1U
#define DMA_IFCR_CHTIF1__HTIF_FlgClr                    0x1U
#define DMA_IFCR_CTEIF1__HTIF_FlgClr                    0x1U

#define DMA_IFCR_CGIF2__GIF_TEIF_HTIF_TCIF_FlgClr       0x1U
#define DMA_IFCR_CTCIF2__TCIF_FlgClr                    0x1U
#define DMA_IFCR_CHTIF2__HTIF_FlgClr                    0x1U
#define DMA_IFCR_CTEIF2__HTIF_FlgClr                    0x1U

#define DMA_IFCR_CGIF3__GIF_TEIF_HTIF_TCIF_FlgClr       0x1U
#define DMA_IFCR_CTCIF3__TCIF_FlgClr                    0x1U
#define DMA_IFCR_CHTIF3__HTIF_FlgClr                    0x1U
#define DMA_IFCR_CTEIF3__HTIF_FlgClr                    0x1U

#define DMA_IFCR_CGIF4__GIF_TEIF_HTIF_TCIF_FlgClr       0x1U
#define DMA_IFCR_CTCIF4__TCIF_FlgClr                    0x1U
#define DMA_IFCR_CHTIF4__HTIF_FlgClr                    0x1U
#define DMA_IFCR_CTEIF4__HTIF_FlgClr                    0x1U

#define DMA_IFCR_CGIF5__GIF_TEIF_HTIF_TCIF_FlgClr       0x1U
#define DMA_IFCR_CTCIF5__TCIF_FlgClr                    0x1U
#define DMA_IFCR_CHTIF5__HTIF_FlgClr                    0x1U
#define DMA_IFCR_CTEIF5__HTIF_FlgClr                    0x1U

#define DMA_IFCR_CGIF6__GIF_TEIF_HTIF_TCIF_FlgClr       0x1U
#define DMA_IFCR_CTCIF6__TCIF_FlgClr                    0x1U
#define DMA_IFCR_CHTIF6__HTIF_FlgClr                    0x1U
#define DMA_IFCR_CTEIF6__HTIF_FlgClr                    0x1U

#define DMA_IFCR_CGIF7__GIF_TEIF_HTIF_TCIF_FlgClr       0x1U
#define DMA_IFCR_CTCIF7__TCIF_FlgClr                    0x1U
#define DMA_IFCR_CHTIF7__HTIF_FlgClr                    0x1U
#define DMA_IFCR_CTEIF7__HTIF_FlgClr                    0x1U


// DMA : CCR1
#define DMA_CCR1_EN__CH1_ChannelEnable                  0x1U
#define DMA_CCR1_EN__CH1_ChannelDisable                 0x0U

#define DMA_CCR1_TCIE__CH1_TransferCmplt_IntrptEnbl     0x1U
#define DMA_CCR1_TCIE__CH1_TransferCmplt_IntrptDsbl     0x0U

#define DMA_CCR1_HTIE__CH1_HalfTransfer_IntrptEnbl      0x1U
#define DMA_CCR1_HTIE__CH1_HalfTransfer_IntrptDsbl      0x0U

#define DMA_CCR1_TEIE__CH1_TransferError_IntrptEnbl     0x1U
#define DMA_CCR1_TEIE__CH1_TransferError_IntrptDsbl     0x0U

#define DMA_CCR1_DIR__CH1_ReadPeripheral                0x0U
#define DMA_CCR1_DIR__CH1_ReadMemory                    0x1U

#define DMA_CCR1_CIRC__CH1_Circular_Enable              0x1U
#define DMA_CCR1_CIRC__CH1_Circular_Disable             0x0U

#define DMA_CCR1_PINC__CH1_PeripheralIncrement_Enable   0x1U
#define DMA_CCR1_PINC__CH1_PeripheralIncrement_Disable  0x0U

#define DMA_CCR1_MINC__CH1_MemoryIncrement_Enable       0x1U
#define DMA_CCR1_MINC__CH1_MemoryIncrement_Disable      0x0U

#define DMA_CCR1_PSIZE__CH1_PeripheralSize_8bit         0x0U
#define DMA_CCR1_PSIZE__CH1_PeripheralSize_16bit        0x1U
#define DMA_CCR1_PSIZE__CH1_PeripheralSize_32bit        0x2U

#define DMA_CCR1_MSIZE__CH1_MemorySize_8bit             0x0U
#define DMA_CCR1_MSIZE__CH1_MemorySize_16bit            0x1U
#define DMA_CCR1_MSIZE__CH1_MemorySize_32bit            0x2U

#define DMA_CCR1_PL__CH1_Priority_Low                   0x0U
#define DMA_CCR1_PL__CH1_Priority_Medium                0x1U
#define DMA_CCR1_PL__CH1_Priority_High                  0x2U
#define DMA_CCR1_PL__CH1_Priority_VeryHigh              0x3U

#define DMA_CCR1_MEM2MEM__CH1_Memory2Memory_Enable      0x1U
#define DMA_CCR1_MEM2MEM__CH1_Memory2Memory_Disable     0x0U

// DMA : CCR2
#define DMA_CCR2_EN__CH2_ChannelEnable                  0x1U
#define DMA_CCR2_EN__CH2_ChannelDisable                 0x0U

#define DMA_CCR2_TCIE__CH2_TransferCmplt_IntrptEnbl     0x1U
#define DMA_CCR2_TCIE__CH2_TransferCmplt_IntrptDsbl     0x0U

#define DMA_CCR2_HTIE__CH2_HalfTransfer_IntrptEnbl      0x1U
#define DMA_CCR2_HTIE__CH2_HalfTransfer_IntrptDsbl      0x0U

#define DMA_CCR2_TEIE__CH2_TransferError_IntrptEnbl     0x1U
#define DMA_CCR2_TEIE__CH2_TransferError_IntrptDsbl     0x0U

#define DMA_CCR2_DIR__CH2_ReadPeripheral                0x0U
#define DMA_CCR2_DIR__CH2_ReadMemory                    0x1U

#define DMA_CCR2_CIRC__CH2_Circular_Enable              0x1U
#define DMA_CCR2_CIRC__CH2_Circular_Disable             0x0U

#define DMA_CCR2_PINC__CH2_PeripheralIncrement_Enable   0x1U
#define DMA_CCR2_PINC__CH2_PeripheralIncrement_Disable  0x0U

#define DMA_CCR2_MINC__CH2_MemoryIncrement_Enable       0x1U
#define DMA_CCR2_MINC__CH2_MemoryIncrement_Disable      0x0U

#define DMA_CCR2_PSIZE__CH2_PeripheralSize_8bit         0x0U
#define DMA_CCR2_PSIZE__CH2_PeripheralSize_16bit        0x1U
#define DMA_CCR2_PSIZE__CH2_PeripheralSize_32bit        0x2U

#define DMA_CCR2_MSIZE__CH2_MemorySize_8bit             0x0U
#define DMA_CCR2_MSIZE__CH2_MemorySize_16bit            0x1U
#define DMA_CCR2_MSIZE__CH2_MemorySize_32bit            0x2U

#define DMA_CCR2_PL__CH2_Priority_Low                   0x0U
#define DMA_CCR2_PL__CH2_Priority_Medium                0x1U
#define DMA_CCR2_PL__CH2_Priority_High                  0x2U
#define DMA_CCR2_PL__CH2_Priority_VeryHigh              0x3U

#define DMA_CCR2_MEM2MEM__CH2_Memory2Memory_Enable      0x1U
#define DMA_CCR2_MEM2MEM__CH2_Memory2Memory_Disable     0x0U

// DMA : CCR3
#define DMA_CCR3_EN__CH3_ChannelEnable                  0x1U
#define DMA_CCR3_EN__CH3_ChannelDisable                 0x0U

#define DMA_CCR3_TCIE__CH3_TransferCmplt_IntrptEnbl     0x1U
#define DMA_CCR3_TCIE__CH3_TransferCmplt_IntrptDsbl     0x0U

#define DMA_CCR3_HTIE__CH3_HalfTransfer_IntrptEnbl      0x1U
#define DMA_CCR3_HTIE__CH3_HalfTransfer_IntrptDsbl      0x0U

#define DMA_CCR3_TEIE__CH3_TransferError_IntrptEnbl     0x1U
#define DMA_CCR3_TEIE__CH3_TransferError_IntrptDsbl     0x0U

#define DMA_CCR3_DIR__CH3_ReadPeripheral                0x0U
#define DMA_CCR3_DIR__CH3_ReadMemory                    0x1U

#define DMA_CCR3_CIRC__CH3_Circular_Enable              0x1U
#define DMA_CCR3_CIRC__CH3_Circular_Disable             0x0U

#define DMA_CCR3_PINC__CH3_PeripheralIncrement_Enable   0x1U
#define DMA_CCR3_PINC__CH3_PeripheralIncrement_Disable  0x0U

#define DMA_CCR3_MINC__CH3_MemoryIncrement_Enable       0x1U
#define DMA_CCR3_MINC__CH3_MemoryIncrement_Disable      0x0U

#define DMA_CCR3_PSIZE__CH3_PeripheralSize_8bit         0x0U
#define DMA_CCR3_PSIZE__CH3_PeripheralSize_16bit        0x1U
#define DMA_CCR3_PSIZE__CH3_PeripheralSize_32bit        0x2U

#define DMA_CCR3_MSIZE__CH3_MemorySize_8bit             0x0U
#define DMA_CCR3_MSIZE__CH3_MemorySize_16bit            0x1U
#define DMA_CCR3_MSIZE__CH3_MemorySize_32bit            0x2U

#define DMA_CCR3_PL__CH3_Priority_Low                   0x0U
#define DMA_CCR3_PL__CH3_Priority_Medium                0x1U
#define DMA_CCR3_PL__CH3_Priority_High                  0x2U
#define DMA_CCR3_PL__CH3_Priority_VeryHigh              0x3U

#define DMA_CCR3_MEM2MEM__CH3_Memory2Memory_Enable      0x1U
#define DMA_CCR3_MEM2MEM__CH3_Memory2Memory_Disable     0x0U

// DMA : CCR4
#define DMA_CCR4_EN__CH4_ChannelEnable                  0x1U
#define DMA_CCR4_EN__CH4_ChannelDisable                 0x0U

#define DMA_CCR4_TCIE__CH4_TransferCmplt_IntrptEnbl     0x1U
#define DMA_CCR4_TCIE__CH4_TransferCmplt_IntrptDsbl     0x0U

#define DMA_CCR4_HTIE__CH4_HalfTransfer_IntrptEnbl      0x1U
#define DMA_CCR4_HTIE__CH4_HalfTransfer_IntrptDsbl      0x0U

#define DMA_CCR4_TEIE__CH4_TransferError_IntrptEnbl     0x1U
#define DMA_CCR4_TEIE__CH4_TransferError_IntrptDsbl     0x0U

#define DMA_CCR4_DIR__CH4_ReadPeripheral                0x0U
#define DMA_CCR4_DIR__CH4_ReadMemory                    0x1U

#define DMA_CCR4_CIRC__CH4_Circular_Enable              0x1U
#define DMA_CCR4_CIRC__CH4_Circular_Disable             0x0U

#define DMA_CCR4_PINC__CH4_PeripheralIncrement_Enable   0x1U
#define DMA_CCR4_PINC__CH4_PeripheralIncrement_Disable  0x0U

#define DMA_CCR4_MINC__CH4_MemoryIncrement_Enable       0x1U
#define DMA_CCR4_MINC__CH4_MemoryIncrement_Disable      0x0U

#define DMA_CCR4_PSIZE__CH4_PeripheralSize_8bit         0x0U
#define DMA_CCR4_PSIZE__CH4_PeripheralSize_16bit        0x1U
#define DMA_CCR4_PSIZE__CH4_PeripheralSize_32bit        0x2U

#define DMA_CCR4_MSIZE__CH4_MemorySize_8bit             0x0U
#define DMA_CCR4_MSIZE__CH4_MemorySize_16bit            0x1U
#define DMA_CCR4_MSIZE__CH4_MemorySize_32bit            0x2U

#define DMA_CCR4_PL__CH4_Priority_Low                   0x0U
#define DMA_CCR4_PL__CH4_Priority_Medium                0x1U
#define DMA_CCR4_PL__CH4_Priority_High                  0x2U
#define DMA_CCR4_PL__CH4_Priority_VeryHigh              0x3U

#define DMA_CCR4_MEM2MEM__CH4_Memory2Memory_Enable      0x1U
#define DMA_CCR4_MEM2MEM__CH4_Memory2Memory_Disable     0x0U

// DMA : CCR5
#define DMA_CCR5_EN__CH5_ChannelEnable                  0x1U
#define DMA_CCR5_EN__CH5_ChannelDisable                 0x0U

#define DMA_CCR5_TCIE__CH5_TransferCmplt_IntrptEnbl     0x1U
#define DMA_CCR5_TCIE__CH5_TransferCmplt_IntrptDsbl     0x0U

#define DMA_CCR5_HTIE__CH5_HalfTransfer_IntrptEnbl      0x1U
#define DMA_CCR5_HTIE__CH5_HalfTransfer_IntrptDsbl      0x0U

#define DMA_CCR5_TEIE__CH5_TransferError_IntrptEnbl     0x1U
#define DMA_CCR5_TEIE__CH5_TransferError_IntrptDsbl     0x0U

#define DMA_CCR5_DIR__CH5_ReadPeripheral                0x0U
#define DMA_CCR5_DIR__CH5_ReadMemory                    0x1U

#define DMA_CCR5_CIRC__CH5_Circular_Enable              0x1U
#define DMA_CCR5_CIRC__CH5_Circular_Disable             0x0U

#define DMA_CCR5_PINC__CH5_PeripheralIncrement_Enable   0x1U
#define DMA_CCR5_PINC__CH5_PeripheralIncrement_Disable  0x0U

#define DMA_CCR5_MINC__CH5_MemoryIncrement_Enable       0x1U
#define DMA_CCR5_MINC__CH5_MemoryIncrement_Disable      0x0U

#define DMA_CCR5_PSIZE__CH5_PeripheralSize_8bit         0x0U
#define DMA_CCR5_PSIZE__CH5_PeripheralSize_16bit        0x1U
#define DMA_CCR5_PSIZE__CH5_PeripheralSize_32bit        0x2U

#define DMA_CCR5_MSIZE__CH5_MemorySize_8bit             0x0U
#define DMA_CCR5_MSIZE__CH5_MemorySize_16bit            0x1U
#define DMA_CCR5_MSIZE__CH5_MemorySize_32bit            0x2U

#define DMA_CCR5_PL__CH5_Priority_Low                   0x0U
#define DMA_CCR5_PL__CH5_Priority_Medium                0x1U
#define DMA_CCR5_PL__CH5_Priority_High                  0x2U
#define DMA_CCR5_PL__CH5_Priority_VeryHigh              0x3U

#define DMA_CCR5_MEM2MEM__CH5_Memory2Memory_Enable      0x1U
#define DMA_CCR5_MEM2MEM__CH5_Memory2Memory_Disable     0x0U

// DMA : CCR6
#define DMA_CCR6_EN__CH6_ChannelEnable                  0x1U
#define DMA_CCR6_EN__CH6_ChannelDisable                 0x0U

#define DMA_CCR6_TCIE__CH6_TransferCmplt_IntrptEnbl     0x1U
#define DMA_CCR6_TCIE__CH6_TransferCmplt_IntrptDsbl     0x0U

#define DMA_CCR6_HTIE__CH6_HalfTransfer_IntrptEnbl      0x1U
#define DMA_CCR6_HTIE__CH6_HalfTransfer_IntrptDsbl      0x0U

#define DMA_CCR6_TEIE__CH6_TransferError_IntrptEnbl     0x1U
#define DMA_CCR6_TEIE__CH6_TransferError_IntrptDsbl     0x0U

#define DMA_CCR6_DIR__CH6_ReadPeripheral                0x0U
#define DMA_CCR6_DIR__CH6_ReadMemory                    0x1U

#define DMA_CCR6_CIRC__CH6_Circular_Enable              0x1U
#define DMA_CCR6_CIRC__CH6_Circular_Disable             0x0U

#define DMA_CCR6_PINC__CH6_PeripheralIncrement_Enable   0x1U
#define DMA_CCR6_PINC__CH6_PeripheralIncrement_Disable  0x0U

#define DMA_CCR6_MINC__CH6_MemoryIncrement_Enable       0x1U
#define DMA_CCR6_MINC__CH6_MemoryIncrement_Disable      0x0U

#define DMA_CCR6_PSIZE__CH6_PeripheralSize_8bit         0x0U
#define DMA_CCR6_PSIZE__CH6_PeripheralSize_16bit        0x1U
#define DMA_CCR6_PSIZE__CH6_PeripheralSize_32bit        0x2U

#define DMA_CCR6_MSIZE__CH6_MemorySize_8bit             0x0U
#define DMA_CCR6_MSIZE__CH6_MemorySize_16bit            0x1U
#define DMA_CCR6_MSIZE__CH6_MemorySize_32bit            0x2U

#define DMA_CCR6_PL__CH6_Priority_Low                   0x0U
#define DMA_CCR6_PL__CH6_Priority_Medium                0x1U
#define DMA_CCR6_PL__CH6_Priority_High                  0x2U
#define DMA_CCR6_PL__CH6_Priority_VeryHigh              0x3U

#define DMA_CCR6_MEM2MEM__CH6_Memory2Memory_Enable      0x1U
#define DMA_CCR6_MEM2MEM__CH6_Memory2Memory_Disable     0x0U

// DMA : CCR7
#define DMA_CCR7_EN__CH7_ChannelEnable                  0x1U
#define DMA_CCR7_EN__CH7_ChannelDisable                 0x0U

#define DMA_CCR7_TCIE__CH7_TransferCmplt_IntrptEnbl     0x1U
#define DMA_CCR7_TCIE__CH7_TransferCmplt_IntrptDsbl     0x0U

#define DMA_CCR7_HTIE__CH7_HalfTransfer_IntrptEnbl      0x1U
#define DMA_CCR7_HTIE__CH7_HalfTransfer_IntrptDsbl      0x0U

#define DMA_CCR7_TEIE__CH7_TransferError_IntrptEnbl     0x1U
#define DMA_CCR7_TEIE__CH7_TransferError_IntrptDsbl     0x0U

#define DMA_CCR7_DIR__CH7_ReadPeripheral                0x0U
#define DMA_CCR7_DIR__CH7_ReadMemory                    0x1U

#define DMA_CCR7_CIRC__CH7_Circular_Enable              0x1U
#define DMA_CCR7_CIRC__CH7_Circular_Disable             0x0U

#define DMA_CCR7_PINC__CH7_PeripheralIncrement_Enable   0x1U
#define DMA_CCR7_PINC__CH7_PeripheralIncrement_Disable  0x0U

#define DMA_CCR7_MINC__CH7_MemoryIncrement_Enable       0x1U
#define DMA_CCR7_MINC__CH7_MemoryIncrement_Disable      0x0U

#define DMA_CCR7_PSIZE__CH7_PeripheralSize_8bit         0x0U
#define DMA_CCR7_PSIZE__CH7_PeripheralSize_16bit        0x1U
#define DMA_CCR7_PSIZE__CH7_PeripheralSize_32bit        0x2U

#define DMA_CCR7_MSIZE__CH7_MemorySize_8bit             0x0U
#define DMA_CCR7_MSIZE__CH7_MemorySize_16bit            0x1U
#define DMA_CCR7_MSIZE__CH7_MemorySize_32bit            0x2U

#define DMA_CCR7_PL__CH7_Priority_Low                   0x0U
#define DMA_CCR7_PL__CH7_Priority_Medium                0x1U
#define DMA_CCR7_PL__CH7_Priority_High                  0x2U
#define DMA_CCR7_PL__CH7_Priority_VeryHigh              0x3U

#define DMA_CCR7_MEM2MEM__CH7_Memory2Memory_Enable      0x1U
#define DMA_CCR7_MEM2MEM__CH7_Memory2Memory_Disable     0x0U


/******************************************************************************
 ******     TIM
******************************************************************************/

// TIM : CR1
#define TIM_CR1_CKD__tDTS_tCK_INT                       0x0U
#define TIM_CR1_CKD__tDTS_2xtCK_INT                     0x1U
#define TIM_CR1_CKD__tDTS_4xCK_INT                      0x2U

#define TIM_CR1_APRE__ARR_Buffered_Disable              0x0U
#define TIM_CR1_APRE__ARR_Buffered_Enable               0x1U

#define TIM_CR1_CMS__EdgeAlignedMode                    0x0U
#define TIM_CR1_CMS__CenterAlignedMode1                 0x1U
#define TIM_CR1_CMS__CenterAlignedMode2                 0x2U
#define TIM_CR1_CMS__CenterAlignedMode3                 0x3U

#define TIM_CR1_DIR__UpCounter                          0x0U
#define TIM_CR1_DIR__DownCounter                        0x1U

#define TIM_CR1_OPM__OnePulseMode_Disable               0x0U
#define TIM_CR1_OPM__OnePulseMode_Enable                0x1U

#define TIM_CR1_URS__AnyEvent_Generate_UEV              0x0U
#define TIM_CR1_URS__OnlyCounterReload_Generate_UEV     0x1U

#define TIM_CR1_UDIS__UEV_Enable                        0x0U
#define TIM_CR1_UDIS__UEV_Disable                       0x1U

#define TIM_CR1_CEN__CounterEnable                      0x1U
#define TIM_CR1_CEN__CounterDisable                     0x0U

// TIM : CR2
#define TIM_CR2_OIS4__OC4_0_AfterDeadTime               0x0U
#define TIM_CR2_OIS4__OC4_1_AfterDeadTime               0x1U

#define TIM_CR2_OIS3N__OC3N_0_AfterDeadTime             0x0U
#define TIM_CR2_OIS3N__OC3N_1_AfterDeadTime             0x1U

#define TIM_CR2_OIS3__OC3_0_AfterDeadTime               0x0U
#define TIM_CR2_OIS3__OC3_1_AfterDeadTime               0x1U

#define TIM_CR2_OIS2N__OC2N_0_AfterDeadTime             0x0U
#define TIM_CR2_OIS2N__OC2N_1_AfterDeadTime             0x1U

#define TIM_CR2_OIS2__OC2_0_AfterDeadTime               0x0U
#define TIM_CR2_OIS2__OC2_1_AfterDeadTime               0x1U

#define TIM_CR2_OIS1N__OC1N_0_AfterDeadTime             0x0U
#define TIM_CR2_OIS1N__OC1N_1_AfterDeadTime             0x1U

#define TIM_CR2_OIS1__OC1_0_AfterDeadTime               0x0U
#define TIM_CR2_OIS1__OC1_1_AfterDeadTime               0x1U

#define TIM_CR2_TI1S__TIMx_CH1_ConnectedTo_TI1          0x0U
#define TIM_CR2_TI1S__TIMx_CH1_CH2_CH3_ConnectedTo_TI1  0x1U

#define TIM_CR2_MMS__Trigger_Output_Reset               0x0U
#define TIM_CR2_MMS__Trigger_Output_Enable              0x1U
#define TIM_CR2_MMS__Trigger_Output_Update              0x2U
#define TIM_CR2_MMS__Trigger_Output_ComparePulse        0x3U
#define TIM_CR2_MMS__Trigger_Output_OC1REF              0x4U
#define TIM_CR2_MMS__Trigger_Output_OC2REF              0x5U
#define TIM_CR2_MMS__Trigger_Output_OC3REF              0x6U
#define TIM_CR2_MMS__Trigger_Output_OC4REF              0x7U

#define TIM_CR2_CCDS__CCx_DMA_Request_CCx_Event         0x0U
#define TIM_CR2_CCDS__CCx_DMA_Request_Update_Event      0x1U

#define TIM_CR2_CCUS__CC_ControlBitsUpdatedBy_COMG      0x0U
#define TIM_CR2_CCUS__CC_ControlBitsUpdatedBy_COMG_TRGI 0x1U

#define TIM_CR2_CCPC__CCxE_CCxNE_OCxM_NotPreloaded      0x0U
#define TIM_CR2_CCPC__CCxE_CCxNE_OCxM_Preloaded         0x1U

// TIM : SMCR
#define TIM_SMCR_ETP__ETR_notInverted                   0x0U
#define TIM_SMCR_ETP__ETR_Inverted                      0x1U

#define TIM_SMCR_ECE__ExternalClockMode2_Disable        0x0U
#define TIM_SMCR_ECE__ExternalClockMode2_Enable         0x1U

#define TIM_SMCR_ETPS__Prescaler_Off                    0x0U
#define TIM_SMCR_ETPS__ETRP_Div_2                       0x1U
#define TIM_SMCR_ETPS__ETRP_Div_4                       0x2U
#define TIM_SMCR_ETPS__ETRP_Div_8                       0x3U

#define TIM_SMCR_ETF__NoFilter                          0x0U
#define TIM_SMCR_ETF__INT_N2                            0x1U
#define TIM_SMCR_ETF__INT_N4                            0x2U
#define TIM_SMCR_ETF__INT_N8                            0x3U
#define TIM_SMCR_ETF__DTSdiv2_N6                        0x4U
#define TIM_SMCR_ETF__DTSdiv2_N8                        0x5U
#define TIM_SMCR_ETF__DTSdiv4_N6                        0x6U
#define TIM_SMCR_ETF__DTSdiv4_N8                        0x7U
#define TIM_SMCR_ETF__DTSdiv8_N6                        0x8U
#define TIM_SMCR_ETF__DTSdiv8_N8                        0x9U
#define TIM_SMCR_ETF__DTSdiv16_N5                       0xAU
#define TIM_SMCR_ETF__DTSdiv16_N6                       0xBU
#define TIM_SMCR_ETF__DTSdiv16_N8                       0xCU
#define TIM_SMCR_ETF__DTSdiv32_N5                       0xDU
#define TIM_SMCR_ETF__DTSdiv32_N6                       0xEU
#define TIM_SMCR_ETF__DTSdiv32_N8                       0xFU

#define TIM_SMCR_MSM__NoAction                          0x0U
#define TIM_SMCR_MSM__Synchronize_MasterAndSlave        0x1U

#define TIM_SMCR_TS__InternalTrigger_ITR0               0x0U
#define TIM_SMCR_TS__InternalTrigger_ITR1               0x1U
#define TIM_SMCR_TS__InternalTrigger_ITR2               0x2U
#define TIM_SMCR_TS__InternalTrigger_ITR3               0x3U
#define TIM_SMCR_TS__InternalTrigger_TI1FED             0x4U
#define TIM_SMCR_TS__InternalTrigger_TI1FP1             0x5U
#define TIM_SMCR_TS__InternalTrigger_TI1FP2             0x6U
#define TIM_SMCR_TS__InternalTrigger_ETRF               0x7U

#define TIM_SMCR_SMS__Disable                           0x0U
#define TIM_SMCR_SMS__EncoderMode1                      0x1U
#define TIM_SMCR_SMS__EncoderMode2                      0x2U
#define TIM_SMCR_SMS__EncoderMode3                      0x3U
#define TIM_SMCR_SMS__ResetMode                         0x4U
#define TIM_SMCR_SMS__GatedMode                         0x5U
#define TIM_SMCR_SMS__TriggerMode                       0x6U
#define TIM_SMCR_SMS__ExternalClockMode1                0x7U

// TIM : DIER
#define TIM_DIER_TDE__Triger_DMA_Enable                 0x1U
#define TIM_DIER_TDE__Triger_DMA_Disable                0x0U

#define TIM_DIER_COMDE__COM_DMA_Enable                  0x1U
#define TIM_DIER_COMDE__COM_DMA_Disable                 0x0U

#define TIM_DIER_CC1DE__CC1_DMA_Enable                  0x1U
#define TIM_DIER_CC1DE__CC1_DMA_Disable                 0x0U

#define TIM_DIER_CC2DE__CC2_DMA_Enable                  0x1U
#define TIM_DIER_CC2DE__CC2_DMA_Disable                 0x0U

#define TIM_DIER_CC3DE__CC3_DMA_Enable                  0x1U
#define TIM_DIER_CC3DE__CC3_DMA_Disable                 0x0U

#define TIM_DIER_CC4DE__CC4_DMA_Enable                  0x1U
#define TIM_DIER_CC4DE__CC4_DMA_Disable                 0x0U

#define TIM_DIER_UDE__Update_DMA_Enable                 0x1U
#define TIM_DIER_UDE__Update_DMA_Disable                0x0U

#define TIM_DIER_BIE__Break_IntrptEnbl                  0x1U
#define TIM_DIER_BIE__Break_IntrptDsbl                  0x0U

#define TIM_DIER_TIE__Triger_IntrptEnbl                 0x1U
#define TIM_DIER_TIE__Triger_IntrptDsbl                 0x0U

#define TIM_DIER_COMIE__COM_IntrptEnbl                  0x1U
#define TIM_DIER_COMIE__COM_IntrptDsbl                  0x0U

#define TIM_DIER_CC1IE__CC1_IntrptEnbl                  0x1U
#define TIM_DIER_CC1IE__CC1_IntrptDsbl                  0x0U

#define TIM_DIER_CC2IE__CC2_IntrptEnbl                  0x1U
#define TIM_DIER_CC2IE__CC2_IntrptDsbl                  0x0U

#define TIM_DIER_CC3IE__CC3_IntrptEnbl                  0x1U
#define TIM_DIER_CC3IE__CC3_IntrptDsbl                  0x0U

#define TIM_DIER_CC4IE__CC4_IntrptEnbl                  0x1U
#define TIM_DIER_CC4IE__CC4_IntrptDsbl                  0x0U

#define TIM_DIER_UIE__Update_IntrptEnbl                 0x1U
#define TIM_DIER_UIE__Update_IntrptDsbl                 0x0U

// TIM : SR
#define TIM_SR_BIF__Break_IntrptEnbl                    0x1U
#define TIM_SR_BIF__Break_IntrptDsbl                    0x0U

#define TIM_SR_TIF__Triger_IntrptEnbl                   0x1U
#define TIM_SR_TIF__Triger_IntrptDsbl                   0x0U

#define TIM_SR_COMIF__COM_IntrptEnbl                    0x1U
#define TIM_SR_COMIF__COM_IntrptDsbl                    0x0U

#define TIM_SR_CC1IF__CC1_IntrptEnbl                    0x1U
#define TIM_SR_CC1IF__CC1_IntrptDsbl                    0x0U

#define TIM_SR_CC2IF__CC2_IntrptEnbl                    0x1U
#define TIM_SR_CC2IF__CC2_IntrptDsbl                    0x0U

#define TIM_SR_CC3IF__CC3_IntrptEnbl                    0x1U
#define TIM_SR_CC3IF__CC3_IntrptDsbl                    0x0U

#define TIM_SR_CC4IF__CC4_IntrptEnbl                    0x1U
#define TIM_SR_CC4IF__CC4_IntrptDsbl                    0x0U

#define TIM_SR_UIF__Update_IntrptEnbl                   0x1U
#define TIM_SR_UIF__Update_IntrptDsbl                   0x0U

// TIM : EGR
#define TIM_EGR_UG__Generate_Update_Event               0x1U
#define TIM_EGR_CC1G__Generate_CC1_Event                0x1U
#define TIM_EGR_CC2G__Generate_CC2_Event                0x1U
#define TIM_EGR_CC3G__Generate_CC3_Event                0x1U
#define TIM_EGR_CC4G__Generate_CC4_Event                0x1U
#define TIM_EGR_COMG__Generate_COM_Event                0x1U
#define TIM_EGR_TG__Generate_Trigger_Event              0x1U
#define TIM_EGR_BG__Generate_Break_Event                0x1U

// TIM : CCMR1
#define TIM_CCMR1_CC1S__Output                          0x0U
#define TIM_CCMR1_CC1S__InputDirect                     0x1U
#define TIM_CCMR1_CC1S__InputIndirect                   0x2U
#define TIM_CCMR1_CC1S__TRC                             0x3U

#define TIM_CCMR1_OC1FE__OutputCompare_FastMode_Disable 0x0U
#define TIM_CCMR1_OC1FE__OutputCompare_FastMode_Enable  0x1U

#define TIM_CCMR1_OC1PE__Preload_Enable                 0x1U
#define TIM_CCMR1_OC1PE__Preload_Disable                0x0U

#define TIM_CCMR1_OC1M__Frozen                          0x0U
#define TIM_CCMR1_OC1M__ActiveOnMatch                   0x1U
#define TIM_CCMR1_OC1M__InactiveOnMatch                 0x2U
#define TIM_CCMR1_OC1M__Toggle                          0x3U
#define TIM_CCMR1_OC1M__ForceInactive                   0x4U
#define TIM_CCMR1_OC1M__ForceActive                     0x5U
#define TIM_CCMR1_OC1M__PWM1                            0x6U
#define TIM_CCMR1_OC1M__PWM2                            0x7U

#define TIM_CCMR1_OC1CE__OCxRef_NotAffectedByETRF       0x1U
#define TIM_CCMR1_OC1CE__OCxRef_CleardByETRF            0x0U

#define TIM_CCMR1_IC1PSC__NoPrescaler                   0x1U
#define TIM_CCMR1_IC1PSC__Div_2                         0x1U
#define TIM_CCMR1_IC1PSC__Div_4                         0x2U
#define TIM_CCMR1_IC1PSC__Div_8                         0x3U

#define TIM_CCMR1_IC1F__NoFilter                        0x0U
#define TIM_CCMR1_IC1F__Fsampling_Fint_N2               0x1U
#define TIM_CCMR1_IC1F__Fsampling_Fint_N4               0x2U
#define TIM_CCMR1_IC1F__Fsampling_Fint_N8               0x3U
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv2_N6           0x4U
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv2_N8           0x5U
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv4_N6           0x6U
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv4_N80          0x7U
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv8_N60          0x8U
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv8_N80          0x9U
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv16_N5          0xAU
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv16_N6          0xBU
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv16_N8          0xCU
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv32_N5          0xDU
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv32_N6          0xEU
#define TIM_CCMR1_IC1F__Fsampling_FdtsDiv32_N8          0xFU

#define TIM_CCMR1_CC2S__Output                          0x0U
#define TIM_CCMR1_CC2S__InputDirect                     0x1U
#define TIM_CCMR1_CC2S__InputIndirect                   0x2U
#define TIM_CCMR1_CC2S__TRC                             0x3U

#define TIM_CCMR1_OC2FE__OutputCompare_FastMode_Disable 0x0U
#define TIM_CCMR1_OC2FE__OutputCompare_FastMode_Enable  0x1U

#define TIM_CCMR1_OC2PE__Preload_Enable                 0x1U
#define TIM_CCMR1_OC2PE__Preload_Disable                0x0U

#define TIM_CCMR1_OC2M__Frozen                          0x0U
#define TIM_CCMR1_OC2M__ActiveOnMatch                   0x1U
#define TIM_CCMR1_OC2M__InactiveOnMatch                 0x2U
#define TIM_CCMR1_OC2M__Toggle                          0x3U
#define TIM_CCMR1_OC2M__ForceInactive                   0x4U
#define TIM_CCMR1_OC2M__ForceActive                     0x5U
#define TIM_CCMR1_OC2M__PWM1                            0x6U
#define TIM_CCMR1_OC2M__PWM2                            0x7U

#define TIM_CCMR1_OC2CE__OCxRef_NotAffectedByETRF       0x1U
#define TIM_CCMR1_OC2CE__OCxRef_CleardByETRF            0x0U

#define TIM_CCMR1_IC2PSC__NoPrescaler                   0x1U
#define TIM_CCMR1_IC2PSC__Div_2                         0x1U
#define TIM_CCMR1_IC2PSC__Div_4                         0x2U
#define TIM_CCMR1_IC2PSC__Div_8                         0x3U

#define TIM_CCMR1_IC1F__NoFilter                        0x0U
#define TIM_CCMR1_IC2F__Fsampling_Fint_N2               0x1U
#define TIM_CCMR1_IC2F__Fsampling_Fint_N4               0x2U
#define TIM_CCMR1_IC2F__Fsampling_Fint_N8               0x3U
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv2_N6           0x4U
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv2_N8           0x5U
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv4_N6           0x6U
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv4_N8           0x7U
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv8_N6           0x8U
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv8_N8           0x9U
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv16_N5          0xAU
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv16_N6          0xBU
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv16_N8          0xCU
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv32_N5          0xDU
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv32_N6          0xEU
#define TIM_CCMR1_IC2F__Fsampling_FdtsDiv32_N8          0xFU

// TIM : CCMR2
#define TIM_CCMR2_CC3S__Output                          0x0U
#define TIM_CCMR2_CC3S__InputDirect                     0x1U
#define TIM_CCMR2_CC3S__InputIndirect                   0x2U
#define TIM_CCMR2_CC3S__TRC                             0x3U

#define TIM_CCMR2_OC3FE__OutputCompare_FastMode_Disable 0x0U
#define TIM_CCMR2_OC3FE__OutputCompare_FastMode_Enable  0x1U

#define TIM_CCMR2_OC3PE__Preload_Enable                 0x1U
#define TIM_CCMR2_OC3PE__Preload_Disable                0x0U

#define TIM_CCMR2_OC3M__Frozen                          0x0U
#define TIM_CCMR2_OC3M__ActiveOnMatch                   0x1U
#define TIM_CCMR2_OC3M__InactiveOnMatch                 0x2U
#define TIM_CCMR2_OC3M__Toggle                          0x3U
#define TIM_CCMR2_OC3M__ForceInactive                   0x4U
#define TIM_CCMR2_OC3M__ForceActive                     0x5U
#define TIM_CCMR2_OC3M__PWM1                            0x6U
#define TIM_CCMR2_OC3M__PWM2                            0x7U

#define TIM_CCMR2_CC4S__Output                          0x0U
#define TIM_CCMR2_CC4S__InputDirect                     0x1U
#define TIM_CCMR2_CC4S__InputIndirect                   0x2U
#define TIM_CCMR2_CC4S__TRC                             0x3U

#define TIM_CCMR2_OC4FE__OutputCompare_FastMode_Disable 0x0U
#define TIM_CCMR2_OC4FE__OutputCompare_FastMode_Enable  0x1U

#define TIM_CCMR2_OC4PE__Preload_Enable                 0x1U
#define TIM_CCMR2_OC4PE__Preload_Disable                0x0U

#define TIM_CCMR2_OC4M__Frozen                          0x0U
#define TIM_CCMR2_OC4M__ActiveOnMatch                   0x1U
#define TIM_CCMR2_OC4M__InactiveOnMatch                 0x2U
#define TIM_CCMR2_OC4M__Toggle                          0x3U
#define TIM_CCMR2_OC4M__ForceInactive                   0x4U
#define TIM_CCMR2_OC4M__ForceActive                     0x5U
#define TIM_CCMR2_OC4M__PWM1                            0x6U
#define TIM_CCMR2_OC4M__PWM2                            0x7U

#define TIM_CCMR2_OC3CE__OC3Ref_NotAffectedByETRF       0x1U
#define TIM_CCMR2_OC3CE__OC3Ref_CleardByETRF            0x0U

#define TIM_CCMR2_OC4CE__OC4Ref_NotAffectedByETRF       0x1U
#define TIM_CCMR2_OC4CE__OC4Ref_CleardByETRF            0x0U

#define TIM_CCMR2_IC3PSC__NoPrescaler                   0x1U
#define TIM_CCMR2_IC3PSC__Div_2                         0x1U
#define TIM_CCMR2_IC3PSC__Div_4                         0x2U
#define TIM_CCMR2_IC3PSC__Div_8                         0x3U

#define TIM_CCMR2_IC4PSC__NoPrescaler                   0x1U
#define TIM_CCMR2_IC4PSC__Div_2                         0x1U
#define TIM_CCMR2_IC4PSC__Div_4                         0x2U
#define TIM_CCMR2_IC4PSC__Div_8                         0x3U

#define TIM_CCMR1_IC1F__NoFilter                        0x0U
#define TIM_CCMR2_IC3F__Fsampling_Fint_N2               0x1U
#define TIM_CCMR2_IC3F__Fsampling_Fint_N4               0x2U
#define TIM_CCMR2_IC3F__Fsampling_Fint_N8               0x3U
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv2_N6           0x4U
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv2_N8           0x5U
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv4_N6           0x6U
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv4_N8           0x7U
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv8_N6           0x8U
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv8_N8           0x9U
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv16_N5          0xAU
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv16_N6          0xBU
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv16_N8          0xCU
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv32_N5          0xDU
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv32_N6          0xEU
#define TIM_CCMR2_IC3F__Fsampling_FdtsDiv32_N8          0xFU

#define TIM_CCMR1_IC1F__NoFilter                        0x0U
#define TIM_CCMR2_IC4F__Fsampling_Fint_N2               0x1U
#define TIM_CCMR2_IC4F__Fsampling_Fint_N4               0x2U
#define TIM_CCMR2_IC4F__Fsampling_Fint_N8               0x3U
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv2_N6           0x4U
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv2_N8           0x5U
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv4_N6           0x6U
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv4_N8           0x7U
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv8_N6           0x8U
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv8_N8           0x9U
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv16_N5          0xAU
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv16_N6          0xBU
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv16_N8          0xCU
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv32_N5          0xDU
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv32_N6          0xEU
#define TIM_CCMR2_IC4F__Fsampling_FdtsDiv32_N8          0xFU

// TIM : CCER
#define TIM_CCER_CCxE__ChannelEnable                    0x1U
#define TIM_CCER_CCxE__ChannelDisable                   0x0U

#define TIM_CCER_CCxP__OCActiveHigh                     0x0U
#define TIM_CCER_CCxP__OCActiveLow                      0x1U

#define TIM_CCER_CCxP__ICRisingEdge                     0x0U
#define TIM_CCER_CCxP__ICFallingEdge                    0x1U

#define TIM_CCER_CCxNE__ChannelEnable                   0x1U
#define TIM_CCER_CCxNE__ChannelDisable                  0x0U

#define TIM_CCER_CCxNP__OCActiveHigh                    0x0U
#define TIM_CCER_CCxNP__OCActiveLow                    0x1U

// TIM : BDTR
#define TIM_BDTR_MOE__Main_output_Disable               0x0U
#define TIM_BDTR_MOE__Main_output_Enable                0x1U

#define TIM_BDTR_AOE__MOE_SetOnlyBySoftware             0x0U
#define TIM_BDTR_AOE__MOE_SetBySoftwareOrUpdateEvent    0x1U

#define TIM_BDTR_BKP__BreakInput_ActiveLow              0x0U
#define TIM_BDTR_BKP__BreakInput_ActiveHigh             0x1U

#define TIM_BDTR_BKE__BreakInputs_Disable               0x0U
#define TIM_BDTR_BKE__BreakInputs_Enable                0x1U

#define TIM_BDTR_OSSR__WhenInactive_OC_OCN_Disable      0x0U
#define TIM_BDTR_OSSR__WhenInactive_OC_OCN_Enabled      0x1U

#define TIM_BDTR_OSSI__WhenInactive_OC_OCN_Disable      0x0U
#define TIM_BDTR_OSSI__WhenInactive_OC_OCN_ForcedIdle   0x1U

#define TIM_BDTR_LOCK__Lock_Off                         0x0U
#define TIM_BDTR_LOCK__L1_DTG_OISx_OISxN_BKE_BKP_AOE    0x1U
#define TIM_BDTR_LOCK__L2_L1plus_CCxP_CCxNP_OSSR_OSSI   0x2U
#define TIM_BDTR_LOCK__L3_L2plus_OCxM_OCxPE             0x3U

// TIM : DCR
#define TIM_DCR_DBL__1_Transfer                         0x0U
#define TIM_DCR_DBL__2_Transfer                         0x1U
#define TIM_DCR_DBL__3_Transfer                         0x2U
#define TIM_DCR_DBL__4_Transfer                         0x3U
#define TIM_DCR_DBL__5_Transfer                         0x4U
#define TIM_DCR_DBL__6_Transfer                         0x5U
#define TIM_DCR_DBL__7_Transfer                         0x6U
#define TIM_DCR_DBL__8_Transfer                         0x7U
#define TIM_DCR_DBL__9_Transfer                         0x8U
#define TIM_DCR_DBL__10_Transfer                        0x9U
#define TIM_DCR_DBL__11_Transfer                        0xAU
#define TIM_DCR_DBL__12_Transfer                        0xBU
#define TIM_DCR_DBL__13_Transfer                        0xCU
#define TIM_DCR_DBL__14_Transfer                        0xDU
#define TIM_DCR_DBL__15_Transfer                        0xEU
#define TIM_DCR_DBL__16_Transfer                        0xFU
#define TIM_DCR_DBL__17_Transfer                        0x10U
#define TIM_DCR_DBL__18_Transfer                        0x11U

#define TIM_DCR_DBA__TIMx_CR1                           0x0U
#define TIM_DCR_DBA__TIMx_CR2                           0x1U
#define TIM_DCR_DBA__TIMx_SMCR                          0x2U
#define TIM_DCR_DBA__TIMx_DIER                          0x3U
#define TIM_DCR_DBA__TIMx_SR                            0x4U
#define TIM_DCR_DBA__TIMx_EGR                           0x5U
#define TIM_DCR_DBA__TIMx_CCMR1                         0x6U
#define TIM_DCR_DBA__TIMx_CCMR2                         0x7U
#define TIM_DCR_DBA__TIMx_CCER                          0x8U
#define TIM_DCR_DBA__TIMx_CNT                           0x9U
#define TIM_DCR_DBA__TIMx_PSC                           0xAU
#define TIM_DCR_DBA__TIMx_ARR                           0xBU
#define TIM_DCR_DBA__TIMx_RCR                           0xCU
#define TIM_DCR_DBA__TIMx_CCR1                          0xDU
#define TIM_DCR_DBA__TIMx_CCR2                          0xEU
#define TIM_DCR_DBA__TIMx_CCR3                          0xFU
#define TIM_DCR_DBA__TIMx_CCR4                          0x10U
#define TIM_DCR_DBA__TIMx_BDTR                          0x11U
#define TIM_DCR_DBA__TIMx_DCR                           0x12U
#define TIM_DCR_DBA__TIMx_DMAR                          0x13U

/*****************************************************************************
 ******     RTC
******************************************************************************/

// RTC : CRH
#define RTC_CRH_SECIE__Second_IntrptDsbl                0x0U
#define RTC_CRH_SECIE__Second_IntrptEnbl                0x1U

#define RTC_CRH_ALRIE__Alarm_IntrptDsbl                 0x0U
#define RTC_CRH_ALRIE__Alarm_IntrptEnbl                 0x1U

#define RTC_CRH_OWIE__Overflow_IntrptDsbl               0x0U
#define RTC_CRH_OWIE__Overflow_IntrptEnbl               0x1U

// RTC : CRL
#define RTC_CRL_ALRF__No_Alarm                          0x0U
#define RTC_CRL_ALRF__Alarm_Detected                    0x1U

#define RTC_CRL_SECF__No_SecondFlag                     0x0U
#define RTC_CRL_SECF__SecondFlagConditionMet            0x1U

#define RTC_CRL_OWF__No_OverFlow                        0x0U
#define RTC_CRL_OWF__OverFlow_Occurred                  0x1U

#define RTC_CRL_RSF__Registers_NOT_Synchronized         0x0U
#define RTC_CRL_RSF__Registers_Synchronized             0x1U

#define RTC_CRL_CNF__ExitConfigurationMode              0x0U
#define RTC_CRL_CNF__EnterConfigurationMode             0x1U

#define RTC_CRL_RTOFF__WriteOperation_Ongoing           0x0U
#define RTC_CRL_RTOFF__WriteOperation_Terminated        0x1U

/*****************************************************************************
 ******     IWDG
******************************************************************************/

// IWDG : KR
#define IWDG_KR_KEY__UpdateCounter                      0xAAAAU
#define IWDG_KR_KEY__PR_RLR_EnableAccess                0x5555U
#define IWDG_KR_KEY__StartWatchDog                      0xCCCCU

// IWDG : PR
#define IWDG_PR_PR__Div_4                               0x0U
#define IWDG_PR_PR__Div_8                               0x1U
#define IWDG_PR_PR__Div_16                              0x2U
#define IWDG_PR_PR__Div_32                              0x3U
#define IWDG_PR_PR__Div_64                              0x4U
#define IWDG_PR_PR__Div_128                             0x5U
#define IWDG_PR_PR__Div_256                             0x6U
#define IWDG_PR_PR__Div_256_                            0x7U

// IWDG : SR
#define IWDG_SR_RVU__ReloadValueUpdate_Cmpltd           0x0U
#define IWDG_SR_RVU__ReloadValueUpdate_Ongoing          0x1U

#define IWDG_SR_RVU__PrescalerValueUpdate_Cmpltd        0x0U
#define IWDG_SR_RVU__PrescalerValueUpdate_Ongoing       0x1U

/*****************************************************************************
 ******     20 WWDG
******************************************************************************/

// WWDG : CR
#define WWDG_CR_WDGA__Watchdog_Disable                  0x0U
#define WWDG_CR_WDGA__Watchdog_Enable                   0x1U

// WWDG : CFR
#define WWDG_CFR_WDGTB__PCLK1_Div_1                     0x0U
#define WWDG_CFR_WDGTB__PCLK1_Div_2                     0x1U
#define WWDG_CFR_WDGTB__PCLK1_Div_4                     0x2U
#define WWDG_CFR_WDGTB__PCLK1_Div_8                     0x3U

#define WWDG_CFR_EWI__EearlyWakeUp_IntrptDsbl           0x0U
#define WWDG_CFR_EWI__EearlyWakeUp_IntrptEnbl           0x1U

// WWDG : SR
#define WWDG_SR_EWIF__EerlyWakeUpInterrupt_Occurred     0x1U
#define WWDG_SR_EWIF__EerlyWakeUpInterrupt_FlgClr       0x0U

/*****************************************************************************
 ******     USART
******************************************************************************/

// USART : SR
#define USART_SR_PE__NO_ParityError                     0x0U
#define USART_SR_PE__ParityError_Detected               0x1U

#define USART_SR_FE__NO_FramingError                    0x0U
#define USART_SR_FE__FramingError_Detected              0x1U

#define USART_SR_NE__NO_NoiseError                      0x0U
#define USART_SR_NE__NoiseError_Detected                0x1U

#define USART_SR_ORE__NO_OverrunError                   0x0U
#define USART_SR_ORE__OverrunError_Detected             0x1U

#define USART_SR_IDLE__IdleLine_NOT_Detected            0x0U
#define USART_SR_IDLE__IdleLine_Detected                0x1U

#define USART_SR_RXNE__RXNE_FlgClr                      0x0U
#define USART_SR_RXNE__Data_NOT_Received                0x0U
#define USART_SR_RXNE__Data_Received                    0x1U

#define USART_SR_TC__TC_FlgClr                          0x0U
#define USART_SR_TC__Transmission_NOT_Cmplt             0x0U
#define USART_SR_TC__Transmission_Cmplt                 0x1U

#define USART_SR_TXE__DataNotTransferredToShiftReg      0x0U
#define USART_SR_TXE__DataTransferredToShiftReg         0x1U

#define USART_SR_LBD__LBD_FlgClr                        0x0U
#define USART_SR_LBD__LINbreak_NOT_Detected             0x0U
#define USART_SR_LBD__LINbreak_Detected                 0x1U

#define USART_SR_CTS__CTS_FlgClr                        0x0U
#define USART_SR_CTS__NoChangeOn_CTS_StatusLine         0x0U
#define USART_SR_CTS__ChangeOn_CTS_StatusLine           0x1U

// USART : CR1
#define USART_CR1_SBK__NoBreakCharacter                 0x0U
#define USART_CR1_SBK__BreakCharacter_Transmitted       0x1U

#define USART_CR1_RWU__Receiver_ActiveMode              0x0U
#define USART_CR1_RWU__Receiver_MuteMode                0x1U

#define USART_CR1_RE__Receiver_Disable                  0x0U
#define USART_CR1_RE__Receiver_Enable                   0x1U

#define USART_CR1_TE__Transmitter_Disable               0x0U
#define USART_CR1_TE__Transmitter_Enable                0x1U

#define USART_CR1_IDLEIE__IDLE_IntrptDsbl               0x0U
#define USART_CR1_IDLEIE__IDLE_IntrptEnbl               0x1U

#define USART_CR1_RXNEIE__RXNE_IntrptDsbl               0x0U
#define USART_CR1_RXNEIE__RXNE_IntrptEnbl               0x1U

#define USART_CR1_TCIE__TC_IntrptDsbl                   0x0U
#define USART_CR1_TCIE__TC_IntrptEnbl                   0x1U

#define USART_CR1_TXEIE__TXE_IntrptDsbl                 0x0U
#define USART_CR1_TXEIE__TXE_IntrptEnbl                 0x1U

#define USART_CR1_PEIE__PE_IntrptDsbl                   0x0U
#define USART_CR1_PEIE__PE_IntrptEnbl                   0x1U

#define USART_CR1_PS__Even_Parity                       0x0U
#define USART_CR1_PS__Odd_Parity                        0x1U

#define USART_CR1_PCE__ParityControl_Disable            0x0U
#define USART_CR1_PCE__ParityControl_Enable             0x1U

#define USART_CR1_WAKE__WakeUpMethod_IdleLine           0x0U
#define USART_CR1_WAKE__WakeUpMethod_AddressMark        0x1U

#define USART_CR1_M__1_StartBit_8_DataBits              0x0U
#define USART_CR1_M__1_StartBit_9_DataBits              0x1U

#define USART_CR1_UE__USART_Disable                     0x0U
#define USART_CR1_UE__USART_Enable                      0x1U

// USART : CR2
#define USART_CR2_ADD__NodeAddress_0                    0x0U
#define USART_CR2_ADD__NodeAddress_1                    0x1U
#define USART_CR2_ADD__NodeAddress_2                    0x2U
#define USART_CR2_ADD__NodeAddress_3                    0x3U
#define USART_CR2_ADD__NodeAddress_4                    0x4U
#define USART_CR2_ADD__NodeAddress_5                    0x5U
#define USART_CR2_ADD__NodeAddress_6                    0x6U
#define USART_CR2_ADD__NodeAddress_7                    0x7U
#define USART_CR2_ADD__NodeAddress_8                    0x8U
#define USART_CR2_ADD__NodeAddress_9                    0x9U
#define USART_CR2_ADD__NodeAddress_10                   0xAU
#define USART_CR2_ADD__NodeAddress_11                   0xBU
#define USART_CR2_ADD__NodeAddress_12                   0xCU
#define USART_CR2_ADD__NodeAddress_13                   0xDU
#define USART_CR2_ADD__NodeAddress_14                   0xEU
#define USART_CR2_ADD__NodeAddress_15                   0xFU

#define USART_CR2_LBDL__10_BitBreakDetection            0x0U
#define USART_CR2_LBDL__11_BitBreakDetection            0x1U

#define USART_CR2_LBDIE__LBD_IntrptDsbl                 0x0U
#define USART_CR2_LBDIE__LBD_IntrptEnbl                 0x1U

#define USART_CR2_LBCL__LastDataBitClockPulseNotOnCKpin 0x0U
#define USART_CR2_LBCL__LastDataBitClockPulseIsOnCKpin  0x1U

#define USART_CR2_CPHA__1stClock_FirstDataCapture       0x0U
#define USART_CR2_CPHA__2ndClock_FirstDataCapture       0x1U

#define USART_CR2_CPOL__CKpin_Low                       0x0U
#define USART_CR2_CPOL__CKpin_High                      0x1U

#define USART_CR2_CLKEN__CKpin_Enable                   0x0U
#define USART_CR2_CLKEN__CKpin_Disable                  0x1U

#define USART_CR2_STOP__1_StopBit                       0x0U
#define USART_CR2_STOP__0p5_StopBit                     0x1U
#define USART_CR2_STOP__2_StopBit                       0x2U
#define USART_CR2_STOP__1p5_StopBit                     0x3U

#define USART_CR2_LINEN__LINmode_Disable                0x0U
#define USART_CR2_LINEN__LINmode_Enable                 0x1U

// USART : CR3
#define USART_CR3_EIE__Error_IntrptDsbl                 0x0U
#define USART_CR3_EIE__Error_IntrptEnbl                 0x1U

#define USART_CR3_IREN__IrDA_Disable                    0x0U
#define USART_CR3_IREN__IrDA_Enable                     0x1U

#define USART_CR3_IRLP__NormalMode                      0x0U
#define USART_CR3_IRLP__LowPowerMode                    0x1U

#define USART_CR3_HDSEL__HalfDuplexMode_Not_Selected    0x0U
#define USART_CR3_HDSEL__HalfDuplexMode_Selected        0x1U

#define USART_CR3_NACK__NACK_Transmission_Disable       0x0U
#define USART_CR3_NACK__NACK_Transmission_Enable        0x1U

#define USART_CR3_SCEN__SmartCardMode_Disable           0x0U
#define USART_CR3_SCEN__SmartCardMode_Enable            0x1U

#define USART_CR3_DMAR__Reception_DMA_Disable           0x0U
#define USART_CR3_DMAR__Reception_DMA_Enable            0x1U

#define USART_CR3_DMAT__Transmission_DMA_Disable        0x0U
#define USART_CR3_DMAT__Transmission_DMA_Enable         0x1U

#define USART_CR3_RTSE__RTS_Disable                     0x0U
#define USART_CR3_RTSE__RTS_Enable                      0x1U

#define USART_CR3_CTSE__CTS_Disable                     0x0U
#define USART_CR3_CTSE__CTS_Enable                      0x1U

#define USART_CR3_CTSIE__CTS_IntrptDsbl                 0x0U
#define USART_CR3_CTSIE__CTS_IntrptEnbl                 0x1U

/*****************************************************************************
 ******     I2C
******************************************************************************/

// I2C : CR1
#define I2C_CR1_PE__Peripheral_Disable                  0x0U
#define I2C_CR1_PE__Peripheral_Enable                   0x1U

#define I2C_CR1_SMBUS__I2CMode                          0x0U
#define I2C_CR1_SMBUS__SMBusMode                        0x1U

#define I2C_CR1_SMBTYPE__SMBusDevice                    0x0U
#define I2C_CR1_SMBTYPE__SMBusHost                      0x1U

#define I2C_CR1_ENARP__ARP_Disable                      0x0U
#define I2C_CR1_ENARP__ARP_Enable                       0x1U

#define I2C_CR1_ENPEC__PEC_Calculation_Disable          0x0U
#define I2C_CR1_ENPEC__PEC_Calculation_Enable           0x1U

#define I2C_CR1_ENGC__GeneralCall_Disabled              0x0U
#define I2C_CR1_ENGC__GeneralCall_Enabled               0x1U

#define I2C_CR1_NOSTRETCH__ClockStretching_Enable       0x0U
#define I2C_CR1_NOSTRETCH__ClockStretching_Disable      0x1U

#define I2C_CR1_START__StartGeneration_Disable          0x0U
#define I2C_CR1_START__StartGeneration_Enable           0x1U

#define I2C_CR1_STOP__StopGeneration_Disable            0x0U
#define I2C_CR1_STOP__StopGeneration_Enable             0x1U

#define I2C_CR1_ACK__Acknowledge_Disable                0x0U
#define I2C_CR1_ACK__Acknowledge_Enable                 0x1U

#define I2C_CR1_POS__CurrentByte                        0x0U
#define I2C_CR1_POS__NextByte                           0x1U

#define I2C_CR1_PEC__NoPECtransfer                      0x0U
#define I2C_CR1_PEC__PECtransfer                        0x1U

#define I2C_CR1_ALERT__ReleasesSMBApinHigh              0x0U
#define I2C_CR1_ALERT__DrivesSMBApinLow                 0x1U

#define I2C_CR1_SWRST__I2C_Not_Under_Reset              0x0U
#define I2C_CR1_SWRST__I2C_Under_Reset                  0x1U

// I2C : CR2
#define I2C_CR2_ITERREN__Error_IntrptDsbl               0x0U
#define I2C_CR2_ITERREN__Error_IntrptEnbl               0x1U

#define I2C_CR2_ITEVTEN__Event_IntrptDsbl               0x0U
#define I2C_CR2_ITEVTEN__Event_IntrptEnbl               0x1U

#define I2C_CR2_ITBUFEN__Buffer_IntrptDsbl              0x0U
#define I2C_CR2_ITBUFEN__Buffer_IntrptEnbl              0x1U

#define I2C_CR2_DMAEN__DMA_Disable                      0x0U
#define I2C_CR2_DMAEN__DMA_Enable                       0x1U

#define I2C_CR2_LAST__DMA_EOT_Not_LastTransfer          0x0U
#define I2C_CR2_LAST__DMA_EOT_is_LastTransfer           0x1U

// I2C : OAR1
#define I2C_OAR1_ADDMODE__7_Bit_SlaveAddress            0x0U
#define I2C_OAR1_ADDMODE__10_Bit_SlaveAddress           0x1U

// I2C : OAR2
#define I2C_OAR2_ENDUAL__OnlyOAR1                       0x0U
#define I2C_OAR2_ENDUAL__OAR1_OAR2                      0x1U

// I2C : SR1
#define I2C_SR1_SB__NoStartCondition                    0x0U
#define I2C_SR1_SB__StartConditionGenerated             0x1U

#define I2C_SR1_ADDR__AddressMismatched                 0x0U
#define I2C_SR1_ADDR__ReivedAddressMatched              0x1U

#define I2C_SR1_BTF__Transfer_NotDone                   0x0U
#define I2C_SR1_BTF__Transfer_Succeeded                 0x1U

#define I2C_SR1_ADD10__NoADD10Event                     0x0U
#define I2C_SR1_ADD10__MasterSentFirstAddressByte       0x1U

#define I2C_SR1_STOPF__NoStopConditionDetected          0x0U
#define I2C_SR1_STOPF__StopConditionDetected            0x1U

#define I2C_SR1_RxNE__DataRegister_Empty                0x0U
#define I2C_SR1_RxNE__DataRegister_NotEmpty             0x1U

#define I2C_SR1_TxE__DataRegister_Empty                 0x0U
#define I2C_SR1_TxE__DataRegister_NotEmpty              0x1U

#define I2C_SR1_BERR__BERR_FlgClr                       0x0U
#define I2C_SR1_BERR__No_MisplacedStartOrStopCondition  0x0U
#define I2C_SR1_BERR__MisplacedStartOrStopCondition     0x1U

#define I2C_SR1_ARLO__ARLO_FlgClr                       0x0U
#define I2C_SR1_ARLO__No_ArbitrationLostDetected        0x0U
#define I2C_SR1_ARLO__ArbitrationLostDetected           0x1U

#define I2C_SR1_AF__AF_FlgClr                           0x0U
#define I2C_SR1_AF__No_AcknowledgeFailure               0x0U
#define I2C_SR1_AF__AcknowledgeFailure                  0x1U

#define I2C_SR1_OVR__Clear_OVR_Flag                     0x0U
#define I2C_SR1_OVR__No_OverrunOrUnderrun               0x0U
#define I2C_SR1_OVR__OverrunOrUnderrun                  0x1U

#define I2C_SR1_PECERR__PECERR_FlgClr                   0x0U
#define I2C_SR1_PECERR__No_PECerror                     0x0U
#define I2C_SR1_PECERR__PECerror                        0x1U

#define I2C_SR1_TIMEOUT__TIMEOUT_FlgClr                 0x0U
#define I2C_SR1_TIMEOUT__No_TimeOutError                0x0U
#define I2C_SR1_TIMEOUT__TimeOutError                   0x1U

#define I2C_SR1_SMBALERT__SMBALERT_FlgClr               0x0U
#define I2C_SR1_SMBALERT__No_SMBALERT                   0x0U
#define I2C_SR1_SMBALERT__SMBALERT_Occurred             0x1U

// I2C : SR2
#define I2C_SR2_MSL__SlaveMode                          0x0U
#define I2C_SR2_MSL__MasterMode                         0x1U

#define I2C_SR2_BUSY__No_Communication                  0x0U
#define I2C_SR2_BUSY__Ongoing_Communication             0x1U

#define I2C_SR2_TRA__DataBytes_Received                 0x0U
#define I2C_SR2_TRA__DataBytes_Transmitted              0x1U

#define I2C_SR2_GENCALL__No_GeneralCall                 0x0U
#define I2C_SR2_GENCALL__GeneralCallAddressReceived     0x1U

#define I2C_SR2_SMBDEFAULT__No_SMBusDeviceAddress       0x0U
#define I2C_SR2_SMBDEFAULT__SMBusDeviceAddress_Received 0x1U

#define I2C_SR2_SMBHOST__No_SMBusHostAddress            0x0U
#define I2C_SR2_SMBHOST__SMBusHostAddress_Received      0x1U

#define I2C_SR2_DUALF__ReceivedAddressMatched_OAR1      0x0U
#define I2C_SR2_DUALF__ReceivedAddressMatched_OAR2      0x1U

// I2C : CCR
#define I2C_CCR_FS__SM_Mode_I2C                         0x0U
#define I2c_CCR_FS__FM_Mode_I2C                         0x1U

#define I2C_CCR_DUTY__FM_Mode_tLow_div_tHigh_2          0x0U
#define I2C_CCR_DUTY__FM_Mode_tLow_div_tHigh_16div9     0x1U

/*****************************************************************************
 ******     SPI
******************************************************************************/

// SPI : CR1
#define SPI_CR1_CPHA__DataCaptureEdge_FirstClock        0x0U
#define SPI_CR1_CPHA__DataCaptureEdge_SecondClock       0x1U

#define SPI_CR1_CPOL__CKto0_WhenIdle                    0x0U
#define SPI_CR1_CPOL__CKto1_WhenIdle                    0x1U

#define SPI_CR1_MSTR__SlaveMode                         0x0U
#define SPI_CR1_MSTR__MasterMode                        0x1U

#define SPI_CR1_BR__BaudRate_PCLK_Div_2                 0x0U
#define SPI_CR1_BR__BaudRate_PCLK_Div_4                 0x1U
#define SPI_CR1_BR__BaudRate_PCLK_Div_8                 0x2U
#define SPI_CR1_BR__BaudRate_PCLK_Div_16                0x3U
#define SPI_CR1_BR__BaudRate_PCLK_Div_32                0x4U
#define SPI_CR1_BR__BaudRate_PCLK_Div_64                0x5U
#define SPI_CR1_BR__BaudRate_PCLK_Div_128               0x6U
#define SPI_CR1_BR__BaudRate_PCLK_Div_256               0x7U

#define SPI_CR1_SPE__Peripheral_Disable                 0x0U
#define SPI_CR1_SPE__Peripheral_Enable                  0x1U

#define SPI_CR1_LSBFIRST__MSB_TransmittedFirst          0x0U
#define SPI_CR1_LSBFIRST__LSB_TransmittedFirst          0x1U

#define SPI_CR1_SSI__NSS_Low                            0x0U
#define SPI_CR1_SSI__NSS_High                           0x1U

#define SPI_CR1_SSM__SoftwareSlaveManagement_Disable    0x0U
#define SPI_CR1_SSM__SoftwareSlaveManagement_Enable     0x1U

#define SPI_CR1_RXONLY__FullDuplex                      0x0U
#define SPI_CR1_RXONLY__Receive_Only                    0x1U

#define SPI_CR1_DFF__8_Bit_DataFrameFormat              0x0U
#define SPI_CR1_DFF__16_Bit_DataFrameFormat             0x1U

#define SPI_CR1_CRCNEXT__DataPhase                      0x0U
#define SPI_CR1_CRCNEXT__NextTransferIsCRC              0x1U

#define SPI_CR1_CRCEN__CRC_Calculation_Disable          0x0U
#define SPI_CR1_CRCEN__CRC_Calculation_Enable           0x1U

#define SPI_CR1_BIDIOE__OutputDisable_ReceiveOnly       0x0U
#define SPI_CR1_BIDIOE__OutputEnable_TransmitOnly       0x1U

#define SPI_CR1_BIDIMODE__2Line_UnidirectionalDataMode  0x0U
#define SPI_CR1_BIDIMODE__1Line_bidirectionalDataMode   0x1U

// SPI : CR2
#define SPI_CR2_RXDMAEN__RX_DMA_Disable                 0x0U
#define SPI_CR2_RXDMAEN__RX_DMA_Enable                  0x1U

#define SPI_CR2_TXDMAEN__TX_DMA_Disable                 0x0U
#define SPI_CR2_TXDMAEN__TX_DMA_Enable                  0x1U

#define SPI_CR2_SSOE__SS_Output_Disable                 0x0U
#define SPI_CR2_SSOE__SS_Output_Enable                  0x1U

#define SPI_CR2_ERRIE__Error_IntrptEnbl                 0x1U
#define SPI_CR2_ERRIE__Error_IntrptDsbl                 0x0U

#define SPI_CR2_RXNEIE__RXBUFF_not_empty_IntrptEnbl     0x1U
#define SPI_CR2_RXNEIE__RXBUFF_not_empty_IntrptDsbl     0x0U

#define SPI_CR2_TXEIE__TXBUFF_empty_IntrptEnbl          0x1U
#define SPI_CR2_TXEIE__TXBUFF_empty_IntrptDsbl          0x0U

// SPI : SR
#define SPI_SR_RXNE__RX_Buffer_Empty                    0x0U
#define SPI_SR_RXNE__RX_Buffer_NotEmpty                 0x1U

#define SPI_SR_TXE__TX_Buffer_NotEmpty                  0x0U
#define SPI_SR_TXE__TX_Buffer_Empty                     0x1U

#define SPI_SR_CHSIDE__Left_Channel                     0x0U
#define SPI_SR_CHSIDE__Right_Channel                    0x0U

#define SPI_SR_UDR__No_UnderrunOccurred	                0x0U
#define SPI_SR_UDR__UnderrunOccurred	                0x0U

#define SPI_SR_CRCERR__CRC_Match                        0x0U
#define SPI_SR_CRCERR__CRC_No_Match                     0x1U

#define SPI_SR_MODF__No_ModeFaultOccurred               0x0U
#define SPI_SR_MODF__ModeFaultOccurred                  0x1U

#define SPI_SR_OVR__No_OverrunOccurred                  0x0U
#define SPI_SR_OVR__OverrunOccurred                     0x1U

#define SPI_SR_BSY__SPI_I2S_NotBusy                     0x0U
#define SPI_SR_BSY__SPI_I2S_Busy                        0x1U

// SPI_I2S : SPI_I2SCFGR
#define SPI_SPI_I2SCFGR_I2SMOD__SPI_Mode                0x0U
#define SPI_SPI_I2SCFGR_I2SMOD__I2S_Mode                0x1U

#define SPI_SPI_I2SCFGR_I2SE__I2S_Disable               0x0U
#define SPI_SPI_I2SCFGR_I2SE__I2S_Enable                0x1U

#define SPI_SPI_I2SCFGR_I2SCFG__I2S_Slave_Transmit      0x0U
#define SPI_SPI_I2SCFGR_I2SCFG__I2S_Slave_Receive       0x1U
#define SPI_SPI_I2SCFGR_I2SCFG__I2S_Master_Transmit     0x2U
#define SPI_SPI_I2SCFGR_I2SCFG__I2S_Master_Receive      0x3U

#define SPI_SPI_I2SCFGR_PCMSYNC__ShortFrameSync         0x0U
#define SPI_SPI_I2SCFGR_PCMSYNC__LongFrameSync          0x1U

#define SPI_SPI_I2SCFGR_I2SSTD__I2S_Standard_Philips    0x0U
#define SPI_SPI_I2SCFGR_I2SSTD__I2S_Standard_MSB        0x1U
#define SPI_SPI_I2SCFGR_I2SSTD__I2S_Standard_LSB        0x2U
#define SPI_SPI_I2SCFGR_I2SSTD__I2S_Standard_PCM        0x3U

#define SPI_SPI_I2SCFGR_CKPOL_I2S_ClockSteadyState_Low  0x0U
#define SPI_SPI_I2SCFGR_CKPOL_I2S_ClockSteadyState_High 0x1U

#define SPI_SPI_I2SCFGR_DATLEN__DataLength_16_Bit       0x0U
#define SPI_SPI_I2SCFGR_DATLEN__DataLength_24_Bit       0x1U
#define SPI_SPI_I2SCFGR_DATLEN__DataLength_32_Bit       0x2U

#define SPI_SPI_I2SCFGR_CHLEN__ChannelLength_16_Bit     0x0U
#define SPI_SPI_I2SCFGR_CHLEN__ChannelLength_32_Bit     0x1U

// SPI : SPI_I2SPR
#define SPI_SPI_I2SPR_MCKOE__MasterClockOutput_Disable  0x0U
#define SPI_SPI_I2SPR_MCKOE__MasterClockOutput_Enable   0x1U

#define SPI_SPI_I2SPR_ODD__DividerValue_I2SDIVx2        0x0U
#define SPI_SPI_I2SPR_ODD__DividerValue_I2SDIVx2_Plus1  0x1U

/*****************************************************************************
 ******     USB
******************************************************************************/

// USB : CNTR
#define USB_CNTR_FRES__Clear_USB_Reset                  0x0U
#define USB_CNTR_FRES__Force_USB_Reset                  0x1U

#define USB_CNTR_PDWN__Exit_PowerDown                   0x0U
#define USB_CNTR_PDWN__Enter_PowerDown                  0x1U

#define USB_CNTR_LPMODE__No_LowPowerMode                0x0U
#define USB_CNTR_LPMODE__Enter_LowPowerMode             0x1U

#define USB_CNTR_FSUSP__Enter_SuspendMode               0x1U

#define USB_CNTR_RESUME__ResumeSignal_Deactive          0x0U
#define USB_CNTR_RESUME__ResumeSignal_Active            0x1U

#define USB_CNTR_ESOFM__ExpectedStartOfFrame_IntrptDsbl 0x0U
#define USB_CNTR_ESOFM__ExpectedStartOfFrame_IntrptEnbl 0x1U

#define USB_CNTR_SOFM__StartOfFrame_IntrptDsbl          0x0U
#define USB_CNTR_SOFM__StartOfFrame_IntrptEnbl          0x1U

#define USB_CNTR_RESETM__Reset_IntrptDsbl               0x0U
#define USB_CNTR_RESETM__Reset_IntrptEnbl               0x1U

#define USB_CNTR_SUSPM__SuspendMode_IntrptDsbl          0x0U
#define USB_CNTR_SUSPM__SuspendMode_IntrptEnbl          0x1U

#define USB_CNTR_WKUPM__WakeUp_IntrptDsbl               0x0U
#define USB_CNTR_WKUPM__WakeUp_IntrptEnbl               0x1U

#define USB_CNTR_ERRM__Error_IntrptDsbl                 0x0U
#define USB_CNTR_ERRM__Error_IntrptEnbl                 0x1U

#define USB_CNTR_PMAOVRM__PMAOVR_IntrptDsbl             0x0U
#define USB_CNTR_PMAOVRM__PMAOVR_IntrptEnbl             0x1U

#define USB_CNTR_CTRM__CorrectTransfer_IntrptDsbl       0x0U
#define USB_CNTR_CTRM__CorrectTransfer_IntrEnbl         0x1U

// USB : ISTR
#define USB_ISTR_EP_ID__EndPoint_0                      0x0U
#define USB_ISTR_EP_ID__EndPoint_1                      0x1U
#define USB_ISTR_EP_ID__EndPoint_2                      0x2U
#define USB_ISTR_EP_ID__EndPoint_3                      0x3U
#define USB_ISTR_EP_ID__EndPoint_4                      0x4U
#define USB_ISTR_EP_ID__EndPoint_5                      0x5U
#define USB_ISTR_EP_ID__EndPoint_6                      0x6U
#define USB_ISTR_EP_ID__EndPoint_7                      0x7U
#define USB_ISTR_EP_ID__EndPoint_8                      0x8U
#define USB_ISTR_EP_ID__EndPoint_9                      0x9U
#define USB_ISTR_EP_ID__EndPoint_10                     0xAU
#define USB_ISTR_EP_ID__EndPoint_11                     0xBU
#define USB_ISTR_EP_ID__EndPoint_12                     0xCU
#define USB_ISTR_EP_ID__EndPoint_13                     0xDU
#define USB_ISTR_EP_ID__EndPoint_14                     0xEU
#define USB_ISTR_EP_ID__EndPoint_15                     0xFU

#define USB_ISTR_DIR__Transmission                      0x0U
#define USB_ISTR_DIR__Reception                         0x1U

#define USB_ISTR_ESOF__ESOF_IntrptFlgRst                0x0U
#define USB_ISTR_ESOF__ESOF_IntrptFlgSt                 0x1U

#define USB_ISTR_SOF__SOF_IntrptFlgRst                  0x0U
#define USB_ISTR_SOF__SOF_IntrptFlgSt                   0x1U

#define USB_ISTR_RESET__Reset_IntrptFlgRst              0x0U
#define USB_ISTR_RESET__Reset_IntrptFlgSt               0x1U

#define USB_ISTR_SUSP__SuspendMode_IntrptFlgRst         0x0U
#define USB_ISTR_SUSP__SuspendMode_IntrptFlgSt          0x1U

#define USB_ISTR_WKUP__WakeUp_IntrptFlgRst              0x0U
#define USB_ISTR_WKUP__WakeUp_IntrptFlgSt               0x1U

#define USB_ISTR_ERR__Error_IntrptFlgRst                0x0U
#define USB_ISTR_ERR__Error_IntrptFlgSt                 0x1U

#define USB_ISTR_PMAOVR__PMAOVR_IntrptFlgRst            0x0U
#define USB_ISTR_PMAOVR__PMAOVR_IntrptFlgSt             0x1U

#define USB_ISTR_CTR__NoTransactionCmpltd               0x0U
#define USB_ISTR_CTR__EndPointCmpltdTransaction         0x1U

// Bit masks for clearing USB interrupt flags
#define USB_ISTR_ESOF__ESOF_IntrptFlgClr                0xFEFF
#define USB_ISTR_SOF__SOF_IntrptFlgClr                  0xFDFF
#define USB_ISTR_RESET__Reset_IntrptFlgClr              0xFBFF
#define USB_ISTR_SUSP__Suspend_IntrptFlgClr             0xF7FF
#define USB_ISTR_WKUP__WakeUp_IntrptFlgClr              0xEFFF
#define USB_ISTR_ERR__Error_IntrptFlgClr                0xDFFF

// USB : FNR
#define USB_FNR_LCK__unLocked                           0x0U
#define USB_FNR_LCK__Locked                             0x1U

#define USB_FNR_RXDM__RXDM_Reset                        0x0U
#define USB_FNR_RXDM__RXDM_Set                          0x1U

#define USB_FNR_RXDP__RXDP_Reset                        0x0U
#define USB_FNR_RXDP__RXDP_Set                          0x1U

#define USB_FNR_LSOF__LostSOF_0                         0x0U
#define USB_FNR_LSOF__LostSOF_1                         0x1U
#define USB_FNR_LSOF__LostSOF_2                         0x2U
#define USB_FNR_LSOF__LostSOF_3                         0x3U

// USB : DADDR
#define USB_DADDR_EF__USB_Function_Enable               0x1U
#define USB_DADDR_EF__USB_Function_Disable              0x0U

// USB : EPxR
#define USB_EPxR_EA__EndPointAddress_0                  0x0U
#define USB_EPxR_EA__EndPointAddress_1                  0x1U
#define USB_EPxR_EA__EndPointAddress_2                  0x2U
#define USB_EPxR_EA__EndPointAddress_3                  0x3U
#define USB_EPxR_EA__EndPointAddress_4                  0x4U
#define USB_EPxR_EA__EndPointAddress_5                  0x5U
#define USB_EPxR_EA__EndPointAddress_6                  0x6U
#define USB_EPxR_EA__EndPointAddress_7                  0x7U
#define USB_EPxR_EA__EndPointAddress_8                  0x8U
#define USB_EPxR_EA__EndPointAddress_9                  0x9U
#define USB_EPxR_EA__EndPointAddress_10                 0xAU
#define USB_EPxR_EA__EndPointAddress_11                 0xBU
#define USB_EPxR_EA__EndPointAddress_12                 0xCU
#define USB_EPxR_EA__EndPointAddress_13                 0xDU
#define USB_EPxR_EA__EndPointAddress_14                 0xEU
#define USB_EPxR_EA__EndPointAddress_15                 0xFU

#define USB_EPxR_STAT_TX__Disabled                      0x0U
#define USB_EPxR_STAT_TX__Stall                         0x1U
#define USB_EPxR_STAT_TX__NAK                           0x2U
#define USB_EPxR_STAT_TX__Valid                         0x3U

#define USB_EPxR_DTOG_TX__DataToggle_DATA0              0x0U
#define USB_EPxR_DTOG_TX__DataToggle_DATA1              0x1U

#define USB_EPxR_CTR_TX__CTR_TX_FlgClr                  0x0U
#define USB_EPxR_CTR_TX__TransmissionCorrectTransfer    0x1U

#define USB_EPxR_EP_KIND__Rst                           0x0U
#define USB_EPxR_EP_KIND__DoubleBuffer_Enable           0x1U
#define USB_EPxR_EP_KIND__ExpectedStatusOutTransaction  0x1U

#define USB_EPxR_EPTYPE__Bulk                           0x0U
#define USB_EPxR_EPTYPE__Control                        0x1U
#define USB_EPxR_EPTYPE__ISO                            0x2U
#define USB_EPxR_EPTYPE__Interrupt                      0x3U

#define USB_EPxR_SETUP__No_SetupTransaction             0x0U
#define USB_EPxR_SETUP__SetupTransactionCmpltd          0x1U

#define USB_EPxR_STAT_RX__Disabled                      0x0U
#define USB_EPxR_STAT_RX__Stall                         0x1U
#define USB_EPxR_STAT_RX__NAK                           0x2U
#define USB_EPxR_STAT_RX__Valid                         0x3U

#define USB_EPxR_DTOG_RX__DataToggle_DATA0              0x0U
#define USB_EPxR_DTOG_RX__DataToggle_DATA1              0x1U

#define USB_EPxR_CTR_RX__CTR_RX_Clear                   0x0U
#define USB_EPxR_CTR_RX__ReceptionCorrectTransfer       0x1U

// USB : COUNTn_RX
#define USB_COUNTn_RX_BLSIZE__2_Bytes_MemoryBlock       0x0U
#define USB_COUNTn_RX_BLSIZE__32_Bytes_MemoryBlock      0x1U

#define USB_COUNTn_RX_NUM_BLOCK__0_Block                0x0U
#define USB_COUNTn_RX_NUM_BLOCK__1_Block                0x1U
#define USB_COUNTn_RX_NUM_BLOCK__2_Blocks               0x2U
#define USB_COUNTn_RX_NUM_BLOCK__3_Blocks               0x3U
#define USB_COUNTn_RX_NUM_BLOCK__4_Blocks               0x4U
#define USB_COUNTn_RX_NUM_BLOCK__5_Blocks               0x5U
#define USB_COUNTn_RX_NUM_BLOCK__6_Blocks               0x6U
#define USB_COUNTn_RX_NUM_BLOCK__7_Blocks               0x7U
#define USB_COUNTn_RX_NUM_BLOCK__8_Blocks               0x8U
#define USB_COUNTn_RX_NUM_BLOCK__9_Blocks               0x9U
#define USB_COUNTn_RX_NUM_BLOCK__10_Blocks              0xAU
#define USB_COUNTn_RX_NUM_BLOCK__11_Blocks              0xBU
#define USB_COUNTn_RX_NUM_BLOCK__12_Blocks              0xCU
#define USB_COUNTn_RX_NUM_BLOCK__13_Blocks              0xDU
#define USB_COUNTn_RX_NUM_BLOCK__14_Blocks              0xEU
#define USB_COUNTn_RX_NUM_BLOCK__15_Blocks              0xFU
#define USB_COUNTn_RX_NUM_BLOCK__16_Blocks              0x10U
#define USB_COUNTn_RX_NUM_BLOCK__17_Blocks              0x12U
#define USB_COUNTn_RX_NUM_BLOCK__18_Blocks              0x13U
#define USB_COUNTn_RX_NUM_BLOCK__19_Blocks              0x14U
#define USB_COUNTn_RX_NUM_BLOCK__20_Blocks              0x15U
#define USB_COUNTn_RX_NUM_BLOCK__21_Blocks              0x16U
#define USB_COUNTn_RX_NUM_BLOCK__22_Blocks              0x17U
#define USB_COUNTn_RX_NUM_BLOCK__23_Blocks              0x18U
#define USB_COUNTn_RX_NUM_BLOCK__24_Blocks              0x19U
#define USB_COUNTn_RX_NUM_BLOCK__25_Blocks              0x1AU
#define USB_COUNTn_RX_NUM_BLOCK__26_Blocks              0x1BU
#define USB_COUNTn_RX_NUM_BLOCK__27_Blocks              0x1CU
#define USB_COUNTn_RX_NUM_BLOCK__28_Blocks              0x1DU
#define USB_COUNTn_RX_NUM_BLOCK__29_Blocks              0x1EU
#define USB_COUNTn_RX_NUM_BLOCK__30_Blocks              0x1FU
#define USB_COUNTn_RX_NUM_BLOCK__31_Blocks              0x20U


// USB : COUNTn_TX
#define USB_COUNTn_TX_BLSIZE_0__2_Bytes_MemoryBlock     0x0U
#define USB_COUNTn_TX_BLSIZE_0__32_Bytes_MemoryBlock    0x1U

#define USB_COUNTn_TX_BLSIZE_1__2_Bytes_MemoryBlock     0x0U
#define USB_COUNTn_TX_BLSIZE_1__32_Bytes_MemoryBlock    0x1U

#define USB_COUNTn_TX_NUM_BLOCK_0__0_Block              0x0U
#define USB_COUNTn_TX_NUM_BLOCK_0__1_Block              0x1U
#define USB_COUNTn_TX_NUM_BLOCK_0__2_Blocks             0x2U
#define USB_COUNTn_TX_NUM_BLOCK_0__3_Blocks             0x3U
#define USB_COUNTn_TX_NUM_BLOCK_0__4_Blocks             0x4U
#define USB_COUNTn_TX_NUM_BLOCK_0__5_Blocks             0x5U
#define USB_COUNTn_TX_NUM_BLOCK_0__6_Blocks             0x6U
#define USB_COUNTn_TX_NUM_BLOCK_0__7_Blocks             0x7U
#define USB_COUNTn_TX_NUM_BLOCK_0__8_Blocks             0x8U
#define USB_COUNTn_TX_NUM_BLOCK_0__9_Blocks             0x9U
#define USB_COUNTn_TX_NUM_BLOCK_0__10_Blocks            0xAU
#define USB_COUNTn_TX_NUM_BLOCK_0__11_Blocks            0xBU
#define USB_COUNTn_TX_NUM_BLOCK_0__12_Blocks            0xCU
#define USB_COUNTn_TX_NUM_BLOCK_0__13_Blocks            0xDU
#define USB_COUNTn_TX_NUM_BLOCK_0__14_Blocks            0xEU
#define USB_COUNTn_TX_NUM_BLOCK_0__15_Blocks            0xFU
#define USB_COUNTn_TX_NUM_BLOCK_0__16_Blocks            0x10U
#define USB_COUNTn_TX_NUM_BLOCK_0__17_Blocks            0x12U
#define USB_COUNTn_TX_NUM_BLOCK_0__18_Blocks            0x13U
#define USB_COUNTn_TX_NUM_BLOCK_0__19_Blocks            0x14U
#define USB_COUNTn_TX_NUM_BLOCK_0__20_Blocks            0x15U
#define USB_COUNTn_TX_NUM_BLOCK_0__21_Blocks            0x16U
#define USB_COUNTn_TX_NUM_BLOCK_0__22_Blocks            0x17U
#define USB_COUNTn_TX_NUM_BLOCK_0__23_Blocks            0x18U
#define USB_COUNTn_TX_NUM_BLOCK_0__24_Blocks            0x19U
#define USB_COUNTn_TX_NUM_BLOCK_0__25_Blocks            0x1AU
#define USB_COUNTn_TX_NUM_BLOCK_0__26_Blocks            0x1BU
#define USB_COUNTn_TX_NUM_BLOCK_0__27_Blocks            0x1CU
#define USB_COUNTn_TX_NUM_BLOCK_0__28_Blocks            0x1DU
#define USB_COUNTn_TX_NUM_BLOCK_0__29_Blocks            0x1EU
#define USB_COUNTn_TX_NUM_BLOCK_0__30_Blocks            0x1FU
#define USB_COUNTn_TX_NUM_BLOCK_0__31_Blocks            0x20U

#define USB_COUNTn_TX_NUM_BLOCK_1__0_Block              0x0U
#define USB_COUNTn_TX_NUM_BLOCK_1__1_Block              0x1U
#define USB_COUNTn_TX_NUM_BLOCK_1__2_Blocks             0x2U
#define USB_COUNTn_TX_NUM_BLOCK_1__3_Blocks             0x3U
#define USB_COUNTn_TX_NUM_BLOCK_1__4_Blocks             0x4U
#define USB_COUNTn_TX_NUM_BLOCK_1__5_Blocks             0x5U
#define USB_COUNTn_TX_NUM_BLOCK_1__6_Blocks             0x6U
#define USB_COUNTn_TX_NUM_BLOCK_1__7_Blocks             0x7U
#define USB_COUNTn_TX_NUM_BLOCK_1__8_Blocks             0x8U
#define USB_COUNTn_TX_NUM_BLOCK_1__9_Blocks             0x9U
#define USB_COUNTn_TX_NUM_BLOCK_1__10_Blocks            0xAU
#define USB_COUNTn_TX_NUM_BLOCK_1__11_Blocks            0xBU
#define USB_COUNTn_TX_NUM_BLOCK_1__12_Blocks            0xCU
#define USB_COUNTn_TX_NUM_BLOCK_1__13_Blocks            0xDU
#define USB_COUNTn_TX_NUM_BLOCK_1__14_Blocks            0xEU
#define USB_COUNTn_TX_NUM_BLOCK_1__15_Blocks            0xFU
#define USB_COUNTn_TX_NUM_BLOCK_1__16_Blocks            0x10U
#define USB_COUNTn_TX_NUM_BLOCK_1__17_Blocks            0x12U
#define USB_COUNTn_TX_NUM_BLOCK_1__18_Blocks            0x13U
#define USB_COUNTn_TX_NUM_BLOCK_1__19_Blocks            0x14U
#define USB_COUNTn_TX_NUM_BLOCK_1__20_Blocks            0x15U
#define USB_COUNTn_TX_NUM_BLOCK_1__21_Blocks            0x16U
#define USB_COUNTn_TX_NUM_BLOCK_1__22_Blocks            0x17U
#define USB_COUNTn_TX_NUM_BLOCK_1__23_Blocks            0x18U
#define USB_COUNTn_TX_NUM_BLOCK_1__24_Blocks            0x19U
#define USB_COUNTn_TX_NUM_BLOCK_1__25_Blocks            0x1AU
#define USB_COUNTn_TX_NUM_BLOCK_1__26_Blocks            0x1BU
#define USB_COUNTn_TX_NUM_BLOCK_1__27_Blocks            0x1CU
#define USB_COUNTn_TX_NUM_BLOCK_1__28_Blocks            0x1DU
#define USB_COUNTn_TX_NUM_BLOCK_1__29_Blocks            0x1EU
#define USB_COUNTn_TX_NUM_BLOCK_1__30_Blocks            0x1FU
#define USB_COUNTn_TX_NUM_BLOCK_1__31_Blocks            0x20U

/*****************************************************************************
 ******     CAN
******************************************************************************/

// CAN : MCR
#define CAN_MCR_INRQ__NormalMode                        0x0U
#define CAN_MCR_INRQ__InitializationMode                0x1U

#define CAN_MCR_SLEEP__Exit_SleepMode                   0x0U
#define CAN_MCR_SLEEP__Enter_SleepMode                  0x1U

#define CAN_MCR_TXFP__MessageIdentifier                 0x0U
#define CAN_MCR_TXFP__RequestOrder                      0x1U

#define CAN_MCR_RFLM__ReceiveFIFO_NoLoackedOnOverrun    0x0U
#define CAN_MCR_RFLM__ReceiveFIFO_LoackedOnOverrun      0x1U

#define CAN_MCR_NART__AutomaticTransaction              0x0U
#define CAN_MCR_NART__No_AutomaticTransaction           0x1U

#define CAN_MCR_AWUM__Software_SleepMode                0x0U
#define CAN_MCR_AWUM__Hardware_SleepMode                0x1U

#define CAN_MCR_ABOM__Software_BusOff                   0x0U
#define CAN_MCR_ABOM__Hardware_BusOff                   0x1U

#define CAN_MCR_TTCM__TimeTriggeredCommunication_Disable      0x0U
#define CAN_MCR_TTCM__TimeTriggeredCommunication_Enable       0x1U

#define CAN_MCR_RESET__NormalOperatoin                  0x0U
#define CAN_MCR_RESET__ForceReset                       0x1U

#define CAN_MCR_DBF__CAN_WorkingDuringDebug             0x0U
#define CAN_MCR_DBF__CAN_FrozenDuringDebug              0x1U

// CAN : MSR
#define CAN_MSR_INAK__In_NormalMode                     0x0U
#define CAN_MSR_INAK__In_InitializationMode             0x1U

#define CAN_MSR_SLAK__In_NormalMode                     0x0U
#define CAN_MSR_SLAK__In_SleepMode                      0x1U

#define CAN_MSR_ERRI__CAN_NoErrorOccurred               0x0U
#define CAN_MSR_ERRI__CAN_ErrorOccurred                 0x1U
#define CAN_MSR_ERRI__ERRI_FlgClr                       0x1U

#define CAN_MSR_WKUI__CAN_NotWakedUp                    0x0U
#define CAN_MSR_WKUI__CAN_WakedUp                       0x1U
#define CAN_MSR_WKUI__WKUI_FlgClr                       0x1U

#define CAN_MSR_SLAKI__CAN_EnteredSleepMode             0x1U
#define CAN_MSR_SLAKI__CAN_NotInSleepMode               0x1U
#define CAN_MSR_SLAKI__SLAKI_FlgClr                     0x1U

#define CAN_MSR_TXM__CAN_Not_Transmitter                0x0U
#define CAN_MSR_TXM__CAN_Is_Transmitter                 0x1U

#define CAN_MSR_RXM__CAN_Not_Receiver                   0x0U
#define CAN_MSR_RXM__CAN_Is_Receiver                    0x1U

#define CAN_MSR_SAMP__RX_LastSamplePoint_Was_Reset      0x0U
#define CAN_MSR_SAMP__RX_LastSamplePoint_Was_Set        0x1U

#define CAN_MSR_RX__RX_Pin_Is_Reset                     0x0U
#define CAN_MSR_RX__RX_Pin_Is_Set                       0x1U


// CAN : TSR
#define CAN_TSR_RQCP0__MailBox0_LastRequestPerformed    0x1U
#define CAN_TSR_RQCP0__MailBox0_RQCP0_FlgClr            0x1U

#define CAN_TSR_TXOK0__MailBox0_Transmission_Failed     0x0U
#define CAN_TSR_TXOK0__MailBox0_Transmission_Ok         0x1U

#define CAN_TSR_ALST0__MailBox0_TX_Failed_ArbitrationLost     0x1U
#define CAN_TSR_ALST0__MailBox0_ALST0_FlgClr                  0x1U

#define CAN_TSR_TERR0__MailBox0_TX_Failed_Error         0x1U
#define CAN_TSR_TERR0__MailBox0_TERR0_FlgClr            0x1U

#define CAN_TSR_ABRQ0__MailBox0_AbortTransmissionRequest      0x1U
#define CAN_TSR_ABRQ0__MailBox0_Empty                         0x0U

#define CAN_TSR_RQCP1__MailBox1_LastRequestPerformed    0x1U
#define CAN_TSR_RQCP1__MailBox1_RQCP0_FlgClr            0x1U

#define CAN_TSR_TXOK1__MailBox1_Transmission_Failed     0x0U
#define CAN_TSR_TXOK1__MailBox1_Transmission_Ok         0x1U

#define CAN_TSR_ALST1__MailBox1_TX_Failed_ArbitrationLost     0x1U
#define CAN_TSR_ALST1__MailBox1_ALST0_FlgClr                  0x1U

#define CAN_TSR_TERR1__MailBox1_TX_Failed_Error         0x1U
#define CAN_TSR_TERR1__MailBox1_TERR0_FlgClr            0x1U

#define CAN_TSR_ABRQ1__MailBox1_AbortTransmissionRequest      0x1U
#define CAN_TSR_ABRQ1__MailBox1_Empty                         0x0U

#define CAN_TSR_RQCP2__MailBox2_LastRequestPerformed    0x1U
#define CAN_TSR_RQCP2__MailBox2_RQCP0_FlgClr            0x1U

#define CAN_TSR_TXOK2__MailBox2_Transmission_Failed     0x0U
#define CAN_TSR_TXOK2__MailBox2_Transmission_Ok         0x1U

#define CAN_TSR_ALST2__MailBox2_TX_Failed_ArbitrationLost     0x1U
#define CAN_TSR_ALST2__MailBox2_ALST0_FlgClr                  0x1U

#define CAN_TSR_TERR2__MailBox2_TX_Failed_Error         0x1U
#define CAN_TSR_TERR2__MailBox2_TERR0_FlgClr            0x1U

#define CAN_TSR_ABRQ2__MailBox2_AbortTransmissionRequest      0x1U
#define CAN_TSR_ABRQ2__MailBox2_Empty                         0x0U

#define CAN_TSR_CODE__       0x0U
//#define CAN_TSR_CODE__       0x1U

#define CAN_TSR_TME0__MailBox0_NoTransmitRequestPending       0x1U
#define CAN_TSR_TME1__MailBox1_NoTransmitRequestPending       0x1U
#define CAN_TSR_TME2__MailBox2_NoTransmitRequestPending       0x1U

#define CAN_TSR_LOW0__       0x1U
#define CAN_TSR_LOW1__       0x1U
#define CAN_TSR_LOW2__       0x1U

// CAN : RF0R
#define CAN_RF0R_FMP0__FIFO0_0_MessagePending            0x0U
#define CAN_RF0R_FMP0__FIFO0_1_MessagePending            0x1U
#define CAN_RF0R_FMP0__FIFO0_2_MessagePending            0x2U
#define CAN_RF0R_FMP0__FIFO0_3_MessagePending            0x3U

#define CAN_RF0R_FULL0__FIFO0_3_Messages_Stored          0x1U
#define CAN_RF0R_FULL0__FULL0_Clear                      0x1U

#define CAN_RF0R_FOVR0__FIFO0_Receiver_OverRun           0x1U
#define CAN_RF0R_FOVR0__FOVR0_Clear                      0x1U

#define CAN_RF0R_RFOM0__Release_FIFO0_OutputMailBox      0x1U

// CAN : RF0R
#define CAN_RF1R_FMP1__FIFO1_0_MessagePending            0x0U
#define CAN_RF1R_FMP1__FIFO1_1_MessagePending            0x1U
#define CAN_RF1R_FMP1__FIFO1_2_MessagePending            0x2U
#define CAN_RF1R_FMP1__FIFO1_3_MessagePending            0x3U

#define CAN_RF1R_FULL1__FIFO1_3_Messages_Stored          0x1U
#define CAN_RF1R_FULL1__FULL1_Clear                      0x1U

#define CAN_RF1R_FOVR1__FIFO1_Receiver_OverRun           0x1U
#define CAN_RF1R_FOVR1__FOVR1_Clear                      0x1U

#define CAN_RF1R_RFOM1__Release_FIFO1_OutputMailBox      0x1U

// CAN : IER
#define CAN_IER_TMEIE__TransmitMailboxEmpty_IntrptDsbl         0x0U
#define CAN_IER_TMEIE__TransmitMailboxEmpty_IntrptEnbl          0x1U

#define CAN_IER_FMPIE0__FIFO0_MessagePending_IntrptDsbl        0x0U
#define CAN_IER_FMPIE0__FIFO0_MessagePending_IntrptEnbl         0x1U

#define CAN_IER_FFIE0__FIFO0_Full_IntrptDsbl                   0x0U
#define CAN_IER_FFIE0__FIFO0_Full_IntrptEnbl                    0x1U

#define CAN_IER_FOVIE0__FIFO0_Overrun_IntrptDsbl               0x0U
#define CAN_IER_FOVIE0__FIFO0_Overrun_IntrptEnbl                0x1U

#define CAN_IER_FMPIE1__FIFO1_MessagePending_IntrptDsbl        0x0U
#define CAN_IER_FMPIE1__FIFO1_MessagePending_IntrptEnbl         0x1U

#define CAN_IER_FFIE1__FIFO1_Full_IntrptDsbl                   0x0U
#define CAN_IER_FFIE1__FIFO1_Full_IntrptEnbl                    0x1U

#define CAN_IER_FOVIE1__FIFO1_Overrun_IntrptDsbl               0x0U
#define CAN_IER_FOVIE1__FIFO1_Overrun_IntrptEnbl                0x1U

#define CAN_IER_EWGIE__ErrorWarning_IntrptDsbl                 0x0U
#define CAN_IER_EWGIE__ErrorWarning_IntrptEnbl                  0x1U

#define CAN_IER_EPVIE__ErrorPhase_IntrptDsbl                   0x0U
#define CAN_IER_EPVIE__ErrorPhase_IntrptEnbl                    0x1U

#define CAN_IER_BOFIE__BusOff_IntrptDsbl                       0x0U
#define CAN_IER_BOFIE__BusOff_IntrptEnbl                        0x1U

#define CAN_IER_LECIE__LastErrorCode_IntrptDsbl                0x0U
#define CAN_IER_LECIE__LastErrorCode_IntrptEnbl                 0x1U

#define CAN_IER_ERRIE__Error_IntrptDsbl                        0x0U
#define CAN_IER_ERRIE__Error_IntrptEnbl                         0x1U

#define CAN_IER_WKUIE__Wakeup_IntrptDsbl                       0x0U
#define CAN_IER_WKUIE__Wakeup_IntrptEnbl                        0x1U

#define CAN_IER_SLKIE__Sleep_IntrptDsbl                        0x0U
#define CAN_IER_SLKIE__Sleep_IntrptEnbl                         0x1U

// CAN : ESR
#define CAN_ESR_EWGF__                          0x0U
#define CAN_ESR_EWGF__WarningLimitReached       0x1U

#define CAN_ESR_EPVF__                                0x0U
#define CAN_ESR_EPVF__ErrorPassiveLimitReached       0x1U

#define CAN_ESR_BOFF__       0x0U
#define CAN_ESR_BOFF__EnterBusOffState       0x1U

#define CAN_ESR_LEC__LastErrorCode_NoError               0x0U
#define CAN_ESR_LEC__LastErrorCode_Stuff                 0x1U
#define CAN_ESR_LEC__LastErrorCode_Form                  0x2U
#define CAN_ESR_LEC__LastErrorCode_Acknowledgmnt         0x3U
#define CAN_ESR_LEC__LastErrorCode_BitRecessive          0x4U
#define CAN_ESR_LEC__LastErrorCode_BitDominant           0x5U
#define CAN_ESR_LEC__LastErrorCode_CRC                   0x6U
#define CAN_ESR_LEC__LastErrorCode_SetBySoftware         0x7U


// CAN : BTR
#define CAN_BTR_LBKM__LoopBackMode_Disable               0x0U
#define CAN_BTR_LBKM__LoopBackMode_Enable                0x1U

#define CNA_BTR_SILM__NormalOperation                    0x0U
#define CNA_BTR_SILM__SilentMode                         0x0U

// CAN : TI0R
#define CAN_TI0R_TXRQ__MailBox0_RequestTransmit          0x0U
#define CAN_TI0R_TXRQ__MailBox0_Empty                    0x1U

#define CAN_TI0R_IDE__MailBox0_Standard_Identifier       0x0U
#define CAN_TI0R_IDE__MailBox0_Extended_Identifier       0x1U

#define CAN_TI0R_RTR__MailBox0_Data_Frame                0x0U
#define CAN_TI0R_RTR__MailBox0_Remote_Frame              0x1U

// CAN : TI1R
#define CAN_TI1R_TXRQ__MailBox1_RequestTransmit          0x0U
#define CAN_TI1R_TXRQ__MailBox1_Empty                    0x1U

#define CAN_TI1R_IDE__MailBox1_Standard_Identifier       0x0U
#define CAN_TI1R_IDE__MailBox1_Extended_Identifier       0x1U

#define CAN_TI1R_RTR__MailBox1_Data_Frame                0x0U
#define CAN_TI1R_RTR__MailBox1_Remote_Frame              0x1U

// CAN : TI2R
#define CAN_TI2R_TXRQ__MailBox2_RequestTransmit          0x0U
#define CAN_TI2R_TXRQ__MailBox2_Empty                    0x1U

#define CAN_TI2R_IDE__MailBox2_Standard_Identifier       0x0U
#define CAN_TI2R_IDE__MailBox2_Extended_Identifier       0x1U

#define CAN_TI2R_RTR__MailBox2_Data_Frame                0x0U
#define CAN_TI2R_RTR__MailBox2_Remote_Frame              0x1U


// CAN : TDT0R
#define CAN_TDT0R_DLC__MailBox0_0_Byte_DataFrame         0x0U
#define CAN_TDT0R_DLC__MailBox0_1_Byte_DataFrame         0x1U
#define CAN_TDT0R_DLC__MailBox0_2_Byte_DataFrame         0x2U
#define CAN_TDT0R_DLC__MailBox0_3_Byte_DataFrame         0x3U
#define CAN_TDT0R_DLC__MailBox0_4_Byte_DataFrame         0x4U
#define CAN_TDT0R_DLC__MailBox0_5_Byte_DataFrame         0x5U
#define CAN_TDT0R_DLC__MailBox0_6_Byte_DataFrame         0x6U
#define CAN_TDT0R_DLC__MailBox0_7_Byte_DataFrame         0x7U
#define CAN_TDT0R_DLC__MailBox0_8_Byte_DataFrame         0x8U

// CAN : TDT1R
#define CAN_TDT1R_DLC__MailBox1_0_Byte_DataFrame         0x0U
#define CAN_TDT1R_DLC__MailBox1_1_Byte_DataFrame         0x1U
#define CAN_TDT1R_DLC__MailBox1_2_Byte_DataFrame         0x2U
#define CAN_TDT1R_DLC__MailBox1_3_Byte_DataFrame         0x3U
#define CAN_TDT1R_DLC__MailBox1_4_Byte_DataFrame         0x4U
#define CAN_TDT1R_DLC__MailBox1_5_Byte_DataFrame         0x5U
#define CAN_TDT1R_DLC__MailBox1_6_Byte_DataFrame         0x6U
#define CAN_TDT1R_DLC__MailBox1_7_Byte_DataFrame         0x7U
#define CAN_TDT1R_DLC__MailBox1_8_Byte_DataFrame         0x8U

// CAN : TDT2R
#define CAN_TDT2R_DLC__MailBox2_0_Byte_DataFrame         0x0U
#define CAN_TDT2R_DLC__MailBox2_1_Byte_DataFrame         0x1U
#define CAN_TDT2R_DLC__MailBox2_2_Byte_DataFrame         0x2U
#define CAN_TDT2R_DLC__MailBox2_3_Byte_DataFrame         0x3U
#define CAN_TDT2R_DLC__MailBox2_4_Byte_DataFrame         0x4U
#define CAN_TDT2R_DLC__MailBox2_5_Byte_DataFrame         0x5U
#define CAN_TDT2R_DLC__MailBox2_6_Byte_DataFrame         0x6U
#define CAN_TDT2R_DLC__MailBox2_7_Byte_DataFrame         0x7U
#define CAN_TDT2R_DLC__MailBox2_8_Byte_DataFrame         0x8U



#define CAN_TDT0R_TGT__TimeStamp_NotSent                 0x0U
#define CAN_TDT0R_TGT__Send_TimeStamp                    0x1U




#define SET_BIT(REG, BIT)                    ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)                  ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)                   ((REG) & (BIT))
#define CLEAR_REG(REG)                       ((REG) = (0x0))
#define WRITE_REG(REG, VAL)                  ((REG) = (VAL))
#define READ_REG(REG)                        ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)                    (__CLZ(__RBIT(VAL)))
