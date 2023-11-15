#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

/* ------------ RCC registers bit address in the alias region ----------- */
#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE)

/* --- CR Register ---*/

/* Alias word address of HSION bit */
#define CR_OFFSET                 (RCC_OFFSET + 0x00)


#ifdef air32F10X_CL
 /* Alias word address of PLL2ON bit */
 #define PLL2ON_BitNumber          0x1A
 #define CR_PLL2ON_BB              (PERIPH_BB_BASE + (CR_OFFSET * 32) + (PLL2ON_BitNumber * 4))

 /* Alias word address of PLL3ON bit */
 #define PLL3ON_BitNumber          0x1C
 #define CR_PLL3ON_BB              (PERIPH_BB_BASE + (CR_OFFSET * 32) + (PLL3ON_BitNumber * 4))
#endif /* air32F10X_CL */ 



/* --- CFGR Register ---*/

/* Alias word address of USBPRE bit */
#define CFGR_OFFSET               (RCC_OFFSET + 0x04)

#ifndef air32F10X_CL
 #define USBPRE_BitNumber          0x16
 #define CFGR_USBPRE_BB            (PERIPH_BB_BASE + (CFGR_OFFSET * 32) + (USBPRE_BitNumber * 4))
#else
 #define OTGFSPRE_BitNumber        0x16
 #define CFGR_OTGFSPRE_BB          (PERIPH_BB_BASE + (CFGR_OFFSET * 32) + (OTGFSPRE_BitNumber * 4))
#endif /* air32F10X_CL */ 

/* --- BDCR Register ---*/




/* --- CSR Register ---*/



#ifdef air32F10X_CL
/* --- CFGR2 Register ---*/

 /* Alias word address of I2S2SRC bit */
 #define CFGR2_OFFSET              (RCC_OFFSET + 0x2C)
 #define I2S2SRC_BitNumber         0x11
 #define CFGR2_I2S2SRC_BB          (PERIPH_BB_BASE + (CFGR2_OFFSET * 32) + (I2S2SRC_BitNumber * 4))

 /* Alias word address of I2S3SRC bit */
 #define I2S3SRC_BitNumber         0x12
 #define CFGR2_I2S3SRC_BB          (PERIPH_BB_BASE + (CFGR2_OFFSET * 32) + (I2S3SRC_BitNumber * 4))
#endif /* air32F10X_CL */

/* ---------------------- RCC registers bit mask ------------------------ */

/* CR register bit mask */
#define CR_HSEBYP_Reset           ((volatile uint32_t)0xFFFBFFFF)
#define CR_HSEBYP_Set             ((volatile uint32_t)0x00040000)
#define CR_HSEON_Reset            ((volatile uint32_t)0xFFFEFFFF)
#define CR_HSEON_Set              ((volatile uint32_t)0x00010000)
#define CR_HSITRIM_Mask           ((volatile uint32_t)0xFFFFFF07)

/* CFGR register bit mask */
#if defined (air32F10X_LD_VL) || defined (air32F10X_MD_VL) || defined (air32F10X_HD_VL) || defined (air32F10X_CL) 
 #define CFGR_PLL_Mask            ((volatile uint32_t)0xFFC2FFFF)
#else
 #define CFGR_PLL_Mask            ((volatile uint32_t)0xFFC0FFFF)
#endif /* air32F10X_CL */ 

#define CFGR_PLLMull_Mask         ((volatile uint32_t)0x003C0000)
#define CFGR_PLLSRC_Mask          ((volatile uint32_t)0x00010000)
#define CFGR_PLLXTPRE_Mask        ((volatile uint32_t)0x00020000)
#define CFGR_SWS_Mask             ((volatile uint32_t)0x0000000C)
#define CFGR_SW_Mask              ((volatile uint32_t)0xFFFFFFFC)
#define CFGR_HPRE_Reset_Mask      ((volatile uint32_t)0xFFFFFF0F)
#define CFGR_HPRE_Set_Mask        ((volatile uint32_t)0x000000F0)
#define CFGR_PPRE1_Reset_Mask     ((volatile uint32_t)0xFFFFF8FF)
#define CFGR_PPRE1_Set_Mask       ((volatile uint32_t)0x00000700)
#define CFGR_PPRE2_Reset_Mask     ((volatile uint32_t)0xFFFFC7FF)
#define CFGR_PPRE2_Set_Mask       ((volatile uint32_t)0x00003800)
#define CFGR_ADCPRE_Reset_Mask    ((volatile uint32_t)0x9FFF3FFF)
#define CFGR_ADCPRE_Set_Mask      ((volatile uint32_t)0x0000C000)

/* CSR register bit mask */
#define CSR_RMVF_Set              ((volatile uint32_t)0x01000000)

#if defined (air32F10X_LD_VL) || defined (air32F10X_MD_VL) || defined (air32F10X_HD_VL) || defined (air32F10X_CL) 
/* CFGR2 register bit mask */
 #define CFGR2_PREDIV1SRC         ((volatile uint32_t)0x00010000)
 #define CFGR2_PREDIV1            ((volatile uint32_t)0x0000000F)
#endif
#ifdef air32F10X_CL
 #define CFGR2_PREDIV2            ((volatile uint32_t)0x000000F0)
 #define CFGR2_PLL2MUL            ((volatile uint32_t)0x00000F00)
 #define CFGR2_PLL3MUL            ((volatile uint32_t)0x0000F000)
#endif /* air32F10X_CL */ 

/* RCC Flag Mask */
#define FLAG_Mask                 ((volatile uint8_t)0x1F)



/**
  * @}
  */ 

/** @defgroup RCC_Private_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup RCC_Private_Variables
  * @{
  */ 

static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
static __I uint8_t ADCPrescTable[4] = {2, 4, 6, 8};

typedef enum 
{
	FLASH_Div_0 = 0,
	FLASH_Div_2 = 1,
	FLASH_Div_4 = 2,
	FLASH_Div_6 = 3,
	FLASH_Div_8 = 4,
	FLASH_Div_16 = 5,
}FlashClkDiv;

#define SysFreq_Set		(*((void (*)(uint32_t, FlashClkDiv , uint8_t, uint8_t))(*(uint32_t *)0x1FFFD00C)))

uint32_t AIR_RCC_PLLConfig(uint32_t RCC_PLLSource, uint32_t RCC_PLLMul, uint8_t Latency)
{	
	volatile uint32_t sramsize = 0;
	uint32_t pllmul = 0;
	FunctionalState pwr_gating_state = 0;
	/* Check the parameters */
	assert_param(IS_RCC_PLL_SOURCE(RCC_PLLSource));
	assert_param(IS_RCC_PLL_MUL(RCC_PLLMul));
	
	*(volatile uint32_t *)(0x400210F0) = BIT(0);//开启sys_cfg门控
	*(volatile uint32_t *)(0x40016C00) = 0xa7d93a86;//解一、二、三级锁
	*(volatile uint32_t *)(0x40016C00) = 0xab12dfcd;
	*(volatile uint32_t *)(0x40016C00) = 0xcded3526;
	sramsize = *(volatile uint32_t *)(0x40016C18);
	*(volatile uint32_t *)(0x40016C18) = 0x200183FF;//配置sram大小, 将BOOT使用对sram打开
	*(volatile uint32_t *)(0x4002228C) = 0xa5a5a5a5;//QSPI解锁
	
	SysFreq_Set(RCC_PLLMul,Latency ,0,1);
	RCC->CFGR = (RCC->CFGR & ~0x00030000) | RCC_PLLSource;
	
	//恢复配置前状态
	*(volatile uint32_t *)(0x40016C18) = sramsize;
	*(volatile uint32_t *)(0x400210F0) = 0;//开启sys_cfg门控
	*(volatile uint32_t *)(0x40016C00) = ~0xa7d93a86;//加一、二、三级锁
	*(volatile uint32_t *)(0x40016C00) = ~0xab12dfcd;
	*(volatile uint32_t *)(0x40016C00) = ~0xcded3526;
	*(volatile uint32_t *)(0x4002228C) = ~0xa5a5a5a5;//QSPI解锁
	
	
	return 1;
}
