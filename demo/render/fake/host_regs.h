// Host render: STM32F4 registers and SPL calls as inert stand-ins (the engine's timing hardware
// is not emulated: the host harness drives the scenes field by field)
#pragma once
#include <stdint.h>
typedef struct {
    volatile uint32_t CR, CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR,
        CCR1, CCR2, CCR3, CCR4, DCR, DMAR, OR, NDTR, PAR, M0AR, M1AR, FCR, LISR, HISR, LIFCR,
        HIFCR, AHB1ENR, APB1ENR, APB2ENR, DR, CTRL, CYCCNT, LAR, LOAD, VAL;
} HOST_REG_TypeDef;
typedef HOST_REG_TypeDef TIM_TypeDef, DMA_Stream_TypeDef, DMA_TypeDef, SPI_TypeDef, RCC_TypeDef, DWT_Type, SysTick_Type;
extern HOST_REG_TypeDef hostRegs[16];
#define TIM2 (&hostRegs[0])
#define TIM3 (&hostRegs[1])
#define TIM5 (&hostRegs[2])
#define DMA1 (&hostRegs[3])
#define DMA1_Stream4 (&hostRegs[4])
#define DMA1_Stream5 (&hostRegs[5])
#define DMA1_Stream7 (&hostRegs[6])
#define SPI2 (&hostRegs[7])
#define SPI5 (&hostRegs[8])
#define RCC (&hostRegs[9])
#define DWT (&hostRegs[10])
#define SysTick (&hostRegs[11])
#define __DSB() do { } while (0)
#define __get_PRIMASK() 0u
#define __set_PRIMASK(x) ((void)(x))
#define __disable_irq() do { } while (0)
#define __enable_irq() do { } while (0)
#define __CORTEX_M 4
#define __NVIC_PRIO_BITS 4
typedef enum { TIM5_IRQn = 50, SPI5_IRQn = 85, SysTick_IRQn = -1 } IRQn_Type;
#define NVIC_SetPriority(i, p) ((void)(i), (void)(p))
void hostPendIrq(int irq);
#define NVIC_SetPendingIRQ(i) hostPendIrq((int)(i))
#define NVIC_EnableIRQ(i) ((void)(i))
#define NVIC_DisableIRQ(i) ((void)(i))
#define NVIC_ClearPendingIRQ(i) ((void)(i))
#define NVIC_PRIO_MAX 1
#define NVIC_BUILD_PRIORITY(a, b) ((((a) << 4) | (b)) & 0xF0)
#define NVIC_PRIORITY_BASE(p) (((p) >> 4) & 0xF)
#define NVIC_PRIORITY_SUB(p) ((p) & 0xF)
#define TIM_DIER_CC1IE 2
#define TIM_SR_CC1IF 2
#define TIM_SR_CC2IF 4
#define TIM_DIER_CC2DE (1u << 10)
#define TIM_EGR_UG 1
#define TIM_CR1_CEN 1
#define TIM_CR2_MMS (7u << 4)
#define TIM_CCMR1_CC1S_0 (1u << 0)
#define TIM_CCMR1_CC2S_0 (1u << 8)
#define TIM_CCER_CC1P (1u << 1)
#define TIM_CCER_CC1E (1u << 0)
#define TIM_CCER_CC2P (1u << 5)
#define TIM_CCER_CC2E (1u << 4)
#define RCC_APB1ENR_TIM3EN 2
#define RCC_APB1ENR_TIM2EN 1
#define RCC_APB1ENR_TIM5EN 8
#define RCC_AHB1ENR_DMA1EN (1u << 21)
#define DMA_SxCR_EN 1
#define DMA_SxCR_TCIE (1u << 4)
#define DMA_SxCR_HTIE (1u << 3)
#define DMA_SxCR_PSIZE_0 (1u << 11)
#define DMA_SxCR_MSIZE_0 (1u << 13)
#define DMA_SxCR_PL_0 (1u << 16)
#define DMA_SxCR_PL_1 (1u << 17)
#define DMA_SxCR_MINC (1u << 10)
#define DMA_SxCR_DIR_0 (1u << 6)
#define DMA_SxCR_CIRC (1u << 8)
#define DMA_IT_TCIF 0x20
#define DMA_IT_HTIF 0x10
#define DMA_IT_TEIF 0x08
#define DMA_IT_DMEIF 0x04
#define DMA_IT_FEIF 0x01
#define DMA_GET_FLAG_STATUS(d, f) 0
#define DMA_CLEAR_FLAG(d, f) ((void)(d))
#define SPI5_IRQHandler hostSpi5IrqHandler
