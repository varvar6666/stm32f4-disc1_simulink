#include "stm32f407xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_dac.h"
#include "math.h"


#define F_CPU       100000000UL
#define AHB1        F_CPU
#define APB1        F_CPU/4
#define APB1_TIM    APB1*2
#define APB2        F_CPU/2
#define APB2_TIM    APB2*2
#define SysTicks_pr 1000
#define SysTicks    F_CPU/SysTicks_pr


#define PIN0    0
#define PIN1    1
#define PIN2    2
#define PIN3    3

#define PIN4    4
#define PIN5    5
#define PIN6    6
#define PIN7    7

#define PIN8    8
#define PIN9    9
#define PIN10   10
#define PIN11   11

#define PIN12   12
#define PIN13   13
#define PIN14   14
#define PIN15   15

#define USART_TX_BUFF_SIZE 10
#define USART_RX_BUFF_SIZE 4

uint16_t usart_tx_buff[USART_TX_BUFF_SIZE] = {0,11,12,13,14,15,16,17,18,20};
uint8_t  usart_rx_buff[USART_RX_BUFF_SIZE];
uint8_t tx_i = USART_TX_BUFF_SIZE;

void USART1_IRQHandler(void)
{
    static uint8_t cc = 0, rx_i = 0 ;
    
    if(USART1->SR & USART_SR_RXNE)
    {
        usart_rx_buff[rx_i] = USART1->DR;
        rx_i++;
        if(rx_i == USART_RX_BUFF_SIZE) 
            rx_i=0;
    }
    if(USART1->SR & USART_SR_TC)
    {
        if (tx_i != USART_TX_BUFF_SIZE)
        {
            if(cc == 0)
            {
                USART1->DR = usart_tx_buff[tx_i];
                cc++;
            }
            else
            {
                USART1->DR = usart_tx_buff[tx_i]>>8;
                cc=0;
                tx_i++;       
            }
        }
        else
        {
            USART1->CR1 &= ~USART_CR1_TCIE;
            cc=0;
        }
    }
}


void TIM6_DAC_IRQHandler(void)
{
    static uint16_t tim_mk = 0;
    if(TIM6->SR & TIM_SR_UIF)
    { 
        TIM6->SR=0;
        tx_i=0;
        usart_tx_buff[0] = tim_mk;
        USART1->CR1 |= USART_CR1_TCIE;
        tim_mk ++;
        if (tim_mk == 50) tim_mk = 0;
    }

}

//SysTick Interrupt
void SysTick_Handler(void)
{
    static uint32_t del = 0;
    static uint8_t s = 0;

    del++;
  
    if (del == SysTicks_pr) // 1s
    {
        del = 0;
        if (s==0)
        {
            GPIOD->BSRR |= GPIO_BSRR_BR13;
            s=1;
        }else
        {
            GPIOD->BSRR |= GPIO_BSRR_BS13;
            s=0;            
        }       
    }
}

int main(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
    
    RCC->CR |= RCC_CR_HSEON;

    do
    {
        HSEStatus = RCC->CR & RCC_CR_HSERDY;
        StartUpCounter++;
    }    
    while((HSEStatus == 0) && (StartUpCounter != HSE_TIMEOUT_VALUE));
    
    if( (RCC->CR & RCC_CR_HSERDY) != RESET)
    {
        /* Включаем буфер предвыборки FLASH */
        FLASH->ACR |= FLASH_ACR_PRFTEN;

        /* Конфигурируем Flash на 2 цикла ожидания */
    	/* Это нужно потому, что Flash не может работать на высокой частоте */        
    	FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    	FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2WS;   
        
        /* HCLK = SYSCLK || AHB prescaler*/
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1; //AHB clk = 100MHz
        
    	/* PCLK1 = HCLK || APB Low speed prescaler (APB1)*/
    	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV4;

        /* PCLK2 = HCLK || APB high-speed prescaler (APB2)*/
    	RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV2;
        
        /* Set PLL input sourse*/
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
        
        /*Set PLL M prescaler */
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk;
        RCC->PLLCFGR |= (4 << RCC_PLLCFGR_PLLM_Pos);
        
        /*Set PLL N prescaler */
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk;
        RCC->PLLCFGR |= (100 << RCC_PLLCFGR_PLLN_Pos);
        
        /*Set PLL P prescaler */
        //RCC->PLLCFGR |= RCC_PLLCFGR_PLLP;
        
        RCC->CR |= RCC_CR_PLLON;
        
        while ((RCC->CR & RCC_CR_PLLRDY) == 0)
        {}
            
        /*Set SYSCLK as PLL */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= RCC_CFGR_SW_PLL;
            
        while ((RCC->CFGR & RCC_CFGR_SWS) !=  RCC_CFGR_SWS_PLL)
        {}
    }
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN |
                    RCC_AHB1ENR_GPIODEN |
                    RCC_AHB1ENR_GPIOEEN;
    
    //GPIOD PIN12-15 leds
    GPIOD->MODER |= GPIO_MODE_OUTPUT_PP << PIN12*2 |
                    GPIO_MODE_OUTPUT_PP << PIN13*2 |
                    GPIO_MODE_OUTPUT_PP << PIN14*2 |
                    GPIO_MODE_OUTPUT_PP << PIN15*2;
    
    //GPIOB PIN6,7 USART1 TX RX
    GPIOB->MODER |= GPIO_MODE_AF_PP << PIN6*2 |
                    GPIO_MODE_AF_PP << PIN7*2;
    GPIOB->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN6*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN7*2;
    GPIOB->PUPDR |= GPIO_PULLUP << PIN6*2 |
                    GPIO_PULLUP << PIN7*2;
    GPIOB->AFR[0] |= GPIO_AF7_USART1 << PIN6*4 |
                     GPIO_AF7_USART1 << PIN7*4;

    //Set GPIOA PIN1-3, B0-1, C0-1 as analog in
    GPIOA->MODER |= GPIO_MODE_ANALOG << PIN1*2 |
                    GPIO_MODE_ANALOG << PIN2*2 |
                    GPIO_MODE_ANALOG << PIN3*2;
    GPIOB->MODER |= GPIO_MODE_ANALOG << PIN0*2 |
                    GPIO_MODE_ANALOG << PIN1*2;
    GPIOC->MODER |= GPIO_MODE_ANALOG << PIN1*2;    
    GPIOA->PUPDR |= GPIO_NOPULL <<PIN1*2 |
                    GPIO_NOPULL <<PIN2*2 |
                    GPIO_NOPULL <<PIN3*2;
    GPIOB->PUPDR |= GPIO_NOPULL <<PIN0*2 |
                    GPIO_NOPULL <<PIN1*2;
    GPIOC->PUPDR |= GPIO_NOPULL <<PIN1*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_HIGH << PIN1*2 |
                      GPIO_SPEED_FREQ_HIGH << PIN2*2 |
                      GPIO_SPEED_FREQ_HIGH << PIN3*2;
    GPIOB->OSPEEDR |= GPIO_SPEED_FREQ_HIGH << PIN0*2 |
                      GPIO_SPEED_FREQ_HIGH << PIN1*2;
    GPIOC->OSPEEDR |= GPIO_SPEED_FREQ_HIGH << PIN1*2;
    
    //Set GPIOE PIN9,11,13 as TIM1 PWM out
    GPIOE->MODER |= GPIO_MODE_AF_PP << PIN9*2  |
                    GPIO_MODE_AF_PP << PIN11*2 |
                    GPIO_MODE_AF_PP << PIN13*2;
    GPIOE->PUPDR |= GPIO_NOPULL << PIN9*2  |
                    GPIO_NOPULL << PIN11*2 |
                    GPIO_NOPULL << PIN13*2;
    GPIOE->OSPEEDR |= GPIO_SPEED_FREQ_MEDIUM << PIN9*2  |
                      GPIO_SPEED_FREQ_MEDIUM << PIN11*2 |
                      GPIO_SPEED_FREQ_MEDIUM << PIN13*2;
    GPIOE->AFR[1] |= GPIO_AF1_TIM1 << (PIN9*4  - 32) |
                     GPIO_AF1_TIM1 << (PIN11*4 - 32) |
                     GPIO_AF1_TIM1 << (PIN13*4 - 32);   
    
    //GPIOC PIN6 for capture
    GPIOC->MODER |= GPIO_MODE_AF_PP << PIN6*2;
    GPIOA->MODER |= GPIO_MODE_AF_PP << PIN15*2 |
                    GPIO_MODE_AF_PP << PIN0*2;
    GPIOC->PUPDR |= GPIO_NOPULL << PIN6*2;
    GPIOA->PUPDR |= GPIO_NOPULL << PIN15*2 |
                    GPIO_NOPULL << PIN0*2;
    GPIOC->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN6*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN15*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN0*2;
    GPIOC->AFR[0] |= GPIO_AF3_TIM8 << PIN6*4;
    GPIOA->AFR[0] |= GPIO_AF2_TIM5 << PIN0*4;
    GPIOA->AFR[1] |= GPIO_AF1_TIM2 << (PIN15*4-32);
    
    
    //MX_USART2_UART_Init();
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;//APB2 clk=50MHz
    USART1->BRR = APB2/500000;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;   
    
    //TIM6 URQ for uart
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;//APB1_tim clk = 50MHz
    TIM6->PSC = 49999;//clk = 1kHz
    TIM6->ARR = 20;// Tirq=50Hz
    TIM6->DIER = TIM_DIER_UIE;
    TIM6->CR1 = TIM_CR1_CEN;
    
    //ADC1 Init
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //APB2 clk = 50MHz
    ADC1->CR2 = ADC_CR2_ADON | 
                ADC_CR2_DMA |
                ADC_CR2_DDS |
                ADC_CR2_EXTSEL_3 | 
                ADC_CR2_EXTEN_0;// подаем питание на АЦП
    ADC1->CR1 = ADC_CR1_SCAN;// разрешаем прерывания по окончанию преобразования
    ADC1->SMPR2 = ADC_SMPR2_SMP1_0 | //; для всех каналов 3 семпла
                  ADC_SMPR2_SMP2_0 |
                  ADC_SMPR2_SMP3_0 |
                  ADC_SMPR2_SMP8_0 |
                  ADC_SMPR2_SMP9_0;
    ADC1->SMPR1 = ADC_SMPR1_SMP11_0;
    ADC1->SQR1 =  5 << ADC_SQR1_L_Pos; // 6 преобразований (5 + 1)
    ADC1->SQR3 = (1 << ADC_SQR3_SQ1_Pos) | //порядок каналов
                 (2 << ADC_SQR3_SQ2_Pos) |
                 (3 << ADC_SQR3_SQ3_Pos) |
                 (8 << ADC_SQR3_SQ4_Pos) |
                 (9 << ADC_SQR3_SQ5_Pos) |
                (11 << ADC_SQR3_SQ6_Pos);
                   
    //TIM3 for ADC1 Init
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;//50 Mhz
    TIM3->PSC = 49;
    TIM3->ARR = 10;
    TIM3->CR2 = TIM_CR2_MMS_1;
    TIM3->CR1 = TIM_CR1_CEN;

    //DMA Init
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    DMA2_Stream0->CR = DMA_SxCR_CIRC |
                       DMA_SxCR_MINC | 
                       DMA_SxCR_MSIZE_0 | 
                       DMA_SxCR_PSIZE_0;
    DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t) &usart_tx_buff[1];
    DMA2_Stream0->NDTR = 6;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    
    //TIM1 PWM mode
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;//APB2_TIM clk=100MHz
    TIM1->PSC = 9999;
    TIM1->ARR = 99;//100Hz
    TIM1->CCER = TIM_CCER_CC1E |
                 TIM_CCER_CC2E |
                 TIM_CCER_CC3E;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 |
                  TIM_CCMR1_OC1M_2 |
                  TIM_CCMR1_OC2M_1 |
                  TIM_CCMR1_OC2M_2;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 |
                  TIM_CCMR2_OC3M_2;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->CCR1 = 20;
    TIM1->CCR2 = 40;
    TIM1->CCR3 = 60;
    TIM1->CR1 = TIM_CR1_CEN;


    //TIM8 for input capture 1
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;//APB1_TIM clk = 50MHz
    TIM8->CCMR1 = TIM_CCMR1_CC1S_0;
    TIM8->CCER = TIM_CCER_CC1P |
                 TIM_CCER_CC1E;
    TIM8->SMCR = TIM_SMCR_SMS_2 |
                 TIM_SMCR_TS_2;
    TIM8->PSC = 99;
    TIM8->CR1 = TIM_CR1_CEN;
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0;
    TIM2->CCER = TIM_CCER_CC1P |
                 TIM_CCER_CC1E;
    TIM2->SMCR = TIM_SMCR_SMS_2 |
                 TIM_SMCR_TS_2;
    TIM2->PSC = 49;
    TIM2->CR1 = TIM_CR1_CEN;

    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    TIM5->CCMR1 = TIM_CCMR1_CC1S_0;
    TIM5->CCER = TIM_CCER_CC1P |
                 TIM_CCER_CC1E;
    TIM5->SMCR = TIM_SMCR_SMS_2 |
                 TIM_SMCR_TS_2;
    TIM5->PSC = 49;
    TIM5->CR1 = TIM_CR1_CEN;




    SysTick_Config(SysTicks);
    
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    
    GPIOD->BSRR |= GPIO_BSRR_BS12;

    while(1)
    {
        TIM1->CCR1 = usart_rx_buff[1];
        TIM1->CCR2 = usart_rx_buff[2];
        TIM1->CCR3 = usart_rx_buff[3];
        usart_tx_buff[7] = TIM8->CCR1;
        usart_tx_buff[8] = TIM2->CCR1;
        usart_tx_buff[9] = TIM5->CCR1;
        if (usart_rx_buff[0] < 5)
            GPIOD->BSRR |= GPIO_BSRR_BS13;
        else
            GPIOD->BSRR |= GPIO_BSRR_BR13;
    
    }

    return 0;
}
