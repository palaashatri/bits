#include "stm32f4xx.h"
#include <stdio.h>

// UART transmit helper
void uart2_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= (2 << 4); // PA2 as alternate function
    GPIOA->AFR[0] |= (7 << 8); // AF7 for USART2

    USART2->BRR = 0x1117; // Baud rate ~9600
    USART2->CR1 |= USART_CR1_TE | USART_CR1_UE;
}

void uart_transmit(char *data) {
    while (*data) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = *data++;
    }
}

// ADC init
void adc_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER |= (3 << 0); // PA0 analog mode

    ADC1->SQR3 = 0; // Channel 0
    ADC1->CR2 |= ADC_CR2_ADON;
}

// Timer for PWM (TIM4 CH1)
void tim4_pwm_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    GPIOD->MODER |= (2 << 24); // PD12 alternate
    GPIOD->AFR[1] |= (2 << 16); // AF2 for TIM4

    TIM4->PSC = 84 - 1;
    TIM4->ARR = 1000;
    TIM4->CCMR1 |= (6 << 4); // PWM mode 1
    TIM4->CCER |= TIM_CCER_CC1E;
    TIM4->CR1 |= TIM_CR1_CEN;
}

// Timer for periodic interrupt (TIM3)
void tim3_interrupt_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 8400 - 1;
    TIM3->ARR = 10000 - 1;
    TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM3_IRQn);
}

// ISR: sample ADC, compute temp, adjust PWM, log UART
void TIM3_IRQHandler(void) {
    if (TIM3->SR & TIM_SR_UIF) {
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC));
        uint16_t adc_val = ADC1->DR;

        float temp_c = (float)adc_val * (330.0f / 4096.0f);
        uint16_t duty = (temp_c > 30.0f) ? 100 : 0;
        uint16_t ccr_val = (uint16_t)(((float)duty / 100.0f) * TIM4->ARR);
        TIM4->CCR1 = ccr_val;

        char msg[64];
        sprintf(msg, "Temp: %.1fC, Fan Duty: %d%%\r\n", temp_c, duty);
        uart_transmit(msg);

        TIM3->SR &= ~TIM_SR_UIF;
    }
}

int main(void) {
    uart2_init();
    adc_init();
    tim4_pwm_init();
    tim3_interrupt_init();

    while (1) {
        // Main loop does nothing; all work in ISR
    }
}
