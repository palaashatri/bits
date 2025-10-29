/**
 * @file main.c
 * @brief High-Precision Industrial Environmental Controller (HPIEC) Firmware
 *
 * This code implements a temperature monitoring and fan control system for an ARM Cortex-M4
 * based MCU (e.g., STM32F4xx) using register-level programming.
 *
 * Peripherals Used:
 * 1. ADC: Read temperature sensor (LM35).
 * 2. TIM2: Generate PWM for cooling fan speed control.
 * 3. EXTI0: External Interrupt for Emergency Shutdown Button.
 * 4. UART2: Transmit status data to a terminal.
 *
 * Keil $\mu$Vision 5 Simulation Target: STM32F401/F411
 * Author: Palaash Atri (2025NS01017)
 */

#include <stdint.h>
#include <stdio.h> // Required for snprintf

// --- Register Base Addresses (STM32F4xx Common) ---

#define RCC_BASE        0x40023800UL
#define GPIOA_BASE      0x40020000UL
#define GPIOB_BASE      0x40020400UL
#define ADC1_BASE       0x40012000UL
#define TIM2_BASE       0x40000000UL
#define USART2_BASE     0x40004400UL
#define EXTI_BASE       0x40013C00UL
#define SYSCFG_BASE     0x40013800UL
#define NVIC_BASE       0xE000E100UL // NVIC base address

// --- Register Definitions ---

// RCC Registers (Clocks)
#define RCC_AHB1ENR     (*((volatile uint32_t *)(RCC_BASE + 0x30)))
#define RCC_APB1ENR     (*((volatile uint32_t *)(RCC_BASE + 0x40)))
#define RCC_APB2ENR     (*((volatile uint32_t *)(RCC_BASE + 0x44)))

// GPIOA/GPIOB Registers
#define GPIOA_MODER     (*((volatile uint32_t *)(GPIOA_BASE + 0x00)))
#define GPIOA_AFRL      (*((volatile uint32_t *)(GPIOA_BASE + 0x20)))
#define GPIOB_MODER     (*((volatile uint32_t *)(GPIOB_BASE + 0x00)))
#define GPIOB_PUPDR     (*((volatile uint32_t *)(GPIOB_BASE + 0x0C)))

// ADC1 Registers
#define ADC_SR          (*((volatile uint32_t *)(ADC1_BASE + 0x00))) // Status Register (needed for EOC flag)
#define ADC_CR2         (*((volatile uint32_t *)(ADC1_BASE + 0x08)))
#define ADC_SMPR2       (*((volatile uint32_t *)(ADC1_BASE + 0x0C)))
#define ADC_SQR3        (*((volatile uint32_t *)(ADC1_BASE + 0x34)))
#define ADC_DR          (*((volatile uint32_t *)(ADC1_BASE + 0x4C)))

// TIM2 Registers
#define TIM_CCMR1       (*((volatile uint32_t *)(TIM2_BASE + 0x18)))
#define TIM_CCER        (*((volatile uint32_t *)(TIM2_BASE + 0x20)))
#define TIM_PSC         (*((volatile uint32_t *)(TIM2_BASE + 0x24)))
#define TIM_ARR         (*((volatile uint32_t *)(TIM2_BASE + 0x2C)))
#define TIM_CCR1        (*((volatile uint32_t *)(TIM2_BASE + 0x34)))
#define TIM_CR1         (*((volatile uint32_t *)(TIM2_BASE + 0x00)))

// USART2 Registers
#define USART_SR        (*((volatile uint32_t *)(USART2_BASE + 0x00)))
#define USART_DR        (*((volatile uint32_t *)(USART2_BASE + 0x04)))
#define USART_BRR       (*((volatile uint32_t *)(USART2_BASE + 0x08))) // CORRECTED: Was TIM_BRR
#define USART_CR1       (*((volatile uint32_t *)(USART2_BASE + 0x0C)))

// EXTI Registers
#define EXTI_IMR        (*((volatile uint32_t *)(EXTI_BASE + 0x00)))
#define EXTI_FTSR       (*((volatile uint32_t *)(EXTI_BASE + 0x0C)))
#define EXTI_PR         (*((volatile uint32_t *)(EXTI_BASE + 0x14)))

// SYSCFG Registers
#define SYSCFG_EXTICR1  (*((volatile uint32_t *)(SYSCFG_BASE + 0x08)))

// NVIC Registers (Cortex-M4 Feature)
#define NVIC_ISER0      (*((volatile uint32_t *)(NVIC_BASE + 0x100))) // Interrupt Set-Enable Register
#define NVIC_IPR1       (*((volatile uint32_t *)(NVIC_BASE + 0x004))) // Interrupt Priority Register for EXTI0

// --- Global Variables ---
volatile uint8_t emergency_shutdown = 0;
uint16_t pwm_period = 1000; // PWM resolution: 1000 steps

// --- Function Prototypes ---
void uart_transmit(char *data);
void set_fan_duty(uint8_t duty); // Duty: 0-100
uint16_t adc_read(void);
void delay_ms(uint32_t ms);

// --- Initialisation Functions ---

/**
 * @brief Initializes GPIO clocks and pin modes.
 */
void gpio_init(void) {
    // 1. Enable GPIOA and GPIOB clocks (RCC_AHB1ENR)
    RCC_AHB1ENR |= (1 << 0) | (1 << 1); // Enable GPIOA and GPIOB

    // PA0 (ADC): Analog Mode (0b11)
    GPIOA_MODER |= (3 << (0 * 2));

    // PA5 (TIM2_CH1 PWM): Alternate Function Mode (0b10)
    GPIOA_MODER &= ~(3 << (5 * 2));
    GPIOA_MODER |= (2 << (5 * 2));
    GPIOA_AFRL |= (1 << (5 * 4));    // AF1 for TIM2_CH1

    // PA2 (UART2_TX): Alternate Function Mode (0b10)
    GPIOA_MODER &= ~(3 << (2 * 2));
    GPIOA_MODER |= (2 << (2 * 2));
    GPIOA_AFRL |= (7 << (2 * 4));    // AF7 for UART2

    // PB0 (EXTI0): Input Mode (0b00) with Pull-Down (0b10)
    GPIOB_MODER &= ~(3 << (0 * 2));
    GPIOB_PUPDR |= (2 << (0 * 2));
}

/**
 * @brief Initializes ADC1 for single, 12-bit conversion on Channel 0 (PA0).
 */
void adc_init(void) {
    // 1. Enable ADC1 clock (RCC_APB2ENR, Bit 8)
    RCC_APB2ENR |= (1 << 8);

    // 2. Set Channel 0 (PA0) as the only sequence (SQR3)
    ADC_SQR3 &= ~(0x1F);
    ADC_SQR3 |= 0; // Channel 0 is selected

    // 3. Set Sample Time (SMPR2)
    ADC_SMPR2 |= (3 << (0 * 3)); // 28 cycles

    // 4. Turn ON the ADC (CR2, Bit 0)
    ADC_CR2 |= (1 << 0);
}

/**
 * @brief Initializes TIM2 for PWM generation on Channel 1 (PA5).
 */
void tim2_pwm_init(void) {
    // 1. Enable TIM2 clock (RCC_APB1ENR, Bit 0)
    RCC_APB1ENR |= (1 << 0);

    // 2. Set Prescaler (PSC) and Auto-Reload Register (ARR)
    // PCLK1 = 42MHz. PSC = 4200 - 1. ARR = 1000 - 1. F_PWM = 10 kHz
    TIM_PSC = 4200 - 1;
    TIM_ARR = pwm_period - 1;

    // 3. Set Channel 1 as PWM Mode 1 (TIM_CCMR1, OC1M) and enable Preload (OC1PE)
    TIM_CCMR1 |= (6 << 4) | (1 << 3);

    // 4. Enable Output Compare 1 (TIM_CCER, CC1E bit 0)
    TIM_CCER |= (1 << 0);

    // 5. Initialize CCR1 to 0 (Fan Off) and Enable Timer
    TIM_CCR1 = 0;
    TIM_CR1 |= (1 << 0);
}

/**
 * @brief Initializes UART2 for 9600 baud, 8N1 transmission.
 */
void uart2_init(void) {
    // 1. Enable USART2 clock (RCC_APB1ENR, Bit 17)
    RCC_APB1ENR |= (1 << 17);

    // 2. Set Baud Rate (9600 @ 42MHz PCLK1)
    // BRR value = 273.4375 -> 0x1117
    USART_BRR = (273 << 4) | 7; // CORRECTED REGISTER USAGE

    // 3. Enable Transmitter (TE) and UART (UE)
    USART_CR1 |= (1 << 3) | (1 << 13);
}

/**
 * @brief Initializes External Interrupt Line 0 for PB0 (Falling Edge).
 */
void exti0_init(void) {
    // 1. Enable SYSCFG clock (RCC_APB2ENR, Bit 14)
    RCC_APB2ENR |= (1 << 14);

    // 2. Map EXTI Line 0 to Port B (PB0) in SYSCFG_EXTICR1
    SYSCFG_EXTICR1 |= (1 << 0);

    // 3. Unmask the interrupt line (EXTI_IMR, Bit 0)
    EXTI_IMR |= (1 << 0);

    // 4. Configure trigger to Falling Edge (EXTI_FTSR, Bit 0)
    EXTI_FTSR |= (1 << 0);

    // 5. **Cortex-M4 Feature: NVIC Configuration**
    // Set Priority 0 (highest) for EXTI0 (IRQ 6)
    NVIC_IPR1 &= ~(0xFF << 8);  // Clear old priority bits
    NVIC_IPR1 |= (0x00 << 8);   // Set the highest priority (0)
    
    // Enable EXTI0 interrupt in NVIC (ISER0, Bit 6)
    NVIC_ISER0 |= (1 << 6);
}

// --- Utility Functions ---

/**
 * @brief Transmits a null-terminated string over UART2.
 */
void uart_transmit(char *data) {
    for (int i = 0; data[i] != '\0'; i++) {
        // Wait until TXE flag is set (SR, Bit 7)
        while (!(USART_SR & (1 << 7)));
        // Write the character
        USART_DR = (data[i] & 0xFF);
    }
}

/**
 * @brief Sets the fan speed using PWM duty cycle.
 */
void set_fan_duty(uint8_t duty) {
    if (duty > 100) duty = 100;
    
    // Calculate CCR1 value: (Duty / 100) * ARR
    uint16_t ccr_val = (uint16_t)(((float)duty / 100.0) * pwm_period);
    
    // Update the Capture/Compare Register 1
    TIM_CCR1 = ccr_val;
}

/**
 * @brief Performs a single ADC conversion on Channel 0.
 * @return 12-bit digital value.
 */
uint16_t adc_read(void) {
    // Start the conversion (CR2, Bit 3)
    ADC_CR2 |= (1 << 3);

    // Wait for EOC (End of Conversion, Bit 1 in ADC_SR)
    while (!(ADC_SR & (1 << 1))); // CORRECTED REGISTER USAGE

    // Read the Data Register (clears the EOC flag automatically on read)
    return (uint16_t)(ADC_DR & 0xFFF);
}

/**
 * @brief Simple software delay (in milliseconds).
 */
void delay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 5000; j++); 
    }
}


// --- Main Program ---
int main(void) {
    // 1. Initialise all peripherals
    gpio_init();
    adc_init();
    tim2_pwm_init();
    uart2_init();
    exti0_init();

    uart_transmit("[HPIEC v1.0] Starting System...\r\n");

    while (1) {
        if (emergency_shutdown) {
            delay_ms(100);
            continue;
        }

        // --- 1. Read Sensor Data ---
        uint16_t adc_value = adc_read();

        // --- 2. Calculate Temperature (LM35: Vref=3.3V, 12-bit ADC) ---
        // T_C = ADC_Value * (3300mV / 4096) / 10mV
        float temperature_c = (float)adc_value * (330.0f / 4096.0f); 

        // --- 3. Control Logic (Proportional Control) ---
        uint8_t fan_duty = 0;
        float setpoint = 30.0f; // Critical threshold
        float error = temperature_c - setpoint;

        if (error > 0) {
            // Simple proportional gain: 10% duty per 1 degree over setpoint
            fan_duty = (uint8_t)(error * 10.0f);

            if (fan_duty > 100) {
                fan_duty = 100;
            }
        }

        // --- 4. Actuation ---
        set_fan_duty(fan_duty);

        // --- 5. Data Transmission ---
        char tx_buffer[64];
        if (fan_duty == 0) {
            snprintf(tx_buffer, sizeof(tx_buffer), "Temp: %.1fC, Fan Duty: %d%% (STABLE)\r\n", temperature_c, fan_duty);
        } else if (fan_duty == 100) {
            snprintf(tx_buffer, sizeof(tx_buffer), "Temp: %.1fC, Fan Duty: %d%% (MAX COOL)\r\n", temperature_c, fan_duty);
        } else {
            snprintf(tx_buffer, sizeof(tx_buffer), "Temp: %.1fC, Fan Duty: %d%%\r\n", temperature_c, fan_duty);
        }
        uart_transmit(tx_buffer);

        // Control loop delay (50ms execution time)
        delay_ms(50); 
    }
}

// --- Interrupt Service Routine (Cortex-M4/NVIC) ---

/**
 * @brief EXTI Line 0 Interrupt Handler.
 * Triggered by the Emergency Shutdown Button (PB0 falling edge).
 */
void EXTI0_IRQHandler(void) {
    if (EXTI_PR & (1 << 0)) {
        // Clear the pending bit
        EXTI_PR |= (1 << 0);

        // --- CRITICAL SAFETY LOGIC ---
        emergency_shutdown = 1; // Set global flag
        set_fan_duty(0);        // Force fan OFF immediately
        uart_transmit("*** EMERGENCY SHUTDOWN TRIGGERED! ***\r\n");
        uart_transmit("Fan Duty set to 0% (Forced Stop).\r\n");
    }
}
