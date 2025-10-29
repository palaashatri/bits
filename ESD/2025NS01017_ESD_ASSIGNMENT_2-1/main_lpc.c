// ==============================================================================
// FIRMWARE FILE: main_lpc.c
// PROJECT: High-Precision Industrial Environmental Controller (HPIEC)
// TARGET: ARM V4 (LPC2378 ARM7TDMI-S Register Level)
// PERIPHERALS: ADC, Timer/PWM, UART, External Interrupt (VIC)
// DESCRIPTION: Implements real-time temperature monitoring and fan control
//              with an emergency shutdown mechanism on the LPC2378.
// ==============================================================================

#include <stdint.h>
#include <stddef.h>

// Assuming PCLK is 12MHz (standard for Keil simulation setup on LPC2378)
#define PCLK_FREQ           12000000UL
#define CPU_FREQ            PCLK_FREQ

// ==============================================================================
// BASE ADDRESS AND REGISTER DEFINITIONS (LPC2378)
// ==============================================================================

// System Control Block (SCB)
#define SCB_BASE            0xE01FC000UL
#define CLK_CTRL_BASE       0xE01FC000UL
#define PCLKSEL0            (*(volatile uint32_t *)(CLK_CTRL_BASE + 0x1C0))
#define PCLKSEL1            (*(volatile uint32_t *)(CLK_CTRL_BASE + 0x1C4))
#define PCONP               (*(volatile uint32_t *)(CLK_CTRL_BASE + 0x0C4)) // Peripheral Power Control

// GPIO (Pin Connect Block & Port 0/1)
#define PINSEL_BASE         0xE002C000UL
#define PINSEL0             (*(volatile uint32_t *)(PINSEL_BASE + 0x00)) // P0.0 - P0.15
#define PINSEL1             (*(volatile uint32_t *)(PINSEL_BASE + 0x04)) // P0.16 - P0.31
#define PINSEL4             (*(volatile uint32_t *)(PINSEL_BASE + 0x10)) // P2.0 - P2.15

#define GPIO_BASE_0         0xE0028000UL
#define IO0DIR              (*(volatile uint32_t *)(GPIO_BASE_0 + 0x00))
#define IO0SET              (*(volatile uint32_t *)(GPIO_BASE_0 + 0x04))
#define IO0CLR              (*(volatile uint32_t *)(GPIO_BASE_0 + 0x08))
#define IO0PIN              (*(volatile uint32_t *)(GPIO_BASE_0 + 0x14))

#define GPIO_BASE_2         0xE0028000UL // Fast GPIO
#define FIO2DIR             (*(volatile uint32_t *)(GPIO_BASE_2 + 0x50))
#define FIO2SET             (*(volatile uint32_t *)(GPIO_BASE_2 + 0x54))
#define FIO2CLR             (*(volatile uint32_t *)(GPIO_BASE_2 + 0x58))
#define FIO2PIN             (*(volatile uint32_t *)(GPIO_BASE_2 + 0x5C))

// ADC (Using ADC0)
#define AD0CR               (*(volatile uint32_t *)(0xE0034000UL + 0x00)) // Control Register
#define AD0DR0              (*(volatile uint32_t *)(0xE0034000UL + 0x08)) // Data Register for Channel 0

// Timer 0 (For PWM)
#define T0MR1               (*(volatile uint32_t *)(0xE0004000UL + 0x18)) // Match Register 1 (PWM Duty)
#define T0MR0               (*(volatile uint32_t *)(0xE0004000UL + 0x1C)) // Match Register 0 (PWM Period)
#define T0PR                (*(volatile uint32_t *)(0xE0004000UL + 0x0C)) // Prescaler Register
#define T0MCR               (*(volatile uint32_t *)(0xE0004000UL + 0x14)) // Match Control Register
#define T0TCR               (*(volatile uint32_t *)(0xE0004000UL + 0x04)) // Timer Control Register
#define T0CCR               (*(volatile uint32_t *)(0xE0004000UL + 0x28)) // Capture Control Register
#define T0PWMC              (*(volatile uint32_t *)(0xE0004000UL + 0x7C)) // PWM Control Register

// UART0
#define U0LCR               (*(volatile uint32_t *)(0xE000C000UL + 0x00)) // Line Control Register
#define U0DLL               (*(volatile uint32_t *)(0xE000C000UL + 0x00)) // Divisor Latch Low Byte
#define U0DLM               (*(volatile uint32_t *)(0xE000C000UL + 0x04)) // Divisor Latch High Byte
#define U0FCR               (*(volatile uint32_t *)(0xE000C000UL + 0x08)) // FIFO Control Register
#define U0LSR               (*(volatile uint32_t *)(0xE000C000UL + 0x14)) // Line Status Register
#define U0THR               (*(volatile uint32_t *)(0xE000C000UL + 0x00)) // Transmit Holding Register (Write)

// VIC (Vector Interrupt Controller)
#define VICIntEnable        (*(volatile uint32_t *)(0xFFFFF010UL)) // VIC Interrupt Enable
#define VICVectAddr0        (*(volatile uint32_t *)(0xFFFFF100UL)) // Slot 0 Vector Address
#define VICVectCntl0        (*(volatile uint32_t *)(0xFFFFF200UL)) // Slot 0 Control Register
#define VICVectAddr         (*(volatile uint32_t *)(0xFFFFF030UL)) // VIC Vector Address Register (Acknowledge)

// External Interrupt (EINT0)
#define EXTINT              (*(volatile uint32_t *)(0xE01FC140UL)) // External Interrupt Flag Register
#define EXTMODE             (*(volatile uint32_t *)(0xE01FC148UL)) // External Interrupt Mode Register
#define EXTPOLAR            (*(volatile uint32_t *)(0xE01FC14CUL)) // External Interrupt Polarity Register

// ==============================================================================
// GLOBAL VARIABLES AND CONSTANTS
// ==============================================================================

volatile uint8_t emergency_shutdown = 0; // Flag set by EINT0 interrupt
const uint16_t pwm_period = 1000;          // Match 0 value for 1kHz PWM
const float critical_temp_c = 30.0f;       // Temperature control setpoint
const float temp_gain = 5.0f;              // 5% duty per 1 degree over setpoint
const uint32_t adc_max = 1024;             // 10-bit ADC resolution for LPC2378

// ==============================================================================
// FUNCTION PROTOTYPES
// ==============================================================================

void delay_ms(uint32_t ms);
void gpio_init(void);
void adc_init(void);
uint16_t adc_read(void);
void timer0_pwm_init(void);
void uart0_init(void);
void uart_transmit(const char *data);
void exti0_init(void);
void EINT0_IRQHandler(void) __irq; // ARM7 ISR declaration (Changed attribute to __irq)

// ==============================================================================
// CORE LOGIC (main)
// ==============================================================================

int main(void) {
    // Initialise all peripherals
    gpio_init();
    adc_init();
    timer0_pwm_init();
    uart0_init();
    exti0_init();

    // Initial sign-on message for serial validation
    uart_transmit("LPC2378 HPIEC V1.0| Starting System...\r\n");

    // Main Control Loop
    while(1) {
        float temperature_c;
        uint16_t adc_value;
        uint16_t fan_duty;
        
        // 1. Read Sensor Data
        adc_value = adc_read();
        
        // 2. Calculate Temperature (LM35: 10mV/C, 10-bit ADC)
        // Vref = 3.3V (3300mV). Max ADC value = 1024.
        // Temp (C) = (ADC_Value * 3300mV / 1024) / 10mV/C
        temperature_c = (float)adc_value * (330.0f / (float)adc_max); 

        // 3. Control Logic: Proportional Control
        if (temperature_c > critical_temp_c) {
            // Calculate error and proportional gain
            float error = temperature_c - critical_temp_c;
            fan_duty = (uint16_t)(error * temp_gain); // 5% duty per 1 degree setpoint
        } else {
            fan_duty = 0; // Fan OFF if at or below setpoint
        }

        // Clamp duty cycle between 0% and 100%
        if (fan_duty > 100) {
            fan_duty = 100;
        }

        // 4. Actuation
        if (emergency_shutdown) {
            fan_duty = 0;
        }
        
        // Update the Match Register (T0MR1) to adjust fan speed
        // T0MR1 controls the duty cycle. The total period is T0MR0.
        T0MR1 = (uint16_t)(((float)fan_duty / 100.0) * pwm_period);

        // 5. Communication and Monitoring
        if (!emergency_shutdown) {
            char buffer[64];
            uint16_t temp_int = (uint16_t)temperature_c;
            uint16_t temp_frac = (uint16_t)((temperature_c - temp_int) * 10.0f);
            
            uart_transmit("Temp: ");
            buffer[0] = (temp_int / 10) + '0'; buffer[1] = (temp_int % 10) + '0'; buffer[2] = '.';
            buffer[3] = temp_frac + '0'; buffer[4] = 'C'; buffer[5] = ','; buffer[6] = ' ';
            buffer[7] = 'F'; buffer[8] = 'a'; buffer[9] = 'n'; buffer[10] = ' '; buffer[11] = 'D'; buffer[12] = 'u'; buffer[13] = 't'; buffer[14] = 'y'; buffer[15] = ':'; buffer[16] = ' ';
            uart_transmit(buffer); 
            
            buffer[0] = (fan_duty / 100) % 10 + '0'; 
            buffer[1] = (fan_duty / 10) % 10 + '0'; 
            buffer[2] = (fan_duty % 10) + '0'; 
            buffer[3] = '%'; buffer[4] = ' '; buffer[5] = '\r'; buffer[6] = '\n'; buffer[7] = '\0';
            uart_transmit(buffer); 
        }

        // 6. Loop Delay
        delay_ms(500); 
    }
}

// ==============================================================================
// PERIPHERAL INITIALIZATION AND DRIVERS
// ==============================================================================

/**
 * @brief Simple delay function based on the main CPU frequency.
 * @param ms Milliseconds to delay.
 */
void delay_ms(uint32_t ms) {
    // Crude busy-wait delay. Adjust loop limit based on CPU frequency
    volatile uint32_t i;
    for (i = 0; i < (CPU_FREQ / 8000 * ms); i++); 
}

/**
 * @brief Initialises Pin Connect Block (PCB) and GPIO.
 * Pin Assignments:
 * - P0.2: TXD0 (UART0) -> Alternate Function 1
 * - P0.23: AD0.0 (ADC input) -> Alternate Function 1
 * - P0.22: MAT0.1 (PWM Output) -> Alternate Function 2
 * - P2.10: EINT0 (External Interrupt Input) -> Alternate Function 1
 */
void gpio_init(void) {
    // Enable power for peripherals used (GPIO is always on)
    PCONP |= (1 << 1) | (1 << 8) | (1 << 12); // PCTIM0 (T0), PCUART0 (U0), PCADC (AD0)

    // 1. P0.2 for TXD0 (UART0) - PINSEL0 bits 4:5
    PINSEL0 &= ~(0b11 << 4);
    PINSEL0 |= (0b01 << 4);  // AF1

    // 2. P0.23 for AD0.0 (ADC Channel 0) - PINSEL1 bits 14:15
    PINSEL1 &= ~(0b11 << 14);
    PINSEL1 |= (0b01 << 14); // AF1

    // 3. P0.22 for MAT0.1 (PWM Output) - PINSEL1 bits 12:13
    PINSEL1 &= ~(0b11 << 12);
    PINSEL1 |= (0b10 << 12); // AF2

    // 4. P2.10 for EINT0 (External Interrupt 0) - PINSEL4 bits 20:21
    PINSEL4 &= ~(0b11 << 20);
    PINSEL4 |= (0b01 << 20); // AF1 (EINT0)

    // Set direction for P0.22 (PWM) to output
    IO0DIR |= (1 << 22);

    // Set direction for P2.10 (EINT0) to input (default, but explicit)
    FIO2DIR &= ~(1 << 10);
}

/**
 * @brief Initialises ADC0 Channel 0 for single, 10-bit conversion.
 */
void adc_init(void) {
    // PCLK for AD0 is set to PCLK_FREQ (12MHz) by default
    
    // AD0CR:
    // Bits 7:0: SEL (Select channel 0) -> 0x01
    // Bits 15:8: CLKDIV (Divide PCLK by 12+1 = 13. ADCCLK = 12MHz/13 ~ 923kHz) -> 0x0C (12)
    // Bit 16: BURST (0 for software mode)
    // Bit 21: PDN (Power Down, 1 to operate)
    // Bits 26:24: START (000 for no start)
    // Bits 31:27: EDGE (0)
    AD0CR = (0x01 << 0) | (0x0C << 8) | (1 << 21); // Channel 0, CLKDIV 12, PDN ON
}

/**
 * @brief Triggers and reads a single 10-bit ADC conversion.
 * @return The 10-bit digital value (0-1023).
 */
uint16_t adc_read(void) {
    // Start conversion (START=0b001, single conversion on AD0.0)
    AD0CR |= (0b001 << 24);

    // Wait for DONE flag (Bit 31 of AD0DR0)
    // FIX: Cast 1 to unsigned (1UL) to avoid warning about signed overflow
    while (!(AD0DR0 & (1UL << 31))) { 
        // Wait...
    }
    
    // Clear the START bits
    AD0CR &= ~(0b111 << 24);

    // Read the Data Register, shifting to get 10-bit result (Bits 15:6)
    return (uint16_t)((AD0DR0 >> 6) & 0x3FF); // 0x3FF is 10 bits (0-1023)
}

/**
 * @brief Initialises Timer 0 for 1kHz PWM signal on P0.22 (MAT0.1).
 */
void timer0_pwm_init(void) {
    // 1. Reset and enable Timer 0
    T0TCR = 0x02; // Reset
    T0TCR = 0x00; // Disable (will enable later)

    // 2. Configure Timebase
    // Prescaler: 0 (No prescale, TCLK = PCLK = 12MHz)
    T0PR = 0; 
    
    // Match 0: Period (12MHz / 1000Hz = 12000)
    T0MR0 = PCLK_FREQ / 1000; // 12000
    
    // Match Control Register (T0MCR)
    // MR0: Interrupt (Bit 0) and Reset (Bit 1) on match 0
    T0MCR = (1 << 0) | (1 << 1); 

    // 3. Configure PWM Control (T0PWMC)
    // Enable PWM on Match Channel 1 (Bit 2)
    T0PWMC = (1 << 1);

    // 4. Set initial duty cycle to 0% (Fan OFF)
    T0MR1 = 0;

    // 5. Enable Timer
    T0TCR = 0x01; // Enable Timer
}

/**
 * @brief Initialises UART0 for 9600 Baud Rate.
 */
void uart0_init(void) {
    // 1. PCLK for UART0 is 12MHz (default)
    
    // 2. Set LCR (Line Control Register) to enable Divisor Latch Access Bit (DLAB)
    U0LCR = (1 << 7); 
    
    // 3. Set Baud Rate (9600 for 12MHz PCLK)
    // Divisor = PCLK / (16 * Baud Rate) = 12,000,000 / (16 * 9600) = 78.125
    // Divisor Latch Value (DLVAL) = 78 (0x4E)
    U0DLL = 78; // LSB
    U0DLM = 0;  // MSB (0)

    // 4. Configure LCR: DLAB=0, 8-bit data, No parity, 1 stop bit
    U0LCR = (0b11 << 0); 

    // 5. FIFO Control Register (FCR): Enable FIFOs (Bit 0) and reset them
    U0FCR = (1 << 0) | (1 << 1) | (1 << 2);
}

/**
 * @brief Transmits a null-terminated string via UART0.
 * @param data Null-terminated string to transmit.
 */
void uart_transmit(const char *data) {
    for (size_t i = 0; data[i] != '\0'; i++) {
        // 1. Wait until the Transmit Holding Register Empty (THRE) flag is set (U0LSR Bit 5)
        while (!(U0LSR & (1 << 5))) {
            // Wait...
        }
        // 2. Write data to the Transmit Holding Register
        U0THR = (data[i] & 0xFF);
    }
}

/**
 * @brief Initialises External Interrupt 0 (EINT0) on P2.10.
 */
void exti0_init(void) {
    // 1. Configure EINT0 (P2.10) to be edge triggered and active low (falling edge)
    EXTMODE |= (1 << 0);     // EINT0 is edge sensitive (Bit 0)
    EXTPOLAR &= ~(1 << 0);   // EINT0 is falling edge triggered (Bit 0 = 0)

    // 2. VIC Configuration (ARM V4 Feature: Vector Interrupt Controller)
    // Select a VIC slot (Slot 0)
    VICVectCntl0 = (1 << 5) | 14; // Enable slot (Bit 5) and assign EINT0 IRQ number (14)
    
    // Assign interrupt service routine (ISR) address
    VICVectAddr0 = (uint32_t)EINT0_IRQHandler; 

    // 3. Enable EINT0 interrupt line in the VIC
    VICIntEnable |= (1 << 14); // EINT0 IRQ is 14
}

// ==============================================================================
// INTERRUPT HANDLER (ARM V4 Feature: IRQ Handler Declaration)
// ==============================================================================

/**
 * @brief Interrupt Service Routine for External Interrupt 0 (EINT0).
 */
void EINT0_IRQHandler(void) __irq { // ADDED __irq HERE
    // 1. Set the global flag for emergency shutdown
    emergency_shutdown = 1;
    
    // 2. Force fan duty cycle to 0 (Actuation)
    T0MR1 = 0; 
    
    // 3. Communication trace
    uart_transmit("*** EMERGENCY SHUTDOWN TRIGGERED! (LPC2378) ***\r\n");
    uart_transmit("Fan Duty set to 0% (Forced Stop).\r\n");

    // 4. Clear the interrupt flag by writing a '1' to it
    EXTINT = (1 << 0); // Clear EINT0 flag

    // 5. Acknowledge and re-enable VIC
    VICVectAddr = 0; 
}
