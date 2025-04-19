/*
* STM32 Traffic Light System with Pedestrian Walk Feature
*
* RED LED: PC8
* YELLOW LED: PC6
* GREEN LED: PC5
* PEDESTRIAN (WHITE) LED: PC7
* BUTTON: PA9
*
* State Sequence:
*   RED (~1s), RED+YELLOW (~0.25s), GREEN (~1s), YELLOW (~0.25s), repeat
*   Pedestrian request via PA9 toggles 'walk' flag; activates WHITE LED during RED state.
*/

#include <stdint.h>

/* Global state variables */ 
volatile uint8_t light_state = 0;  // Tracks current light phase (1–4)
volatile uint8_t walk = 0;         // Pedestrian walk flag, toggled by button press
  
/* State identifiers */
#define RED 1
#define RED_YELLOW 2 
#define GREEN 3 
#define YELLOW 4
 
/* Timing constants (based on SysTick clock ticks) */ 
#define LIGHT 16000000       // ~1s 
#define TRANSITION 4000000   // ~0.25s
 
/* Base addresses for peripherals */
#define RCC_BASE       0x40023800 
#define GPIOC_BASE     0x40020800
#define GPIOA_BASE     0x40020000
#define SYSTICK_BASE   0xE000E010
#define SYSCFG_BASE    0x40013800
#define NVIC_BASE      0xE000E100
#define EXTI_BASE      0x40013C00
 
/* RCC register */
#define RCC_AHB1ENR    (*(volatile uint32_t*)(RCC_BASE + 0x30))
 
/* SYSCFG register for EXTI configuration */
#define SYSCFG_EXTICR3 (*(volatile uint32_t*)(SYSCFG_BASE + 0x10))
 
/* EXTI registers for interrupt setup */ 
#define EXTI_IMR       (*(volatile uint32_t*)(EXTI_BASE + 0x00))
#define EXTI_EMR       (*(volatile uint32_t*)(EXTI_BASE + 0x04))
#define EXTI_RTSR      (*(volatile uint32_t*)(EXTI_BASE + 0x08))
#define EXTI_FTSR      (*(volatile uint32_t*)(EXTI_BASE + 0x0C))
#define EXTI_PR        (*(volatile uint32_t*)(EXTI_BASE + 0x14))
 
/* NVIC register for enabling IRQs */ 
#define NVIC_ISER0     (*(volatile uint32_t*)(NVIC_BASE + 0x00))
 
/* GPIOC registers */
#define GPIOC_MODER    (*(volatile uint32_t*)(GPIOC_BASE + 0x00))
#define GPIOC_OTYPER   (*(volatile uint32_t*)(GPIOC_BASE + 0x04))
#define GPIOC_OSPEEDR  (*(volatile uint32_t*)(GPIOC_BASE + 0x08))
#define GPIOC_PUPDR    (*(volatile uint32_t*)(GPIOC_BASE + 0x0C))
#define GPIOC_ODR      (*(volatile uint32_t*)(GPIOC_BASE + 0x14))
 
/* GPIOA registers */ 
#define GPIOA_MODER    (*(volatile uint32_t*)(GPIOA_BASE + 0x00))
#define GPIOA_PUPDR    (*(volatile uint32_t*)(GPIOA_BASE + 0x0C))
#define GPIOA_IDR      (*(volatile uint32_t*)(GPIOA_BASE + 0x14))
 
/* SysTick registers */
#define SYSTICK_CTRL    (*(volatile uint32_t*)(SYSTICK_BASE + 0x00))
#define SYSTICK_RELOAD  (*(volatile uint32_t*)(SYSTICK_BASE + 0x04))
#define SYSTICK_CURRENT (*(volatile uint32_t*)(SYSTICK_BASE + 0x08))
 
/* Function prototypes */ 
void RCC_Init(void);
void GPIOC_Init(void);
void GPIOA_Init(void);
void SysTick_Init(void);
void SysTick_Set_Counter(uint32_t);
void EXTI9_Init(void);
void set_RED(void);
void set_RED_YELLOW(void);
void set_GREEN(void);
void set_YELLOW(void);
void set_RED_WHITE(void);
 
/* Main function */ 
int main(void) { 
	RCC_Init();        // Enable GPIOA & GPIOC clocks
	GPIOC_Init();      // Configure PC5, PC6, PC7, PC8 as outputs
	GPIOA_Init();      // Configure PA9 as input with pull-down
	SysTick_Init();    // Setup SysTick timer for state transitions
	EXTI9_Init();      // Enable external interrupt for PA9 (button)

	while (1);         // Main loop does nothing; logic runs in ISRs 
}
 
/* Enable GPIOA and GPIOC peripheral clocks */
void RCC_Init(void) {
 	RCC_AHB1ENR |= (1 << 2); // Enable GPIOC clock
 	RCC_AHB1ENR |= (1 << 0); // Enable GPIOA clock
 
}
 
/* Configure PC5-PC8 as push-pull outputs */
void GPIOC_Init(void) {
 	// Clear mode bits
 	GPIOC_MODER &= ~(3 << (5 * 2));
 	GPIOC_MODER &= ~(3 << (6 * 2));
	GPIOC_MODER &= ~(3 << (7 * 2));
 	GPIOC_MODER &= ~(3 << (8 * 2));
 
	// Set as general-purpose output
 	GPIOC_MODER |= (1 << (5 * 2));
 	GPIOC_MODER |= (1 << (6 * 2));
	GPIOC_MODER |= (1 << (7 * 2));
 	GPIOC_MODER |= (1 << (8 * 2));
 
	// Output type: push-pull
 	GPIOC_OTYPER &= ~(1 << 5);
 	GPIOC_OTYPER &= ~(1 << 6);
 	GPIOC_OTYPER &= ~(1 << 7);
 	GPIOC_OTYPER &= ~(1 << 8);

	// Low speed and no pull-up/pull-down
 	GPIOC_OSPEEDR &= ~(3 << (5 * 2));
 	GPIOC_OSPEEDR &= ~(3 << (6 * 2));
 	GPIOC_OSPEEDR &= ~(3 << (7 * 2));
 	GPIOC_OSPEEDR &= ~(3 << (8 * 2));
 
	GPIOC_PUPDR &= ~(3 << (5 * 2));
	GPIOC_PUPDR &= ~(3 << (6 * 2));
 	GPIOC_PUPDR &= ~(3 << (7 * 2));
 	GPIOC_PUPDR &= ~(3 << (8 * 2));
}
 
/* Configure PA9 as input with pull-down resistor */
void GPIOA_Init(void) {
 	GPIOA_MODER &= ~(3 << (9 * 2));  // Set PA9 as input
 	GPIOA_PUPDR &= ~(3 << (9 * 2));
 	GPIOA_PUPDR |=  (2 << (9 * 2));  // Enable pull-down
}
 
/* Initialize SysTick for periodic interrupts */
void SysTick_Init(void) {
 	SysTick_Set_Counter(LIGHT);     // Set initial countdown (~3s)
 	SYSTICK_CTRL |= (1 << 2);       // Clock source = processor clock
	SYSTICK_CTRL |= (1 << 1);       // Enable SysTick interrupt
	SYSTICK_CTRL |= (1 << 0);       // Enable SysTick counter
}
 
/* Set SysTick timer reload value */
void SysTick_Set_Counter(uint32_t ticks) {
	SYSTICK_RELOAD = ticks - 1;
 	SYSTICK_CURRENT = 0;
 }
 
/* SysTick interrupt handler: cycles through traffic light states */
void SysTick_Handler(void) {
	light_state = (light_state % 4) + 1;
 
	switch(light_state) {
 		case RED:
			set_RED();
 				if(walk) {
 					set_RED_WHITE(); // Show pedestrian signal
					walk = 0;        // Clear pedestrian request
				}
			SysTick_Set_Counter(LIGHT);
			break;

 		case RED_YELLOW:
 			set_RED_YELLOW();
 			SysTick_Set_Counter(TRANSITION);
			break;
 
		case GREEN:
 			set_GREEN();
 			SysTick_Set_Counter(LIGHT);
 			break;
 
		case YELLOW:
 			set_YELLOW();
 			SysTick_Set_Counter(TRANSITION);
 			break;
	}
}
 
/* Initialize external interrupt for PA9 (EXTI Line 9) */
void EXTI9_Init(void) {
 	NVIC_ISER0 |= (1 << 23);      // Enable EXTI9_5 interrupt in NVIC
	SYSCFG_EXTICR3 &= ~(0xF << 4); // Map EXTI9 to PA9
	EXTI_FTSR &= ~(1 << 9);       // Disable falling edge trigger
 	EXTI_EMR &= ~(1 << 9);        // Disable event
 	EXTI_RTSR |= (1 << 9);        // Enable rising edge trigger
 	EXTI_IMR  |= (1 << 9);        // Unmask interrupt for line 9
}
 
/* EXTI9_5 interrupt handler: toggle pedestrian walk request */
void EXTI9_5_IRQHandler(void) {
 	EXTI_PR |= (1 << 9); // Clear interrupt pending flag
 	walk ^= 1;           // Toggle walk flag (may replace with walk = 1 for simplicity)
}
 
/* Activate RED light only */
void set_RED(void) {
 	GPIOC_ODR |= (1 << 8);   // Red ON
 	GPIOC_ODR &= ~(1 << 6);  // Yellow OFF
 	GPIOC_ODR &= ~(1 << 5);  // Green OFF
 	GPIOC_ODR &= ~(1 << 7);  // Pedestrian (White) OFF
}
 
/* Activate RED + YELLOW lights (before GREEN) */
void set_RED_YELLOW(void) {
 	GPIOC_ODR |= (1 << 6);   // Yellow ON
	GPIOC_ODR |= (1 << 8);   // Red ON (still active)
 	GPIOC_ODR &= ~(1 << 5);  // Green OFF
 	GPIOC_ODR &= ~(1 << 7);  // Pedestrian (White) OFF
}
 
/* Activate GREEN light only */
void set_GREEN(void) {
 	GPIOC_ODR |= (1 << 5);   // Green ON
 	GPIOC_ODR &= ~(1 << 8);  // Red OFF
 	GPIOC_ODR &= ~(1 << 6);  // Yellow OFF
 	GPIOC_ODR &= ~(1 << 7);  // Pedestrian (White) OFF
}
 
/* Activate YELLOW light only (after GREEN) */
void set_YELLOW(void) {
 	GPIOC_ODR |= (1 << 6);   // Yellow ON
 	GPIOC_ODR &= ~(1 << 5);  // Green OFF
 	GPIOC_ODR &= ~(1 << 8);  // Red OFF
 	GPIOC_ODR &= ~(1 << 7);  // Pedestrian (White) OFF
}
 
/* Activate RED + WHITE (pedestrian walk signal) */ 
void set_RED_WHITE(void) {
 	GPIOC_ODR |= (1 << 8);   // Red ON
 	GPIOC_ODR |= (1 << 7);   // White ON (pedestrian)
 	GPIOC_ODR &= ~(1 << 6);  // Yellow OFF
 	GPIOC_ODR &= ~(1 << 5);  // Green OFF
}