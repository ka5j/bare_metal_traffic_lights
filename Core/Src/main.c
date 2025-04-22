/*
 * STM32 Traffic Light System with Pedestrian Walk Feature
 *
 * RED LED: PC8
 * YELLOW LED: PC6
 * GREEN LED: PC5
 * PEDESTRIAN (WHITE) LED: PC7
 * BUTTON: PA9
 * 7-Segment Display:
 * - Pin 1 (E) -> PB8 -
 * - Pin 2 (D) -> PB9
 * - Pin 4 (C) -> PB6
 * - Pin 6 (B) -> PB4
 * - Pin 7 (A) -> PB5
 * - Pin 9 (F) -> PB3
 * - Pin 10 (G) -> PB12
 * - Pin 3, 8 -> GND
 *
 * State Sequence:
 * RED (~3s), RED+YELLOW (~1s), GREEN (~3s), YELLOW (~1s), repeat
 * Pedestrian request via PA9 toggles 'walk' flag; activates WHITE LED during RED state.
 */
/* Seven Segment Display Numbers
 *
 * | x | x | x | x | x | x | x |
 * MSB LSB
 * a b c d e f g
 * - 0 -> 0b1111110
 * - 1 -> 0b0110000
 * - 2 -> 0b1101101
 * - 3 -> 0b1111001
 * - 4 -> 0b0110011
 * - 5 -> 0b1011011
 * - 6 -> 0b0011111
 * - 7 -> 0b1110000
 * - 8 -> 0b1111111
 * - 9 -> 0b1110011 */

#include <stdint.h>

/* Global state variables */
volatile uint8_t light_state = 0; // Tracks current light phase (1–4)
volatile uint8_t walk = 0; // Pedestrian walk flag, toggled by button press
volatile uint8_t time = 9;

/* State identifiers */
#define RED 1
#define RED_YELLOW 2
#define GREEN 3
#define YELLOW 4

/* Timing constants (based on SysTick clock ticks) */
#define LIGHT 8000 // ~3s
#define TRANSITION 2000 // ~1s

/* Base addresses for peripherals */
#define RCC_BASE 0x40023800
#define GPIOC_BASE 0x40020800
#define GPIOA_BASE 0x40020000
#define GPIOB_BASE 0x40020400
#define SYSCFG_BASE 0x40013800
#define NVIC_BASE 0xE000E100
#define EXTI_BASE 0x40013C00
#define TIM2_BASE 0x40000000
#define TIM3_BASE 0x40000400

/* TIM2 Registers */
#define TIM2_CR1 (*(volatile uint32_t*)(TIM2_BASE + 0x00))
#define TIM2_CNT (*(volatile uint32_t*)(TIM2_BASE + 0x24))
#define TIM2_PSC (*(volatile uint32_t*)(TIM2_BASE + 0x28))
#define TIM2_ARR (*(volatile uint32_t*)(TIM2_BASE + 0x2C))
#define TIM2_DIER (*(volatile uint32_t*)(TIM2_BASE + 0x0C))
#define TIM2_SR (*(volatile uint32_t*)(TIM2_BASE + 0x10))

/* TIM3 Registers */
#define TIM3_CR1 (*(volatile uint32_t*)(TIM3_BASE + 0x00))
#define TIM3_CNT (*(volatile uint32_t*)(TIM3_BASE + 0x24))
#define TIM3_PSC (*(volatile uint32_t*)(TIM3_BASE + 0x28))
#define TIM3_ARR (*(volatile uint32_t*)(TIM3_BASE + 0x2C))
#define TIM3_DIER (*(volatile uint32_t*)(TIM3_BASE + 0x0C))
#define TIM3_SR (*(volatile uint32_t*)(TIM3_BASE + 0x10))

/* RCC register */
#define RCC_AHB1ENR (*(volatile uint32_t*)(RCC_BASE + 0x30))
#define RCC_APB1ENR (*(volatile uint32_t*)(RCC_BASE + 0x40))

/* SYSCFG register for EXTI configuration */
#define SYSCFG_EXTICR3 (*(volatile uint32_t*)(SYSCFG_BASE + 0x10))

/* EXTI registers for interrupt setup */
#define EXTI_IMR (*(volatile uint32_t*)(EXTI_BASE + 0x00))
#define EXTI_EMR (*(volatile uint32_t*)(EXTI_BASE + 0x04))
#define EXTI_RTSR (*(volatile uint32_t*)(EXTI_BASE + 0x08))
#define EXTI_FTSR (*(volatile uint32_t*)(EXTI_BASE + 0x0C))
#define EXTI_PR (*(volatile uint32_t*)(EXTI_BASE + 0x14))

/* NVIC register for enabling IRQs */
#define NVIC_ISER0 (*(volatile uint32_t*)(NVIC_BASE + 0x00))

/* GPIOC registers */
#define GPIOC_MODER (*(volatile uint32_t*)(GPIOC_BASE + 0x00))
#define GPIOC_OTYPER (*(volatile uint32_t*)(GPIOC_BASE + 0x04))
#define GPIOC_OSPEEDR (*(volatile uint32_t*)(GPIOC_BASE + 0x08))
#define GPIOC_PUPDR (*(volatile uint32_t*)(GPIOC_BASE + 0x0C))
#define GPIOC_ODR (*(volatile uint32_t*)(GPIOC_BASE + 0x14))

/* GPIOB registers */
#define GPIOB_MODER (*(volatile uint32_t*)(GPIOB_BASE + 0x00))
#define GPIOB_OTYPER (*(volatile uint32_t*)(GPIOB_BASE + 0x04))
#define GPIOB_OSPEEDR (*(volatile uint32_t*)(GPIOB_BASE + 0x08))
#define GPIOB_PUPDR (*(volatile uint32_t*)(GPIOB_BASE + 0x0C))
#define GPIOB_ODR (*(volatile uint32_t*)(GPIOB_BASE + 0x14))

/* GPIOA registers */
#define GPIOA_MODER (*(volatile uint32_t*)(GPIOA_BASE + 0x00))
#define GPIOA_PUPDR (*(volatile uint32_t*)(GPIOA_BASE + 0x0C))
#define GPIOA_IDR (*(volatile uint32_t*)(GPIOA_BASE + 0x14))

/* Function prototypes */
void GPIOC_Init(void);
void GPIOA_Init(void);
void GPIOB_Init(void);
void set_Number(uint8_t);
void TIM2_Set_Counter(uint32_t);
void TIM2_Init(void);
void TIM3_Init(void);
void TIM3_Set_Counter(uint32_t);
void EXTI9_Init(void);
void set_RED(void);
void set_RED_YELLOW(void);
void set_GREEN(void);
void set_YELLOW(void);
void set_RED_WHITE(void);
void set_Zero(void);
void set_One(void);
void set_Two(void);
void set_Three(void);
void set_Four(void);
void set_Five(void);
void set_Six(void);
void set_Seven(void);
void set_Eight(void);

/* Main function */
int main(void) {
	GPIOC_Init(); // Configure PC5, PC6, PC7, PC8 as outputs
	GPIOA_Init(); // Configure PA9 as input with pull-down
	GPIOB_Init();
	TIM2_Init(); // Setup timer for state transitions
	EXTI9_Init(); // Enable external interrupt for PA9 (button)
	set_Eight();
	while (1); // Main loop does nothing; logic runs in ISRs
}

/* Configure PC5-PC8 as push-pull outputs */
void GPIOC_Init(void) {
	RCC_AHB1ENR |= (1 << 2); // Enable GPIOC clock
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
	RCC_AHB1ENR |= (1 << 0); // Enable GPIOA clock
	GPIOA_MODER &= ~(3 << (9 * 2)); // Set PA9 as input
	GPIOA_PUPDR &= ~(3 << (9 * 2));
	GPIOA_PUPDR |= (2 << (9 * 2)); // Enable pull-down
}

void GPIOB_Init(void){
	RCC_AHB1ENR |= (1 << 1); // Enable GPIOB clock
	GPIOB_MODER &= ~(3 << (3 * 2));
	GPIOB_MODER &= ~(3 << (4 * 2));
	GPIOB_MODER &= ~(3 << (5 * 2));
	GPIOB_MODER &= ~(3 << (6 * 2));
	GPIOB_MODER &= ~(3 << (8 * 2));
	GPIOB_MODER &= ~(3 << (9 * 2));
	GPIOB_MODER &= ~(3 << (12 * 2));
	GPIOB_MODER |= (1 << (3 * 2));
	GPIOB_MODER |= (1 << (4 * 2));
	GPIOB_MODER |= (1 << (5 * 2));
	GPIOB_MODER |= (1 << (6 * 2));
	GPIOB_MODER |= (1 << (8 * 2));
	GPIOB_MODER |= (1 << (9 * 2));
	GPIOB_MODER |= (1 << (12 * 2));
	GPIOB_OTYPER &= ~(3 << (3));
	GPIOB_OTYPER &= ~(3 << (4));
	GPIOB_OTYPER &= ~(3 << (5));
	GPIOB_OTYPER &= ~(3 << (6));
	GPIOB_OTYPER &= ~(3 << (8));
	GPIOB_OTYPER &= ~(3 << (9));
	GPIOB_OTYPER &= ~(3 << (12));
	GPIOB_OSPEEDR &= ~(3 << (3 * 2));
	GPIOB_OSPEEDR &= ~(3 << (4 * 2));
	GPIOB_OSPEEDR &= ~(3 << (5 * 2));
	GPIOB_OSPEEDR &= ~(3 << (6 * 2));
	GPIOB_OSPEEDR &= ~(3 << (8 * 2));
	GPIOB_OSPEEDR &= ~(3 << (9 * 2));
	GPIOB_OSPEEDR &= ~(3 << (12 * 2));
	GPIOB_PUPDR &= ~(3 << (3 * 2));
	GPIOB_PUPDR &= ~(3 << (4 * 2));
	GPIOB_PUPDR &= ~(3 << (5 * 2));
	GPIOB_PUPDR &= ~(3 << (6 * 2));
	GPIOB_PUPDR &= ~(3 << (8 * 2));
	GPIOB_PUPDR &= ~(3 << (9 * 2));
	GPIOB_PUPDR &= ~(3 << (12 * 2));
}

void set_Number(uint8_t number){
	switch(number){
		case 0:
			set_Zero();
			break;
		case 1:
			set_One();
			break;
		case 2:
			set_Two();
			break;
		case 3:
			set_Three();
			break;
		case 4:
			set_Four();
			break;
		case 5:
			set_Five();
			break;
		case 6:
			set_Six();
			break;
		case 7:
			set_Seven();
			break;
		case 8:
			set_Eight();
			break;
		default:
			break;
	}
}

void set_Zero(void){
	GPIOB_ODR |= (1 << 5);
	GPIOB_ODR |= (1 << 4);
	GPIOB_ODR |= (1 << 6);
	GPIOB_ODR |= (1 << 9);
	GPIOB_ODR |= (1 << 8);
	GPIOB_ODR |= (1 << 3);
	GPIOB_ODR &= ~(1 << 12);
}

void set_One(void){
	GPIOB_ODR &= ~(1 << 5);
	GPIOB_ODR |= (1 << 4);
	GPIOB_ODR |= (1 << 6);
	GPIOB_ODR &= ~(1 << 9);
	GPIOB_ODR &= ~(1 << 8);
	GPIOB_ODR &= ~(1 << 3);
	GPIOB_ODR &= ~(1 << 12);
}

void set_Two(void){
	GPIOB_ODR |= (1 << 5);
	GPIOB_ODR |= (1 << 4);
	GPIOB_ODR &= ~(1 << 6);
	GPIOB_ODR |= (1 << 9);
	GPIOB_ODR |= (1 << 8);
	GPIOB_ODR &= ~(1 << 3);
	GPIOB_ODR |= (1 << 12);
}

void set_Three(void){
	GPIOB_ODR |= (1 << 5);
	GPIOB_ODR |= (1 << 4);
	GPIOB_ODR |= (1 << 6);
	GPIOB_ODR |= (1 << 9);
	GPIOB_ODR &= ~(1 << 8);
	GPIOB_ODR &= ~(1 << 3);
	GPIOB_ODR |= (1 << 12);
}

void set_Four(void){
	GPIOB_ODR &= ~(1 << 5);
	GPIOB_ODR |= (1 << 4);
	GPIOB_ODR |= (1 << 6);
	GPIOB_ODR &= ~(1 << 9);
	GPIOB_ODR &= ~(1 << 8);
	GPIOB_ODR |= (1 << 3);
	GPIOB_ODR |= (1 << 12);
}

void set_Five(void){
	GPIOB_ODR |= (1 << 5);
	GPIOB_ODR &= ~(1 << 4);
	GPIOB_ODR |= (1 << 6);
	GPIOB_ODR |= (1 << 9);
	GPIOB_ODR &= ~(1 << 8);
	GPIOB_ODR |= (1 << 3);
	GPIOB_ODR |= (1 << 12);
}

void set_Six(void){
	GPIOB_ODR &= ~(1 << 5);
	GPIOB_ODR &= ~(1 << 4);
	GPIOB_ODR |= (1 << 6);
	GPIOB_ODR |= (1 << 9);
	GPIOB_ODR |= (1 << 8);
	GPIOB_ODR |= (1 << 3);
	GPIOB_ODR |= (1 << 12);
}

void set_Seven(void){
	GPIOB_ODR |= (1 << 5);
	GPIOB_ODR |= (1 << 4);
	GPIOB_ODR |= (1 << 6);
	GPIOB_ODR &= ~(1 << 9);
	GPIOB_ODR &= ~(1 << 8);
	GPIOB_ODR &= ~(1 << 3);
	GPIOB_ODR &= ~(1 << 12);
}

void set_Eight(void){
	GPIOB_ODR |= (1 << 5);
	GPIOB_ODR |= (1 << 4);
	GPIOB_ODR |= (1 << 6);
	GPIOB_ODR |= (1 << 9);
	GPIOB_ODR |= (1 << 8);
	GPIOB_ODR |= (1 << 3);
	GPIOB_ODR |= (1 << 12);
}

void TIM2_Init(void){
	RCC_APB1ENR |= (1 << 0); // Enable TIM2 clock
	NVIC_ISER0 |= (1 << 28); // Enable TIM2 interrupt in NVIC
	TIM2_PSC = 15999; // 1ms tick
	TIM2_Set_Counter(TRANSITION);
	TIM2_DIER |= (1 << 0); // Enable update interrupt
	TIM2_CR1 |= (1 << 0); //Enable timer
}

void TIM2_Set_Counter(uint32_t time_in_ms) {
	TIM2_ARR = time_in_ms - 1;
}

void TIM2_IRQHandler(void){
	TIM2_SR &= ~(1 << 0);
	light_state = (light_state % 4) + 1;
	switch(light_state) {
		case RED:
			set_RED();
			if(walk) {
				TIM3_Init();
				set_RED_WHITE(); // Show pedestrian signal
				walk = 0; // Clear pedestrian request
			}
			TIM2_Set_Counter(LIGHT);
			break;
		case RED_YELLOW:
			set_RED_YELLOW();
			TIM2_Set_Counter(TRANSITION);
			break;
		case GREEN:
			set_GREEN();
			TIM2_Set_Counter(LIGHT);
			break;
		case YELLOW:
			set_YELLOW();
			TIM2_Set_Counter(TRANSITION);
			break;
	}
}

void TIM3_Init(void){
	RCC_APB1ENR |= (1 << 1); // Enable TIM3 clock
	NVIC_ISER0 |= (1 << 29); // Enable TIM2 interrupt in NVIC
	TIM3_PSC = 15999; // 1ms tick
	TIM3_Set_Counter(1000);
	TIM3_DIER |= (1 << 0); // Enable update interrupt
	TIM3_CR1 |= (1 << 0); //Enable timer
}

void TIM3_Set_Counter(uint32_t time_in_ms) {
	TIM3_ARR = time_in_ms - 1;
}

void TIM3_IRQHandler(void){
	TIM3_SR &= ~(1 << 0);
	time = (time - 1 + 9) % 9;
	set_Number(time);
	if(time == 0){
	TIM3_CR1 &= ~(1 << 0); //disable timer
	set_Eight();
	time = 8;
	}
}

/* Initialize external interrupt for PA9 (EXTI Line 9) */
void EXTI9_Init(void) {
	NVIC_ISER0 |= (1 << 23); // Enable EXTI9_5 interrupt in NVIC
	SYSCFG_EXTICR3 &= ~(0xF << 4); // Map EXTI9 to PA9
	EXTI_FTSR &= ~(1 << 9); // Disable falling edge trigger
	EXTI_EMR &= ~(1 << 9); // Disable event
	EXTI_RTSR |= (1 << 9); // Enable rising edge trigger
	EXTI_IMR |= (1 << 9); // Unmask interrupt for line 9
}

/* EXTI9_5 interrupt handler: toggle pedestrian walk request */
void EXTI9_5_IRQHandler(void) {
	EXTI_PR |= (1 << 9); // Clear interrupt pending flag
	walk ^= 1; // Toggle walk flag (may replace with walk = 1 for simplicity)
}

/* Activate RED light only */
void set_RED(void) {
	GPIOC_ODR |= (1 << 8); // Red ON
	GPIOC_ODR &= ~(1 << 6); // Yellow OFF
	GPIOC_ODR &= ~(1 << 5); // Green OFF
	GPIOC_ODR &= ~(1 << 7); // Pedestrian (White) OFF
}

/* Activate RED + YELLOW lights (before GREEN) */
void set_RED_YELLOW(void) {
	GPIOC_ODR |= (1 << 6); // Yellow ON
	GPIOC_ODR |= (1 << 8); // Red ON (still active)
	GPIOC_ODR &= ~(1 << 5); // Green OFF
	GPIOC_ODR &= ~(1 << 7); // Pedestrian (White) OFF
}

/* Activate GREEN light only */
void set_GREEN(void) {
	GPIOC_ODR |= (1 << 5); // Green ON
	GPIOC_ODR &= ~(1 << 8); // Red OFF
	GPIOC_ODR &= ~(1 << 6); // Yellow OFF
	GPIOC_ODR &= ~(1 << 7); // Pedestrian (White) OFF
}

/* Activate YELLOW light only (after GREEN) */
void set_YELLOW(void) {
	GPIOC_ODR |= (1 << 6); // Yellow ON
	GPIOC_ODR &= ~(1 << 5); // Green OFF
	GPIOC_ODR &= ~(1 << 8); // Red OFF
	GPIOC_ODR &= ~(1 << 7); // Pedestrian (White) OFF
}

/* Activate RED + WHITE (pedestrian walk signal) */
void set_RED_WHITE(void) {
	GPIOC_ODR |= (1 << 8); // Red ON
	GPIOC_ODR |= (1 << 7); // White ON (pedestrian)
	GPIOC_ODR &= ~(1 << 6); // Yellow OFF
	GPIOC_ODR &= ~(1 << 5); // Green OFF
}