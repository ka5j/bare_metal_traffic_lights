/**
* STM32 Traffic Light System with Pedestrian Walk Feature
*
* LEDs on GPIOC:
*   - RED     : PC8
*   - YELLOW  : PC6
*   - GREEN   : PC5
*   - WHITE   : PC7 (Pedestrian)
*
* Button Input:
*   - PA9: Toggles pedestrian request flag
*
* Light Sequence:
*   RED (~3s), RED+YELLOW (~1s), GREEN (~3s), YELLOW (~1s), Repeat
*   If pedestrian button is pressed, WHITE LED is enabled during RED state.
*/

#include <stdint.h>

/* ----------------------------- Global State ------------------------------ */
 
volatile uint8_t light_state = 0;   // 1=RED, 2=RED+YELLOW, 3=GREEN, 4=YELLOW
volatile uint8_t walk = 0;          // Pedestrian walk flag, toggled by button
 
/* ----------------------------- State Defines ----------------------------- */
 
#define RED         1
#define RED_YELLOW  2
#define GREEN       3
#define YELLOW      4
 
/* --------------------------- Timing Constants ---------------------------- */
 
#define LIGHT       3000    // ~3 seconds (in ms)
#define TRANSITION  1000    // ~1 second (in ms)
#define PRESCALER   15999   // Generate 1ms ticks
 
/* ------------------------ Peripheral Base Addresses ---------------------- */
 
#define RCC_BASE       0x40023800
#define GPIOC_BASE     0x40020800
#define GPIOA_BASE     0x40020000
#define SYSCFG_BASE    0x40013800
#define NVIC_BASE      0xE000E100
#define EXTI_BASE      0x40013C00
#define TIM2_BASE      0x40000000
 
/* -------------------------- TIM2 Register Map ---------------------------- */
 
#define TIM2_CR1       (*(volatile uint32_t*)(TIM2_BASE + 0x00))
#define TIM2_DIER      (*(volatile uint32_t*)(TIM2_BASE + 0x0C))
#define TIM2_SR        (*(volatile uint32_t*)(TIM2_BASE + 0x10))
#define TIM2_CNT       (*(volatile uint32_t*)(TIM2_BASE + 0x24))
#define TIM2_PSC       (*(volatile uint32_t*)(TIM2_BASE + 0x28))
#define TIM2_ARR       (*(volatile uint32_t*)(TIM2_BASE + 0x2C))
 
/* ---------------------------- RCC Registers ------------------------------ */
 
#define RCC_AHB1ENR    (*(volatile uint32_t*)(RCC_BASE + 0x30))
#define RCC_APB1ENR    (*(volatile uint32_t*)(RCC_BASE + 0x40))
 
/* --------------------------- SYSCFG Registers ---------------------------- */
 
#define SYSCFG_EXTICR3 (*(volatile uint32_t*)(SYSCFG_BASE + 0x10))
 
/* ---------------------------- EXTI Registers ----------------------------- */
 
#define EXTI_IMR       (*(volatile uint32_t*)(EXTI_BASE + 0x00))
#define EXTI_EMR       (*(volatile uint32_t*)(EXTI_BASE + 0x04))
#define EXTI_RTSR      (*(volatile uint32_t*)(EXTI_BASE + 0x08))
#define EXTI_FTSR      (*(volatile uint32_t*)(EXTI_BASE + 0x0C))
#define EXTI_PR        (*(volatile uint32_t*)(EXTI_BASE + 0x14))
 
/* ---------------------------- NVIC Registers ----------------------------- */
 
#define NVIC_ISER0     (*(volatile uint32_t*)(NVIC_BASE + 0x00))
 
/* --------------------------- GPIOC Registers ----------------------------- */
 
#define GPIOC_MODER    (*(volatile uint32_t*)(GPIOC_BASE + 0x00))
#define GPIOC_OTYPER   (*(volatile uint32_t*)(GPIOC_BASE + 0x04))
#define GPIOC_OSPEEDR  (*(volatile uint32_t*)(GPIOC_BASE + 0x08))
#define GPIOC_PUPDR    (*(volatile uint32_t*)(GPIOC_BASE + 0x0C))
#define GPIOC_ODR      (*(volatile uint32_t*)(GPIOC_BASE + 0x14))
 
/* --------------------------- GPIOA Registers ----------------------------- */
 
#define GPIOA_MODER    (*(volatile uint32_t*)(GPIOA_BASE + 0x00))
#define GPIOA_PUPDR    (*(volatile uint32_t*)(GPIOA_BASE + 0x0C))
#define GPIOA_IDR      (*(volatile uint32_t*)(GPIOA_BASE + 0x14))
 
/* ------------------------- Function Prototypes --------------------------- */
 
void GPIOC_Init(void);
void GPIOA_Init(void);
void EXTI9_Init(void);
void TIM2_Init(void);
void TIM2_Set_Counter(uint32_t time_in_ms);
 
void set_RED(void);
void set_RED_YELLOW(void);
void set_GREEN(void);
void set_YELLOW(void);
void set_RED_WHITE(void);
 
/* ---------------------------- Main Function ------------------------------ */
 
int main(void) {
	GPIOC_Init();       // Configure LEDs (PC5–PC8)
	GPIOA_Init();       // Configure button (PA9)
	EXTI9_Init();       // Configure EXTI for PA9
	TIM2_Init();        // Configure timer for state transitions
 
	while (1);          // Main loop is empty; logic is handled via interrupts
}
 
/* -------------------------- GPIO Configuration --------------------------- */
 
/**
 * Initialize GPIOC pins 5–8 as push-pull outputs
 */
void GPIOC_Init(void) {
	RCC_AHB1ENR |= (1 << 2);  // Enable clock for GPIOC
 
	// Set MODER to output mode (01)
	GPIOC_MODER &= ~((3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)) | (3 << (8 * 2)));
	GPIOC_MODER |=  ((1 << (5 * 2)) | (1 << (6 * 2)) | (1 << (7 * 2)) | (1 << (8 * 2)));
 
	// Set push-pull type
	GPIOC_OTYPER &= ~((1 << 5) | (1 << 6) | (1 << 7) | (1 << 8));
 
	// Set low speed
	GPIOC_OSPEEDR &= ~((3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)) | (3 << (8 * 2)));
 
	// Disable pull-up/down
	GPIOC_PUPDR &= ~((3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)) | (3 << (8 * 2)));
}
 
/**
 * Initialize PA9 as input with pull-down
 */
void GPIOA_Init(void) {
	RCC_AHB1ENR |= (1 << 0);      // Enable clock for GPIOA
 
	GPIOA_MODER &= ~(3 << (9 * 2));   // Input mode
	GPIOA_PUPDR &= ~(3 << (9 * 2));   // Clear PUPD
	GPIOA_PUPDR |=  (2 << (9 * 2));   // Set pull-down
}
 
/* --------------------------- EXTI Configuration -------------------------- */
 
/**
 * Configure EXTI Line 9 for PA9 (button)
 */
void EXTI9_Init(void) {
	NVIC_ISER0 |= (1 << 23);             // Enable EXTI9_5 interrupt
 
	SYSCFG_EXTICR3 &= ~(0xF << 4);       // Map EXTI9 to PA9 (Port A)
 
	EXTI_IMR  |= (1 << 9);               // Unmask interrupt
	EXTI_RTSR |= (1 << 9);               // Trigger on rising edge
	EXTI_FTSR &= ~(1 << 9);              // No falling edge
	EXTI_EMR  &= ~(1 << 9);              // Disable event
}
 
/* --------------------------- TIM2 Configuration -------------------------- */
 
/**
* Initialize TIM2 for time-based state transitions
*/
void TIM2_Init(void) {
	RCC_APB1ENR |= (1 << 0);     // Enable TIM2 clock
	NVIC_ISER0 |= (1 << 28);     // Enable TIM2 interrupt
 
	TIM2_PSC = PRESCALER;        // Prescaler for 1ms tick (assuming 16 MHz)
	TIM2_Set_Counter(TRANSITION);
 
	TIM2_DIER |= (1 << 0);       // Enable update interrupt
	TIM2_CR1  |= (1 << 0);       // Start TIM2
}
 
/**
 * Set TIM2 auto-reload value for countdown (in ms)
 */
void TIM2_Set_Counter(uint32_t time_in_ms) {
	TIM2_ARR = time_in_ms - 1;
}
 
/* ------------------------- Interrupt Handlers ---------------------------- */
 
/**
 * Handle button press on PA9 (EXTI9_5)
 */
void EXTI9_5_IRQHandler(void) {
	EXTI_PR |= (1 << 9);     // Clear pending interrupt
	walk = 1;                // Enable pedestrian flag
}
 
/**
* Handle TIM2 update interrupt for light transitions
*/
void TIM2_IRQHandler(void) {
	TIM2_SR &= ~(1 << 0);                // Clear update flag
	light_state = (light_state % 4) + 1; // Cycle states 1→4
 
	switch (light_state) {
		case RED:
			set_RED();
			if (walk) {
				set_RED_WHITE();        // Enable pedestrian light
				walk = 0;               // Reset walk flag
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
 
/* ----------------------------- LED Control ------------------------------- */
 
void set_RED(void) {
	GPIOC_ODR |=  (1 << 8);    // RED ON
	GPIOC_ODR &= ~(1 << 6);    // YELLOW OFF
	GPIOC_ODR &= ~(1 << 5);    // GREEN OFF
	GPIOC_ODR &= ~(1 << 7);    // WHITE OFF
}
 
void set_RED_YELLOW(void) {
	GPIOC_ODR |=  (1 << 8);    // RED ON
	GPIOC_ODR |=  (1 << 6);    // YELLOW ON
	GPIOC_ODR &= ~(1 << 5);    // GREEN OFF
	GPIOC_ODR &= ~(1 << 7);    // WHITE OFF
}
 
void set_GREEN(void) {
	GPIOC_ODR |=  (1 << 5);    // GREEN ON
	GPIOC_ODR &= ~(1 << 6);    // YELLOW OFF
	GPIOC_ODR &= ~(1 << 8);    // RED OFF
	GPIOC_ODR &= ~(1 << 7);    // WHITE OFF
}
 
void set_YELLOW(void) {
	GPIOC_ODR |=  (1 << 6);    // YELLOW ON
	GPIOC_ODR &= ~(1 << 5);    // GREEN OFF
	GPIOC_ODR &= ~(1 << 8);    // RED OFF
	GPIOC_ODR &= ~(1 << 7);    // WHITE OFF
}
 
void set_RED_WHITE(void) {
	GPIOC_ODR |=  (1 << 8);    // RED ON
	GPIOC_ODR |=  (1 << 7);    // WHITE ON (pedestrian)
	GPIOC_ODR &= ~(1 << 6);    // YELLOW OFF
	GPIOC_ODR &= ~(1 << 5);    // GREEN OFF
}
 