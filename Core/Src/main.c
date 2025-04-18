/*This program simulates a traffic light intersection where
 * there is a red LED (PC8), yellow (PC6), and green LED (PC5) that is automated*/

/*RED LED: Pin PC8
 *YELLOW LED: Pin PC6
 *GREEN LED: Pin PC5 */

/*Output Structure:
 * 	RED: ~3s
 * 	RED + YELLOW: ~1s
 * 	GREEN: ~3s
 * 	YELLOW: ~1s
 * 	REPEAT */

#include <stdint.h>

uint8_t light_state = 0;

#define RED 1
#define RED_YELLOW 2
#define GREEN 3
#define YELLOW 4

#define HOLD 16000000
#define PAUSE 4000000

#define RCC_BASE 0x40023800
#define GPIOC_BASE 0x40020800
#define SYSTICK_BASE 0xE000E010

#define RCC_AHB1ENR (*(volatile uint32_t*)(RCC_BASE + 0x30))

#define GPIOC_MODER (*(volatile uint32_t*)(GPIOC_BASE + 0x00))
#define GPIOC_OTYPER (*(volatile uint32_t*)(GPIOC_BASE + 0x04))
#define GPIOC_OSPEEDR (*(volatile uint32_t*)(GPIOC_BASE + 0x08))
#define GPIOC_PUPDR (*(volatile uint32_t*)(GPIOC_BASE + 0x0C))
#define GPIOC_ODR (*(volatile uint32_t*)(GPIOC_BASE + 0x14))

#define SYSTICK_CTRL (*(volatile uint32_t*)(SYSTICK_BASE + 0x00))
#define SYSTICK_RELOAD (*(volatile uint32_t*)(SYSTICK_BASE + 0x04))
#define SYSTICK_CURRENT (*(volatile uint32_t*)(SYSTICK_BASE + 0x08))

void RCC_Init(void);
void GPIOC_Init(void);
void SysTick_Init(void);
void SysTick_Set_Counter(uint32_t);

int main(void){
	RCC_Init();

	GPIOC_Init();

	SysTick_Init();

	while(1);
}

void RCC_Init(void){
	RCC_AHB1ENR |= (1 << 2);
}

void GPIOC_Init(void){
	GPIOC_MODER &= ~(3 << (5 * 2));
	GPIOC_MODER &= ~(3 << (6 * 2));
	GPIOC_MODER &= ~(3 << (8 * 2));
	GPIOC_MODER |= (1 << (5 * 2));
	GPIOC_MODER |= (1 << (6 * 2));
	GPIOC_MODER |= (1 << (8 * 2));

	GPIOC_OTYPER &= ~(3 << 5);
	GPIOC_OTYPER &= ~(3 << 6);
	GPIOC_OTYPER &= ~(3 << 8);

	GPIOC_OSPEEDR &= ~(3 << (5 * 2));
	GPIOC_OSPEEDR &= ~(3 << (6 * 2));
	GPIOC_OSPEEDR &= ~(3 << (8 * 2));

	GPIOC_PUPDR &= ~(3 << (5 * 2));
	GPIOC_PUPDR &= ~(3 << (6 * 2));
	GPIOC_PUPDR &= ~(3 << (8 * 2));

}

void SysTick_Init(void){
	SysTick_Set_Counter(HOLD);
	SYSTICK_CTRL |= (1 << 2);
	SYSTICK_CTRL |= (1 << 1);
	SYSTICK_CTRL |= (1 << 0);
}

void SysTick_Set_Counter(uint32_t ticks){
	SYSTICK_RELOAD = ticks - 1;
	SYSTICK_CURRENT = 0;
}

void SysTick_Handler(void){
	light_state = (light_state % 4) + 1;

	switch(light_state){
		case RED:
			GPIOC_ODR |= (1 << 8);
			GPIOC_ODR &= ~(1 << 6);
			GPIOC_ODR &= ~(1 << 5);
			SysTick_Set_Counter(HOLD);
			break;
		case RED_YELLOW:
			GPIOC_ODR |= (1 << 6);
			GPIOC_ODR &= ~(1 << 5);
			SysTick_Set_Counter(PAUSE);
			break;
		case GREEN:
			GPIOC_ODR |= (1 << 5);
			GPIOC_ODR &= ~(1 << 8);
			GPIOC_ODR &= ~(1 << 6);
			SysTick_Set_Counter(HOLD);
			break;
		case YELLOW:
			GPIOC_ODR |= (1 << 6);
			GPIOC_ODR &= ~(1 << 8);
			GPIOC_ODR &= ~(1 << 5);
			SysTick_Set_Counter(PAUSE);
	}

}

