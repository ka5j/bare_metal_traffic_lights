# STM32F446RE Traffic Light System with Pedestrian Control
This bare-metal embedded systems project implements a simple **traffic light controller** with a **pedestrian crossing feature**, written in **Embedded C** for the **STM32F446RE Nucleo Board**.

The system cycles through standard traffic light states (RED, RED+YELLOW, GREEN, YELLOW) using **SysTick timer interrupts**, and responds to pedestrian crossing requests via a **button on PA9**, triggering a **white pedestrian LED (PC7)** during the red phase.
---

## Hardware Connections

| Component              | STM32 Pin | Color     |
|------------------------|-----------|-----------|
| Traffic Light - RED    | PC8       | Red     |
| Traffic Light - YELLOW | PC6       | Yellow  |
| Traffic Light - GREEN  | PC5       | Green   |
| Pedestrian Light       | PC7       | White    |
| Button (Pedestrian)    | PA9       | Button   |
---

## System Behavior
### State Sequence:
1. **RED** (~1s)
2. **RED + YELLOW** (~0.25s)
3. **GREEN** (~1s)
4. **YELLOW** (~0.25s)

- During **RED**, if a pedestrian request is made (button press), the **white pedestrian LED** (PC7) will briefly activate alongside the red LED.
- The system uses **SysTick timer** for timekeeping and **EXTI9 interrupt** for detecting button presses on **PA9**.
---

## Features
- Precise state timing via **SysTick interrupts**
- Real-time **external interrupt handling** on **PA9**
- Pedestrian "WALK" mode with **white LED** trigger
- Clean modular register-level code (no HAL or CMSIS)
- Uses **direct memory-mapped I/O** for full control
---

## Project Architecture
- `main()` initializes all peripherals and enters an infinite loop.
- **SysTick_Handler** cycles through traffic light states.
- **EXTI9_5_IRQHandler** toggles the pedestrian request flag.
- Modular functions like `set_RED()`, `set_GREEN()` etc. handle LED logic.
---

## How to Run
1. Connect your STM32F446RE board.
2. Flash the code using STM32CubeIDE or OpenOCD + arm-none-eabi toolchain.
3. Connect LEDs and button to the appropriate GPIO pins.
4. Press the **pedestrian button (PA9)** during the RED state to trigger pedestrian crossing.
---

## File Breakdown
- `Core/Src/main.c`: All initialization, ISR logic, GPIO configuration, and timing control
- Uses **bare-metal C**, **no HAL**, **no RTOS**, **no libraries**
---

## What You Learn
✅ SysTick timer  
✅ EXTI peripheral & interrupts  
✅ GPIO register programming  
✅ LED control and button handling  
✅ Embedded systems state machines
✅ Real-time control  
✅ Bare-metal register manipulation
✅ Embedded interrupt-driven design
---

## Future Improvements
- Debounce the pedestrian button using timers or delay
- Add 7-segment countdown display for red/green durations
- Use multiple pedestrian request buttons
- Add audible pedestrian signal (buzzer on a timer)
---

## References
- STM32F446RE Reference Manual (RM0390)
- STM32F446RE Datasheet
- ARM Cortex-M4 Technical Reference
- Register-level programming practices
---