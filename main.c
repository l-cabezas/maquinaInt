#include "MKL46Z4.h"

// LED (RG)
// LED_GREEN = PTD5 (pin 98)
// LED_RED = PTE29 (pin 26)

// SWICHES
// LEFT (SW1) = PTC3 (pin 73)
// RIGHT (SW2) = PTC12 (pin 88)

unsigned int state = 0;
unsigned int was_sw1_pressed=0;
unsigned int was_sw2_pressed=0;

void delay(void)
{
  volatile int i;

  for (i = 0; i < 1000000; i++);
}

// LEFT_SWITCH (SW1) = PTC3
void sw1_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3);
  
  
}

// RIGHT_SWITCH (SW2) = PTC12
void sw2_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 12);
}

int sw1_check()
{
  return( !(GPIOC->PDIR & (1 << 3)) );
}

int sw2_check()
{
  return( !(GPIOC->PDIR & (1 << 12)) );
}

// LEFT_SWITCH (SW1) = PTC3
// RIGHT_SWITCH (SW2) = PTC12
void sws_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);

  PORTC->PCR[3] |= PORT_PCR_IRQC(0xA); // IRQ on falling edge
  PORTC->PCR[12] |= PORT_PCR_IRQC(0xA); // IRQ on falling edge
  
  // IRQ#31: Pin detect for PORTS C & D
  NVIC_SetPriority(31, 0); // Max priority for IRQ#31
  NVIC_EnableIRQ(31);      // Enable IRQ#31
}

// LED_GREEN = PTD5
void led_green_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOD->PSOR = (1 << 5);
}

void led_green_toggle()
{
  GPIOD->PTOR = (1 << 5);
}

// LED_RED = PTE29
void led_red_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR = (1 << 29);
}

void led_red_toggle(void)
{
  GPIOE->PTOR = (1 << 29);
}

// LED_RED = PTE29
// LED_GREEN = PTD5
void leds_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOE->PDDR |= (1 << 29);
  // both LEDS off after init
  GPIOD->PSOR = (1 << 5);
  GPIOE->PSOR = (1 << 29);
}

void PORTDIntHandler(void)
{
  PORTC->ISFR = 0xFFFFFFFF; // Clear IRQ
  
    if(sw1_check()){
    was_sw1_pressed=1;
    }
    while(sw1_check());
    if(sw2_check()){
    was_sw2_pressed=1;
    }
    while(sw2_check());
  
  
  
}


int main(void)
{
  // State 0: Both doors locked
  // State 1: Door1 unlocked, Door2 locked
  // State 2: Door1 locked, Door2 unlocked
  // State 3: Both doors unlocked
  

  leds_init();
  sws_init();

  led_green_toggle(); // State 0: LEDgreen ON

  while (1) {
    if (was_sw1_pressed) {
      switch(state) {
      case 0:
        state = 1; // State 0 => State 1
        led_green_toggle(); // LEDgreen OFF
        led_red_toggle();   // LEDgreen ON
        break;
      case 1:
        state = 0; // State 1 => State 0
        led_red_toggle();   // LEDgreen OFF
        led_green_toggle(); // LEDgreen ON
        break;
      case 2:
        state = 3; // State 2 => State 3
        break;
      case 3:
        state = 2; // State 3 => State 2
        break;
      
      }
      was_sw1_pressed = 0; // Improvement by Miguel Blanco Godón
    }

    if (was_sw2_pressed) {
      switch(state) {
      case 0:
        state = 2; // State 0 => State 2
        led_green_toggle(); // LEDgreen OFF
        led_red_toggle();   // LEDgreen ON
        break;
      case 1:
        state = 3; // State 1 => State 3
        break;
      case 2:
        state = 0; // State 2 => State 0
        led_red_toggle();   // LEDgreen OFF
        led_green_toggle(); // LEDgreen ON
        break;
      case 3:
        state = 1; // State 3 => State 1
        break;
      }
      was_sw2_pressed = 0; // Improvement by Miguel Blanco Godón
    }
  }
  

  return 0;
}
