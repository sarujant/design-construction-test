#include "stm32f407xx.h"
#include "Board_LED.h"
#include "PB_LCD_Drivers.h"
#include <stdio.h>
#include <stdlib.h>

void configPA0(void);
int checkButton(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);

void initialise_adc(void);
void dc_voltage(void);
void ac_voltage(void);
void current(void);
void resistance(void);

int menu_select_counter;
int buttonState;

int startup_screen_status = 1;

int sleep_tick_count = 0;
int cache_sleep_tick_count = 0;
const int STARTUP_SCREEN_DELAY = 5;

const int BTN_DEBOUNCER_DELAY = 3;
int btn_tick_count = 0;
int cache_btn_tick_count = 0;

uint32_t adc1_conv;
uint32_t adc2_conv;

void delay(int time) {
	cache_sleep_tick_count = sleep_tick_count;
	while (cache_sleep_tick_count+time > sleep_tick_count);
}


int main() {
	menu_select_counter = 0;
	buttonState = 0;
	adc1_conv = 0;
	adc2_conv = 0;
	
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/2);
		
	configPA0();
	NVIC_EnableIRQ(EXTI0_IRQn);
	
	
	PB_LCD_Init();
	initialise_adc();
	
	while(1) {
		if (startup_screen_status == 1) {
			PB_LCD_GoToXY (0, 0);
			PB_LCD_Clear();
			PB_LCD_WriteString("---Multimeter---", 50);
			PB_LCD_GoToXY (0, 1);
			PB_LCD_WriteString("  DCT, T-1 G-A", 50);
			
			delay(STARTUP_SCREEN_DELAY);
			
			PB_LCD_GoToXY (0, 0);
			PB_LCD_Clear();
			PB_LCD_WriteString("---Multimeter---", 50);
			PB_LCD_GoToXY (0, 1);
			PB_LCD_WriteString("push btn to strt", 50);
		
			delay(STARTUP_SCREEN_DELAY);
		}
	}
}

void menu() {
			switch(menu_select_counter) {
			case 1 :
				PB_LCD_Clear();
				//char dcVoltage[50];
				//int dcVLen = snprintf(dcVoltage, 50, "DC Voltage");
				PB_LCD_GoToXY (0, 0);
				PB_LCD_WriteString("---DC Voltage---", 50);
				
				//dc_voltage();
				break;
			case 2 :
				PB_LCD_Clear();
				//char acVoltage[50];
				//int acVLen = snprintf(acVoltage, 50, "AC Voltage");
				PB_LCD_GoToXY (0, 0);
				PB_LCD_WriteString("---AC Voltage---", 50);
				//ac_voltage();
				break;
			case 3 :
				PB_LCD_Clear();
				//char current[50];
				//int currLen = snprintf(current, 50, "Current");
				PB_LCD_GoToXY (0, 0);
				PB_LCD_WriteString("----Current ----", 50);
				//current();
				break;
			case 4 :
				menu_select_counter = 0;
				PB_LCD_Clear();
				//char resistance[50];
				//int resLen = snprintf(resistance, 50, "Resistance");
				PB_LCD_GoToXY (0, 0);
				PB_LCD_WriteString("---Resistance---", 50);				
				//resistance();
				break;
			default:
				PB_LCD_Clear();
				//char error[50];
				//int errorLen = snprintf(error, 50, "Error lol");
				PB_LCD_GoToXY (0, 0);
				PB_LCD_WriteString("---- ERROR! ----", 50);
				PB_LCD_GoToXY (0, 1);
				PB_LCD_WriteString(" push btn again ", 50);
		}
}

void configPA0(void) {
	// Configure PA0/button? as a input.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOE->MODER &= ~GPIO_MODER_MODE0_Msk;
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT0_Msk;
	GPIOE->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_Msk;
	GPIOE->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
	GPIOE->PUPDR |= 1 << GPIO_PUPDR_PUPD0_Pos;
		
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enables System config
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // PA0 -> config interrupt to come from PA0

	EXTI->IMR |= EXTI_IMR_MR0;
	EXTI->FTSR |= EXTI_FTSR_TR0;
	
}

int checkButton(void) {
	if ((GPIOA->IDR & 0x00000001) == 0x00000001){
		return 1;
	} else {
		return 0;
	}	
}	

void SysTick_Handler(void){	
	if(buttonState != 0) {
		GPIOA->BSRR |= 0x00000001;
	} else {
		GPIOA->BSRR |= 0x00010000;
	}
	sleep_tick_count++;
	btn_tick_count++;
}

void EXTI0_IRQHandler(void) {
	EXTI->PR = EXTI_PR_PR0;
	if (checkButton()) {
		buttonState = 1;
		startup_screen_status = 0;
		int current_btn_tick_count = btn_tick_count;
		cache_btn_tick_count = current_btn_tick_count;
		int ticks_elapsed = current_btn_tick_count - cache_btn_tick_count;
		if (ticks_elapsed >= BTN_DEBOUNCER_DELAY) {
			menu_select_counter++;
			if(menu_select_counter > 4) {
				menu_select_counter = 0;	
			}
			menu();
		}
	}
	if (checkButton()) {
		
	}
}

void dc_voltage(void) {
	
}

void ac_voltage(void) {
	ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
	while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){} 
	adc1_conv = ADC1->DR;
	ADC2->CR2 |= ADC_CR2_SWSTART_Msk; 
	while(ADC2->SR != (ADC2->SR | ADC_SR_EOC_Msk)){} 
	adc1_conv = ADC2->DR;	
}

void current(void) {
	ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
	while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){} 
	adc1_conv = ADC1->DR;
}

void resistance(void) {
	ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
	while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){} 
	adc1_conv = ADC1->DR;
}
void initialise_adc(void) {
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
	
	ADC1->SQR3 &= 0xFFFFFFE0;
	ADC2->SQR3 &= 0xFFFFFFE0;
	
	ADC1->SQR3 |= 0x00000001;
	ADC2->SQR3 |= 0x00000001;
	
	ADC1->CR2 |= ADC_CR2_ADON_Msk;  
	ADC1->CR1 |= ADC_CR1_DISCEN_Msk;
	ADC1->CR2 |= ADC_CR2_EOCS_Msk;
	
	ADC2->CR2 |= ADC_CR2_ADON_Msk; 
	ADC2->CR1 |= ADC_CR1_DISCEN_Msk;
	ADC2->CR2 |= ADC_CR2_EOCS_Msk;
}
