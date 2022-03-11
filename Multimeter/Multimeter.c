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
void timer_counter_setup(void);
void dac_out(void);

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
int timer_cnt = 0;

int ADC_dc_sample_count = 0;

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
	dac_out();
	
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
				dc_voltage();
				break;
			case 2 :
				PB_LCD_Clear();			
				PB_LCD_GoToXY (0, 0);
				PB_LCD_WriteString("---AC Voltage---", 50);
				ac_voltage();
				break;
			case 3 :
				PB_LCD_Clear();				
				PB_LCD_GoToXY (0, 0);
				PB_LCD_WriteString("----Current ----", 50);
				current();
				break;
			case 4 :
				menu_select_counter = 0;
				PB_LCD_Clear();				
				PB_LCD_GoToXY (0, 0);
				PB_LCD_WriteString("---Resistance---", 50);				
				resistance();
				break;
			default:
				PB_LCD_Clear();				
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
	timer_cnt++;
	
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
	DAC->DHR12R1 = 1;
	
	PB_LCD_Clear();
	PB_LCD_GoToXY (0, 0);
	PB_LCD_WriteString("---DC Voltage---", 50);
	
	const int ADC_VOLTAGE_REFERENCE = 3;
	const int ADC1_RESOLUTION = 4096;
	const float ADC2_RESOLUTION = 40.96;
	const float ADC1_DC_OFFSET;
	const float ADC2_DC_OFFSET;
	
	delay(STARTUP_SCREEN_DELAY);
	
	PB_LCD_Clear();
	PB_LCD_GoToXY(0,0);
	
	
		
	float ADC1_dc_reading_sum = 0;
	int ADC1_dc_sample_count = 0;
	float ADC2_dc_reading_sum = 0;
	int ADC2_dc_sample_count = 0;
	
	// redo some of this with the timer/external trigger input instead
	while(timer_cnt != 2){
		ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
		while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){}
			
		adc1_conv = ADC1->DR;
		ADC1_dc_reading_sum += adc1_conv;
	}
	
	if(timer_cnt == 2) {
		ADC_dc_sample_count = TIM1->CNT;
		timer_cnt = 0;
	}
	
	float ADC1_dc_raw_average = ADC1_dc_reading_sum/ADC_dc_sample_count;
		
	if(ADC1_dc_raw_average <= 40.96) {
		ADC1_dc_raw_average = 0;
			
		while(timer_cnt != 2){			
			ADC2->CR2 |= ADC_CR2_SWSTART_Msk; 
			while(ADC2->SR != (ADC2->SR | ADC_SR_EOC_Msk)){} 
				
			adc2_conv = ADC2->DR;	
			ADC2_dc_reading_sum += adc2_conv;
		}
		
		if(timer_cnt == 2) {
		ADC_dc_sample_count = TIM1->CNT;
		timer_cnt = 0;
		}
		
		float ADC2_dc_raw_average = ADC2_dc_reading_sum/ADC_dc_sample_count;
			
		float ADC2_real_dc_voltage = (ADC2_dc_raw_average/ADC2_RESOLUTION) * ADC_VOLTAGE_REFERENCE;
		float ADC2_output_dc_voltage = (((20 * ADC2_real_dc_voltage) - 30)/3) + ADC2_DC_OFFSET;
		
		PB_LCD_Clear();
		char output_dc_voltage[50]; 
		int output_dc_voltage_length = snprintf(output_dc_voltage, 50, "V-DC     %.4gmV", ADC2_output_dc_voltage);
		PB_LCD_WriteString(output_dc_voltage, output_dc_voltage_length);
		PB_LCD_GoToXY(0, 1);
		PB_LCD_WriteString("100mV", 50);
		
		ADC2_dc_reading_sum = ADC2_dc_raw_average;
		ADC2_dc_sample_count = 1;
		
	} else {
		float ADC1_real_dc_voltage = (ADC1_dc_raw_average/ADC1_RESOLUTION) * ADC_VOLTAGE_REFERENCE;
		float ADC1_output_dc_voltage = (((20 * ADC1_real_dc_voltage) - 30)/3) + ADC1_DC_OFFSET;
		
		PB_LCD_Clear();
		char output_dc_voltage[50]; 
		int output_dc_voltage_length = snprintf(output_dc_voltage, 50, "V-DC      %.4gV", ADC1_output_dc_voltage);
		PB_LCD_WriteString(output_dc_voltage, output_dc_voltage_length);
		PB_LCD_GoToXY(0, 1);
		PB_LCD_WriteString("10V", 50);
		
		ADC1_dc_reading_sum = ADC1_dc_raw_average;
		ADC1_dc_sample_count = 1;		
		
	}
	
	
}

void ac_voltage(void) {
	DAC->DHR12R1 = 2;
	ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
	while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){} 
	adc1_conv = ADC1->DR;
	ADC2->CR2 |= ADC_CR2_SWSTART_Msk; 
	while(ADC2->SR != (ADC2->SR | ADC_SR_EOC_Msk)){} 
	adc1_conv = ADC2->DR;	
}

void current(void) {
	DAC->DHR12R1 = 3;
	ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
	while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){} 
	adc1_conv = ADC1->DR;
}

void resistance(void) {
	DAC->DHR12R1 = 4;
	ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
	while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){} 
	adc1_conv = ADC1->DR;
}
void initialise_adc(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODE1_Msk;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT1_Msk;
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED1_Msk;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD1_Msk;
	
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

void timer_counter_setup(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;// Enables TIM1
	
	GPIOE->MODER &= ~GPIO_MODER_MODER7_Msk; //PE7 -> sets MODER to alternate function mode
	GPIOE->MODER = 2 << GPIO_MODER_MODER7_Pos; 
	GPIOE->PUPDR &= GPIO_PUPDR_PUPD7_Msk;
	GPIOE->PUPDR = 2 << GPIO_PUPDR_PUPD7_Pos;
	
	GPIOE->AFR[0] &= GPIO_AFRL_AFSEL7_Msk; // Select PE7 for alt func low reg
	GPIOE->AFR[0]  |= 1 << GPIO_AFRL_AFSEL7_Pos; // AF1 as that's what the alt func for TIM1_ETR is for PE7
	
	TIM1->SMCR |= TIM_SMCR_ECE;
	TIM1->CR1 |= TIM_CR1_CEN;
	
	/*
	could potentially replace the two respective sample count counters to just a single one 
	the sample count needed isn't initially set
	see how much the counter counts in one second using the timer thingy which should be the frequency
		of the signal generator.
	make a reading every time the counter goes up and add it to the sum value
	once reaching 1 second, set adc sample count to counter and divide the sum by the sample count
	use systick handler as this counts twice every second atm so 2 counts = 1 second
	*/
}

void dac_out(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= GPIO_MODER_MODE4_Msk; // (GPIOA->MODER & ~GPIO_MODER_MODER4_Msk) | (0x11 << GPIO_MODER_MODER4_Pos);
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT4_Msk;
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_Msk;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD4_Msk;
}