#include "stm32f407xx.h"
#include "Board_LED.h"
#include "PB_LCD_Drivers.h"
#include <stdio.h>
#include <stdlib.h>

void configPA0(void);
int checkButton(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);

void initialise_adc1(void);
void initialise_adc2(void);
void timer_counter_setup(void);
void mux_select_in(void);

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

int LCD_TIME_PASSED = 0;
const int LCD_REFRESH_RATE_SECONDS = 6; // The refresh rate x2 as SysTick_Handler runs twice a second due to the system core clock. Ideally not set to anything under 2 as 1 second is the default refresh rate.

const int BTN_DEBOUNCER_DELAY = 3;
int btn_tick_count = 0;
int cache_btn_tick_count = 0;
int timer_cnt = 0;

int ADC_dc_sample_count = 0;

uint32_t adc1_conv;
uint32_t adc2_conv;

void delay(int time) { // Allows a delay/brief pause in the program by an amount of time specified by user.
	cache_sleep_tick_count = sleep_tick_count;
	while (cache_sleep_tick_count+time > sleep_tick_count);
}


int main() {
	menu_select_counter = 0; // Initialises the variable used to change modes of the multimeter by incrementing.
	buttonState = 0;
	adc1_conv = 0;
	adc2_conv = 0;
	
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/2); // Rising edge twice a second.
		
	configPA0();
	NVIC_EnableIRQ(EXTI0_IRQn);
	
	
	PB_LCD_Init(); // Initialises the LCD screen
	initialise_adc1();
	initialise_adc2();
	//timer_counter_setup();
	mux_select_in();
	
	while(1) {
		// Start screen of multimeter, flashes through 2 different screens.
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

// Menu function allows to choose between different modes of the multimeter.
void menu() {
			switch(menu_select_counter) { // Uses switch statement as the variable input into the statement is a counter that can be cycled through with the push button.
			case 1 :
				dc_voltage(); // When menu_select_counter is 1, this sub function will run allowing dc voltage readings to be taken.
				break;
			case 2 :
				// Filler until functions written.
				PB_LCD_Clear();			
				PB_LCD_GoToXY (0, 0);
				PB_LCD_WriteString("---AC Voltage---", 50);
				//ac_voltage();
				break;
			case 3 :
				current();
				break;
			case 4 :
				menu_select_counter = 0;				
				resistance();
				break;
			default:
				// Displays an error message if an undefined state is ever reached and allows to reset menu counter.
				PB_LCD_Clear();				
				PB_LCD_GoToXY (0, 0);
				PB_LCD_WriteString("---- ERROR! ----", 50);
				PB_LCD_GoToXY (0, 1);
				PB_LCD_WriteString(" push btn again ", 50);
		}
}

// Setup PA0 to be used as a button with an interrupts  -> this is the GPIO port and pin that the blue button on the STM board is connected to.
// When pressed, will trigger an interrupt for the button.
void configPA0(void) { 
	// Configures PA0 as a input.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOE->MODER &= ~GPIO_MODER_MODE0_Msk;
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT0_Msk;
	GPIOE->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED0_Msk;
	GPIOE->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
	GPIOE->PUPDR |= 1 << GPIO_PUPDR_PUPD0_Pos;
		
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Enables System config
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // PA0 -> config interrupt to come from PA0

	EXTI->IMR |= EXTI_IMR_MR0; // External interrupt controller told to enable interrupts for pin 0.
	EXTI->FTSR |= EXTI_FTSR_TR0; // Determines that interrupts are generated on the falling edge of the system clock.
	
}

// Function used to check whether the button has been pressed based on the whether the appropriate bit of the input data register is set to 1 for PA0.
int checkButton(void) { 
	if ((GPIOA->IDR & 0x00000001) == 0x00000001){
		return 1;
	} else {
		return 0;
	}	
}	

// Runs at each rising of the system core clock, twice a second in our case
void SysTick_Handler(void){	
	if(buttonState != 0) { // Sets bit set/reset register of the button to 1 when pressed else the clears the respective bit.
		GPIOA->BSRR |= 0x00000001;
	} else {
		GPIOA->BSRR |= 0x00010000;
	}
	sleep_tick_count++; // Counter used for delay.
	//btn_tick_count++; // Used in debouncing the button.
	timer_cnt++; // Counter used to indicate 1 second has passed in the system in order to calculate the frequency of an input signal.
	LCD_TIME_PASSED++;
	
}

void EXTI0_IRQHandler(void) { // The interrupt handler for the button.
	EXTI->PR = EXTI_PR_PR0;
	if (checkButton()) {
		buttonState = 1;
		menu_select_counter++;
		if(menu_select_counter > 4) menu_select_counter = 0;	
		startup_screen_status = 0;
		menu();
	}
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
	
	TIM1->SMCR |= TIM_SMCR_ECE; // Sets the timer's input to be a clock source.
	TIM1->CR1 |= TIM_CR1_CEN; // Enables the timer as a counter
	
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


void dc_voltage(void) {
	
	GPIOB->BSRR = 0x00800000;  // Should be the LSB.
	GPIOB->BSRR = 0x01000000; // Should be the MSB.
	// Sets PA6 & PA7 to 0. Don't really need to do this as the values are set to 0 by default but just in case really.
	// Can connect a wire from PA6 & PA7 to the mux to receive the 2 inputs & decide what mode is selected (2 bits of binary correspond to the menu option (-1)).
	
	PB_LCD_Clear(); // Clears whatever is previously written to the LCD screen
	PB_LCD_GoToXY (0, 0);
	PB_LCD_WriteString("---DC Voltage---", 50); // Initial screen to indicate DC voltage measuring mode has been selected in the menu.
	
	// Instantiates the function specific (local) variables & constants. Make this into a global variable probably.
	const int ADC_VOLTAGE_REFERENCE = 3;
	const int ADC_RESOLUTION = 4096;
	const float ADC1_DC_OFFSET = 0;
	const float ADC2_DC_OFFSET = 0;
	
	//delay(STARTUP_SCREEN_DELAY); // Waits few seconds to read the mode change before the screen changes to the voltmeter.
	
	PB_LCD_Clear();
	PB_LCD_GoToXY(0,0);
		
	float ADC1_dc_reading_sum = 0;
	float ADC2_dc_reading_sum = 0;
	
	// As the SysTick_Handler runs twice a second, the timer used as a counter will take readings from the ADC 
	// 		until the SysTick_Handler runs the second time.
	while(timer_cnt != 2){
		ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
		while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){}
			
		adc1_conv = ADC1->DR; // Read only output from ADC, the final conversion of ADC
		// ADC readings are summed to be divided by the amount of times the timer counter is incremented to in one second.
		ADC1_dc_reading_sum += adc1_conv; 
	}

	if(timer_cnt == 2) { // program does not enter this for some reason
		//PB_LCD_WriteString("Reading", 50);
		ADC_dc_sample_count = TIM1->CNT; // Stores timer count into a variable so can be divided easily.
		TIM1->CNT = 0; // Resets counter for next reading -> does this need to be reset? Should an average of the frequency be taken I wonder?
		timer_cnt = 0; // Resets timer count to 0 so it can count 1 second in the SysTick_Handler again.
	}
	
	float ADC1_dc_raw_average = ADC1_dc_reading_sum/ADC_dc_sample_count; // Takes the average reading of the ADC at the frequency given.
	
	// Determines whether ADC1 or ADC2 is used if the average is below a certain threshold as ADC2 is used in the 100mV range.	
	if((2048 - 20.48) <= ADC1_dc_raw_average <= (2048 + 20.48)) { 
		ADC1_dc_raw_average = 0;
			
		// Similar to ADC1 but for ADC2 instead.
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
			
		float ADC2_real_dc_voltage = (ADC2_dc_raw_average/ADC_RESOLUTION) * ADC_VOLTAGE_REFERENCE; // Changed back into an analogue value of voltage.
		
		// As the hardware changes potential +/- 10V range to 0-3V, this calculation changes it back into the correct form.
		// There is slight offset in the readings and the actual measure ment so offset is needed here in calculation.
		float ADC2_output_dc_voltage = (((2 * ADC2_real_dc_voltage) - 3)/30) + ADC2_DC_OFFSET; 
		
		PB_LCD_Clear();
		
		// Reading is turned into a 16 bit string to be written to the 2x16 bit LCD screen.
		char output_dc_voltage[50]; 
		int output_dc_voltage_length = snprintf(output_dc_voltage, 50, "V-DC     %.4gmV", ADC2_output_dc_voltage * 1000); // The ADC 2 reading is multiplied by 1000 to convert from volts to millivolts.
		
		if(LCD_TIME_PASSED == LCD_REFRESH_RATE_SECONDS){ // Displays the average reading every 3 seconds instead of one second to make readings clearer to read - can adjust the constant value to fit desired refresh rate.
			PB_LCD_WriteString(output_dc_voltage, output_dc_voltage_length);
			PB_LCD_GoToXY(0, 1);
			PB_LCD_WriteString("100mV", 50); 
			LCD_TIME_PASSED = 0;
		}
		
		// Frees memory by making variable contents smaller.
		ADC2_dc_reading_sum = ADC2_dc_raw_average;
		ADC_dc_sample_count = 1;
		
	} else { // For the ADC1 case in case the ADC reading is not in the ADC2 range.
		float ADC1_real_dc_voltage = (ADC1_dc_raw_average/ADC_RESOLUTION) * ADC_VOLTAGE_REFERENCE;
		float ADC1_output_dc_voltage = (((20 * ADC1_real_dc_voltage) - 30)/3) + ADC1_DC_OFFSET;
		
		PB_LCD_Clear();
		char output_dc_voltage[50]; 
		int output_dc_voltage_length = snprintf(output_dc_voltage, 50, "V-DC      %.4gV", ADC1_output_dc_voltage);
		
		if(LCD_TIME_PASSED == LCD_REFRESH_RATE_SECONDS){
			PB_LCD_WriteString(output_dc_voltage, output_dc_voltage_length);
			PB_LCD_GoToXY(0, 1);
			PB_LCD_WriteString("10V", 50);
			LCD_TIME_PASSED = 0;
		}
		
		ADC1_dc_reading_sum = ADC1_dc_raw_average;
		ADC_dc_sample_count = 1;		
	}
}

void ac_voltage(void) {
	
	// Used to reset the MUX input.
	GPIOB->BSRR = 0x00800000;  
	GPIOB->BSRR = 0x01000000;
	
	// Input 01 into the MUX.
	GPIOB->BSRR = 0x00000080; // Should be the LSB.
	GPIOB->BSRR = 0x01000000; // Should be the MSB.
	
	ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
	while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){} 
	adc1_conv = ADC1->DR;
	ADC2->CR2 |= ADC_CR2_SWSTART_Msk; 
	while(ADC2->SR != (ADC2->SR | ADC_SR_EOC_Msk)){} 
	adc1_conv = ADC2->DR;	
}

void current(void) {
	
	// Used to reset the MUX input.
	GPIOB->BSRR = 0x00800000;  
	GPIOB->BSRR = 0x01000000;
	
	// Input 10.
	GPIOB->BSRR = 0x00800000; // Should be the LSB.
	GPIOB->BSRR = 0x00000100; // Should be the MSB.
	 
	
	PB_LCD_Clear(); // Clears whatever is previously written to the LCD screen
	PB_LCD_GoToXY (0, 0);
	PB_LCD_WriteString("----Current ----", 50);
	
	// Instantiates the function specific (local) variables & constants.
	const int ADC_VOLTAGE_REFERENCE = 3;
	const int ADC_RESOLUTION = 4096;
	const float ADC_CURRENT_OFFSET = 0;
	
	delay(STARTUP_SCREEN_DELAY); // Waits few seconds to read the mode change before the screen changes to the voltmeter.
	
	PB_LCD_Clear();
	PB_LCD_GoToXY(0,0);
		
	float ADC_current_raw_reading_sum = 0;
	int ADC_current_sample_count = 0;
		
	while(timer_cnt != 2){
		ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
		while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){}
			
		adc1_conv = ADC1->DR; // Read only output from ADC, the final conversion of ADC
		// ADC readings are summed to be divided by the amount of times the timer counter is incremented to in one second.
		ADC_current_raw_reading_sum += adc1_conv; 
	}
	
	if(timer_cnt == 2) {
		ADC_current_sample_count = TIM1->CNT; // Stores timer count into a variable so can be divided easily.
		TIM1->CNT = 0; // Resets counter for next reading -> does this need to be reset? Should an average of the frequency be taken I wonder?
		timer_cnt = 0; // Resets timer count to 0 so it can count 1 second in the SysTick_Handler again.
	}
	
	float ADC_current_raw_average = ADC_current_raw_reading_sum/ADC_current_sample_count;
	float ADC_real_current_voltage = (ADC_current_raw_average/ADC_RESOLUTION) * ADC_VOLTAGE_REFERENCE;
	float ADC_output_current = (((200 * ADC_real_current_voltage) - 300)/3) + ADC_CURRENT_OFFSET;
	
	PB_LCD_Clear();
	char output_current[50]; 
	int output_current_length = snprintf(output_current, 50, "CRNT     %.4gmA", ADC_output_current);
	
	if(LCD_TIME_PASSED == LCD_REFRESH_RATE_SECONDS){
		PB_LCD_WriteString(output_current, output_current_length);
		LCD_TIME_PASSED = 0;
	}
	
	ADC_current_raw_reading_sum = ADC_current_raw_average;
	ADC_current_sample_count = 1;	
}

void resistance(void) {
	
	// Used to reset the MUX input.
	GPIOB->BSRR = 0x00800000;  
	GPIOB->BSRR = 0x01000000;
	
	// Input 11.
	GPIOB->BSRR = 0x00000080; // Should be the LSB.
	GPIOB->BSRR = 0x00000100; // Should be the MSB.
	
	PB_LCD_Clear();				
	PB_LCD_GoToXY (0, 0);
	PB_LCD_WriteString("---Resistance---", 50);
	
	// Instantiates the function specific (local) variables & constants.
	const int ADC_VOLTAGE_REFERENCE = 3;
	const int ADC_RESOLUTION = 4096;
	const float ADC_RESISTANCE_OFFSET = 0;
	
	delay(STARTUP_SCREEN_DELAY); // Waits few seconds to read the mode change before the screen changes to the voltmeter.
	
	PB_LCD_Clear();
	PB_LCD_GoToXY(0,0);
		
	float ADC_resistance_raw_reading_sum = 0;
	int ADC_resistance_sample_count = 0;
		
	while(timer_cnt != 2){
		ADC1->CR2 |= ADC_CR2_SWSTART_Msk; 
		while(ADC1->SR != (ADC1->SR | ADC_SR_EOC_Msk)){}
			
		adc1_conv = ADC1->DR; // Read only output from ADC, the final conversion of ADC
		// ADC readings are summed to be divided by the amount of times the timer counter is incremented to in one second.
		ADC_resistance_raw_reading_sum += adc1_conv; 
	}
	
	if(timer_cnt == 2) {
		ADC_resistance_sample_count = TIM1->CNT; // Stores timer count into a variable so can be divided easily.
		TIM1->CNT = 0; // Resets counter for next reading -> does this need to be reset? Should an average of the frequency be taken I wonder?
		timer_cnt = 0; // Resets timer count to 0 so it can count 1 second in the SysTick_Handler again.
	}
	
	float ADC_resistance_raw_average = ADC_resistance_raw_reading_sum/ADC_resistance_sample_count;
	float ADC_real_resistanve_voltage = (ADC_resistance_raw_average/ADC_RESOLUTION) * ADC_VOLTAGE_REFERENCE;
	float ADC_output_resistance = ((1000000 * ADC_real_resistanve_voltage)/3) + ADC_RESISTANCE_OFFSET;
	
	PB_LCD_Clear();
	PB_LCD_WriteString("RSTS", 50);
	
	char output_resistance[50]; 
	int output_resistance_length = snprintf(output_resistance, 50, "    %.7g Ohms", ADC_output_resistance);
	
	if(LCD_TIME_PASSED == LCD_REFRESH_RATE_SECONDS){
		PB_LCD_GoToXY(0,1);
		PB_LCD_WriteString(output_resistance, output_resistance_length);
		LCD_TIME_PASSED = 0;
	}
	
	ADC_resistance_raw_reading_sum = ADC_resistance_raw_average;
	ADC_resistance_sample_count = 1;	
}

// PA1 -> input 1 of ADC 1 and 2 set up here. change to PC4
void initialise_adc1(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODE4_Msk;
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT4_Msk;
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED4_Msk;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD4_Msk;
	
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	ADC1->SQR3 &= 0xFFFFFFE0;
	
	ADC1->SQR3 |= 0x00000004;
	
	ADC1->CR2 |= ADC_CR2_ADON_Msk;  
	ADC1->CR1 |= ADC_CR1_DISCEN_Msk;
	ADC1->CR2 |= ADC_CR2_EOCS_Msk;
}

void initialise_adc2(void) { // change to PC5
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	GPIOC->MODER |= GPIO_MODER_MODE5_Msk;
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT5_Msk;
	GPIOC->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED5_Msk;
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD5_Msk;
	
	RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
	
	ADC2->SQR3 &= 0xFFFFFFE0;
	
	ADC2->SQR3 |= 0x00000005;
	
	ADC2->CR2 |= ADC_CR2_ADON_Msk; 
	ADC2->CR1 |= ADC_CR1_DISCEN_Msk;
	ADC2->CR2 |= ADC_CR2_EOCS_Msk;
}


void mux_select_in(void) {
	// Setup PA6 & PA7 as digital outputs so can be set to 0 or 1 via the BSRR. LSB to PB7 MSB to PB8
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	
	// PA6 to be select 1 of the MUX. PB7
	GPIOB->MODER &= ~GPIO_MODER_MODE7_Msk;
	GPIOB->MODER |= 1 << GPIO_MODER_MODE7_Pos;
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT7_Msk;
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED7_Msk;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD7_Msk;
	
	// PA7 to be select 2 of the MUX. PB8
	GPIOB->MODER &= ~GPIO_MODER_MODE8_Msk;
	GPIOB->MODER |= 1 << GPIO_MODER_MODE8_Pos;
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT8_Msk;
	GPIOB->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED8_Msk;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD8_Msk;	
}
