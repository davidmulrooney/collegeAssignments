/*David Mulrooney_22361715
/ Eoin Dooley_22358773
/ Digital_Systems_3
/ Project_2
/ Submission:02-05-2024
/ 
/ Note: Code Compiles and runs, but does not receive a response from the micro controller, 
/ this may be because of the updateServoPosition() or calculateInterruptPeriod() functions.
/
*/
#define F_CPU 20000000  // Define the CPU clock frequency

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdbool.h>

// Function prototypes
void Clock_Init();
void PORT_Init();
void TCA0_Init();
void TCB0_Init();
void TCB1_Init();
void TCB3_Init();
void USART_Init();
void ADC0_Init();
void EVESYS_Init();
void updateServoPosition();
void calculateInterruptPeriod(int speed);

// Global variables – Explain their purpose and expected behavior
volatile int current_servo_speed;                   // Current speed selected by the user
volatile bool rising_edge_captured;                 // Tracks state in input capture
volatile uint16_t first_capture_count;              // Used for timeout detection
volatile int duty_cycle_us;                         // Holds the desired servo duty cycle
volatile int period, high_time, low_time;           // Stores input signal measurements
volatile bool signal_timeout = false;               // Flags if the input signal has stopped
volatile int adc_value;                             // Holds the latest ADC conversion result
volatile const char *current_string = NULL;         // Pointer for text transmission via USART
volatile bool continuous_timer_mode = false;        // Tracks if continuous timer reporting is enabled
volatile bool continuous_adc_mode = false;          // Tracks if continuous ADC reporting is enabled
volatile uint16_t previous_capture;                 // Stores the previous timer capture value
volatile bool new_adc_data_flag = false;            // Signals availability of new ADC data

// Constants with clear explanations
#define TIMEOUT_THRESHOLD (500000 / 0.1) // Timeout in timer counts (500ms timeout)
#define V_TH_35 716                      // ADC value corresponding to 3.5V input

// Macro for efficient baud rate calculation
#define USART0_Baud_Calculation(BAUD)((float)(F_CPU * 64 / (16 * (float) BAUD)) + 0.5) 
//(3.5V / 5V) * 1023  = 716.1 - Voltage to display bit6

void Clock_Init() {
	/*// Enable the 20 MHz internal oscillator
	OSC.CTRL |= OSC_RC20MEN_bm;

	// Wait for the oscillator to stabilize
	while(!(OSC.STATUS & OSC_RC20MRDY_bm));

	// Select the 20 MHz oscillator as the clock source (no prescaler)
	ccp_write_io((void *) &CLK.CTRL, CLK_SCLKSEL_RC20M_gc);
*/
	// Configure the internal oscillator for 20 MHz operation and enable it.
	ccp_write_io(&CLKCTRL.MCLKCTRLB, (0 << CLKCTRL_PEN_bp));
}
void EVSYS_Init() {
	// Configure the Event System (EVSYS) to route events between peripherals:
	EVSYS.CHANNEL4 = EVSYS_GENERATOR_PORT0_PIN3_gc; // Input capture signal -> TCB0
	EVSYS.USERTCB0 = EVSYS_CHANNEL_CHANNEL4_gc;
	EVSYS.CHANNEL0 = EVSYS_GENERATOR_TCB3_CAPT_gc; // TCB3 overflow -> ADC0
	EVSYS.USERADC0 = EVSYS_CHANNEL_CHANNEL0_gc;
}
void TCA0_Init() {
	// Configure Timer/Counter A0 (TCA0) for PWM servo control:
	TCA0.SINGLE.PER = 24999;        // Set period for 50 Hz PWM
	TCA0.SINGLE.CMP1 = 1250;        // Initial compare value (middle position)
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm; // Prescaler 16, enable
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc; // Single-slope PWM mode
}
void TCB0_Init() {
	// Configure Timer/Counter B0 (TCB0) for input signal capture:
	TCB0.INTCTRL = TCB_CAPT_bm;     // Enable capture interrupt
	TCB0.EVCTRL = TCB_CAPTEI_bm | TCB_EDGE_bm; // Trigger on rising edge
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; // Prescaler 2, enable
	TCB0.CTRLB = TCB_CNTMODE_FRQPW_gc; // Frequency and pulse width measurement mode
}
void TCB1_Init() {
	TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;  // CLK_PER/2
	TCB1.CTRLB = 0b01000001;
	TCB1.EVCTRL = ~TCB_EDGE_bm | TCB_CAPTEI_bm;
}
void TCB3_Init() {
	// Configure TCB3 to generate periodic interrupts for ADC triggering and servo updates:
	TCB3.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
	TCB3.CTRLB = TCB_CNTMODE_INT_gc;                // Periodic interrupt mode
	calculateInterruptPeriod(1);                    // Initial period for slowest speed
	TCB3.INTCTRL = TCB_CAPT_bm;                     // Enable overflow interrupt
}
void ADC0_Init() {
	ADC0.CTRLA = 0b00000001; // Enable ADC, free-running mode, 1x prescaler
	ADC0.CTRLB = 0b00000000; // Default, no samples accumulated
	ADC0.CTRLC = 0b00010101; // 10-bit resolution, VCC reference
	ADC0.CTRLD = 0b00000000; // Default settings, no digital windowing
	ADC0.MUXPOS = ADC_MUXPOS_AIN3_gc; // Select AIN3 as the input channel
	ADC0.EVCTRL = ADC_STARTEI_bm; // ADC conversion triggered by event
	ADC0.INTCTRL = ADC_RESRDY_bm; // Enable interrupt on conversion complete
}
void USART_Init() {
	USART0.CTRLA = USART_TXCIE_bm; // Enable transmit complete interrupt
	USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_TXCIE_bm; // Enable RX and TX, enable TX interrupt
	USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc; // a sync, no parity, 8-bit data
	USART0.BAUD = (uint16_t) USART0_Baud_Calculation(115200); // Set baud rate to 115200
}
int main() {
	bool new_high_time_flag = false;
	bool new_adc_data_flag = false;
	//bool first_capture_done = false;
	//uint16_t first_capture_count;
	
	// Initialization
	Clock_Init();  // Ensures 20 MHz operation
	EVSYS_Init();  
	PORT_Init();   // Configure output pins for PWM and LEDs
	TCA0_Init();
	TCB0_Init();
	TCB1_Init();  
	TCB3_Init();
	USART_Init();
	ADC0_Init();
	TCA0_Servo_Init();
	TCB0_Servo_Update_Init();
	
	volatile int period, high_time, low_time;
	volatile bool signal_timeout = false;
	volatile bool new_timer_data_flag = false;
	previous_capture = 0;
	char output_str[80];


	sei(); // Enable global interrupts

	while (1) {
		// Check if a character is received
		if (USART0.STATUS & USART_RXCIF_bm) {
			char received_char = USART0.TXDATAL;

			switch(received_char) {
				
				case 'c':
				case 'C':  // Start Continuous Timer Reporting
				continuous_timer_mode = true;
				break;
				
				case 'e':
				case 'E':  // Stop Continuous Timer Reporting
				continuous_timer_mode = false;
				break;
				
				case 'm':
				case 'M': // Start Continuous ADC Reporting
				continuous_adc_mode = true;
				break;
				
				case 'n':
				case 'N': // Stop Continuous ADC Reporting
				continuous_adc_mode = false;
				break;
				
				case 't':
				case 'T':
				case 'l':
				case 'L':
				case 'h':
				case 'H':
				if (signal_timeout) {
					send_string("Signal stopped!\n");
					signal_timeout = false;
					} else {
					// Format and send the values of period, high_time, low_time
					char output_str[80];

					// Choose which value to report based on the received_char
					switch(received_char) {
						case 't':  case 'T':
						sprintf(output_str, "Timer period = %d us\n", period);
						break;
						case 'l':  case 'L':
						sprintf(output_str, "Low pulse width = %d us\n", low_time);
						break;
						case 'h':  case 'H':
						sprintf(output_str, "High pulse width = %d us\n", high_time);
						break;
					}
					send_string(output_str);
				}
				break;
				
				case 'a':
				case 'A'://ADC Report
				if (new_adc_data_flag) {
					char output_str[80];
					sprintf(output_str, "ADC value = %d\n", adc_value);
					send_string(output_str);
					new_adc_data_flag = false;
				}
				break;
				
				case 'v':
				case 'V'://Voltage report
				if (new_adc_data_flag) {
					int millivolts = (adc_value * 3300) / 1023;  // Max ADC value = 1023 for 10-bit resolution

					char output_str[80];
					sprintf(output_str, "Voltage = %d mV\n", millivolts);
					send_string(output_str);
					new_adc_data_flag = false;
				}
				break;
				
				case '0':
				current_servo_speed = 0;
				break;
				case '1':
				current_servo_speed = 1;
				break;
				case '2':
				current_servo_speed = 2;
				break;
				case '3':
				current_servo_speed = 3;
				break;
				case '4':
				current_servo_speed = 4;
				break;
				case '5':
				current_servo_speed = 5;
				break;
				case '6':
				current_servo_speed = 6;
				break;
				case '7':
				current_servo_speed = 7;
				break;
				case '8':
				current_servo_speed = 8;
				break;
				case '9':
				current_servo_speed = 9;
				break;

			}
			calculateInterruptPeriod(current_servo_speed);
			
			if (continuous_timer_mode) {
				if (new_timer_data_flag) {
					char output_str[80];
					sprintf(output_str, "Timer period = %d us\n", period);
					send_string(output_str);
					new_timer_data_flag = false;
				}
			}

			if (continuous_adc_mode) {
				if (new_adc_data_flag) {
					int millivolts = (adc_value * 3300) / 1023;
					char output_str[80];
					sprintf(output_str, "Voltage = %d mV\n", millivolts);
					send_string(output_str);
					new_adc_data_flag = false;
				}
			}
			if (signal_timeout) {
				send_string("Signal stopped!\n");
				signal_timeout = false;
			}
			
			if (new_high_time_flag) {
				sprintf(output_str, "High pulse width = %d us\n", high_time);
				send_string(output_str);  // send_string function
				new_high_time_flag = false; // Reset the flag
			}
		}
	}

	return 0;
}
ISR(ADC0_RESRDY_vect) {
	uint16_t result = ADC0.RES; // Read the ADC conversion result
	adc_value = result;         // Store the result
	new_adc_data_flag = true;   // Signal that new data is available

	if (result > V_TH_35) {     // Compare ADC value to a threshold
		PORTF.OUTSET = PIN4_bm; // Set LED pin high 
		} else {
		PORTF.OUTCLR = PIN4_bm; // Set LED pin low
	}
	TCB3.INTFLAGS = 0b00000001; // This clears a flag to re trigger ADC
}
ISR(TCB0_INT_vect) {
	TCB0.INTFLAGS = TCB_CAPT_bm;    // Clear the TCB0 interrupt flag
	updateServoPosition();          // Update the servo position
	calculateInterruptPeriod(current_servo_speed); // Recalculate servo timing

	uint16_t current_capture = TCB0.CCMP;  // Read the current capture value

	if (rising_edge_captured) {
		high_time = current_capture - previous_capture;
		rising_edge_captured = false;   // Prepare for detecting the next falling edge
		first_capture_count = 0;        // Reset the timeout counter
		} else {
		low_time = current_capture - previous_capture;
		rising_edge_captured = true;    // Prepare for detecting the next rising edge
		first_capture_count = 0;        // Reset the timeout counter
	}

	// Calculate period (if possible with two edges)

	previous_capture = current_capture; // Store for the next cycle
	first_capture_count++;              // Increment timeout counter
	if (first_capture_count > TIMEOUT_THRESHOLD) {
		signal_timeout = true;          // Signal timeout if no new edges occur
		first_capture_count = 0;        // Reset timeout counter
	}
}
ISR(TCB3_INT_vect) {
	TCB3.INTFLAGS = TCB_CAPT_bm;    // Clear flag
	updateServoPosition();          // Update servo position
	calculateInterruptPeriod(current_servo_speed); // Recalculate timing
}
ISR(USART0_TXC_vect) {
	if (current_string) {
		if (*current_string != '\0') {
			USART0.TXDATAL = *current_string; // Send the next character
			current_string++;                // Move to the next character
			} else {
			current_string = NULL;           // Transmission complete
		}
	}
}
void send_string(const char *str) {
	current_string = str;           // Store the pointer to the beginning of the string
	USART0.TXDATAL = *current_string;// Send the first character
	current_string++;               // Move the pointer to the next character
	USART0.CTRLB |= USART_TXCIE_bm; // Enable the transmission complete interrupt
}
void calculateInterruptPeriod(int speed) {
	if (speed >= 0 && speed <= 9) {
		double desired_period = 0.0;
		// ... (cases for mapping speed to desired_period) ...
		switch (speed) {
			case 0:  desired_period = 0; break; // No movement
			case 1:  desired_period = 1.0; break;
			case 2:  desired_period = 0.5; break;
			case 3:  desired_period = 0.4; break;
			case 4:  desired_period = 0.3; break;
			case 5:  desired_period = 0.25; break;
			case 6:  desired_period = 0.2; break;
			case 7:  desired_period = 0.15; break;
			case 8:  desired_period = 0.1; break;
			case 9:  desired_period = 0.05; break;
		}

		// Convert desired_period (in seconds) to timer counts:
		unsigned int period_counts = (unsigned int)(desired_period * (F_CPU / 2));
		TCB3.CCMP = period_counts;
	}
}
void updateServoPosition() {
	unsigned int compare_value = (unsigned int)(duty_cycle_us * (F_CPU / 1000000) / TCA0.SINGLE.PER);
	TCA0.SINGLE.CMP0 = compare_value;
}
void TCA0_Servo_Init() {
	TCA0.SINGLE.PER = 39999; // 20ms period 
	TCA0.SINGLE.CMP0 = 1500; // Initial duty cycle for middle position
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}
void TCB0_Servo_Update_Init() {
	TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
	TCB0.CTRLB = TCB_CNTMODE_INT_gc;
	TCB0.INTCTRL = TCB_CAPT_bm;
	calculateInterruptPeriod(1);
}
void PORT_Init() {
	// Set PORTA Pin 1, PORTA Pin 0, PORTF Pin 5, PORTC Pin 6, PORTF Pin 4 as outputs
	PORTA.DIRSET = PIN1_bm | PIN0_bm;
	PORTF.DIRSET = PIN5_bm | PIN4_bm;
	PORTC.DIRSET = PIN6_bm;
}