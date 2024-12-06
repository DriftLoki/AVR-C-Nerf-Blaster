#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdlib.h>

#define MAX_THROTTLE 1250
#define MIN_THROTTLE 1032

// Global variables
volatile uint16_t pulse_width = MIN_THROTTLE; // Start with minimum throttle (1016 μs
#define PB4 4

volatile uint8_t pushState = 0;        // Start solenoid push state as unpowered
volatile uint8_t fireTriggerState = 0; // Switch state start as unpressed (0)
volatile uint8_t DPS = 1; //BETA

int MF=0;

void init_timer1(void) {
	// Configure Timer 1
	TCCR1A = 0x00;                       // Normal mode
	TCCR1B |= (1 << CS11) | (1 << CS10); // Set prescaler to 64
	// Calculate the required preload based on fireFrequency
	uint16_t preload = 65536 - (F_CPU / 64 / DPS); // math for delay

	TCNT1 = preload;        // Set the initial value for TCNT1
	TIMSK1 |= (1 << TOIE1); // Enable Timer 1 Overflow Interrupt
}




void LCD_cmd(unsigned char cmd)

{

	PORTD = cmd;      // data lines are set to send command*

	PORTC  &= ~(1<<5);  // RS sets 0

	PORTC  &= ~(1<<4);  // RW sets 0

	PORTC  |= (1<<3);   // make enable from high to low

	_delay_ms(10);

	PORTC  &= ~(1<<3);

	

	return;

}

void init_LCD(void)

{
	DDRD = 0xFF; //Output to LCD
	DDRC |= (1 << 3);  //A3
	DDRC |= (1 << 4);	// A4
	DDRC |= (1 << 5);	// A5

	LCD_cmd(0x38);           // initialization in 8bit mode of 16X2 LCD

	_delay_ms(1);

	LCD_cmd(0x01);          // make clear LCD

	_delay_ms(1);

	LCD_cmd(0x02);          // return home

	_delay_ms(1);

	LCD_cmd(0x06);          // make increment in cursor

	_delay_ms(1);

	LCD_cmd(0x80);          //  8  go to first line and  0  is for 0th position

	_delay_ms(1);

	LCD_cmd(0x0C);  // DISPLAY ON, CURSOR OFF

	_delay_ms(1);
	return;

}


void LCD_write(unsigned char data)

{

	PORTD= data;       // data lines are set to send command

	PORTC  |= (1<<5);    // RS sets 1

	PORTC  &= ~(1<<4);   // RW sets 0

	PORTC  |= (1<<3);    // make enable from high to low

	_delay_ms(10);

	PORTC &= ~(1<<3);

	

	return ;

}



void LCD_Write_String(char *a)
{
	int i;
	for(i=0;a[i]!='\0';i++)
	{
		char big = a[i];
		LCD_write(big);
	}
	
}

void set_pos(unsigned char x, unsigned char y)//x:0~1, y:0~15
{
	uint8_t address = 0;
	if (x==0)
	address = 0x80;
	else if (x==1)
	address = 0xC0;
	if (y<16)
	address += y;
	LCD_cmd(address);
}

void update_lcd() {
	set_pos(0, 6);
	LCD_Write_String("   ");
	set_pos(0, 6);
	char str[3];
	itoa(DPS,str,10);
	LCD_Write_String(str);
}

// Function to configure Timer0 for PWM generation
void setupPWM(void) {
	DDRB |= (1 << PB4);          // Set PB0 as output
	TCCR0A = (1 << WGM01);       // CTC Mode
	TCCR0B = (1 << CS01);        // Prescaler of 8
	TIMSK0 = (1 << OCIE0A);      // Enable Compare Match A interrupt
}

// Function to arm the ESC
void arm_ESC(void) {
	DDRB |= (1 << PB4); // Set PB0 as output

	// Send 1050 μs signal for 1 second
	for (int i = 0; i < 50; i++) {
		PORTB |= (1 << PB4);
		_delay_us(1050);
		PORTB &= ~(1 << PB4);
		_delay_us(18950); // Total period = 20 ms
	}

	// Send 1060 μs signal for a few milliseconds
	for (int i = 0; i < 5; i++) {
		PORTB |= (1 << PB4);
		_delay_us(1060);
		PORTB &= ~(1 << PB4);
		_delay_us(18940); // Total period = 20 ms
	}

	// Set initial throttle to minimum
	pulse_width = MIN_THROTTLE;
}

// Main program
int main(void) {
	setupPWM();   // Configure Timer0
	arm_ESC();    // Perform arming sequence
	
	DDRB |= (1 << 2);  // solenoid output
	DDRB &= ~(1 << 1); // switch input
	PORTB |= (1 << 1); // switch pull up resistor
	
	DDRC |=~(1<<0);
	DDRB |=~(1<<0); //BETA
	DDRD = 0xFF;
	PCICR |=(1<<PCIE1);
	PCICR |=(1<<PCIE0);
	PCMSK1 |=(1<<0);
	PCMSK0 |=(1<<0);
	

	init_timer1(); // Initialize Timer 1
	init_LCD();    // Initialize LCD
	LCD_Write_String("DPS:");
	LCD_cmd(0xC0);  // Move cursor to beginning of second line
	_delay_ms(10);
	LCD_Write_String("M_Speed: 1200");
	_delay_ms(10);
	update_lcd();
	
	sei();        // Enable global interrupts
	
	DDRB &= ~(1 << 3); // switch input
	PORTB |= (1 << 3); // switch pull up resistor

	// Increment throttle signal gradually
	while (1) {
		
		if (PINB & (1 << 1)) {  // if firetrigger pressed, mark state as 1
			fireTriggerState = 1; // Switch pressed (logic high)
		} else {
			fireTriggerState = 0; // Switch released (logic low)
			PORTB &= ~(1 << 2);   // force stop power to solenoid
			pushState = 0;        // mark solenoid as 0
		}
		
		if ((PINB & (1 << 3)) == 0) {
			pulse_width = MAX_THROTTLE;
		} else {
			pulse_width = 1050;
		}
	}
}

// Timer0 Compare Match Interrupt
ISR(TIMER0_COMPA_vect) {
	static uint8_t state = 0;

	if (state == 0) {
		// Start of pulse
		PORTB |= (1 << PB4);
		OCR0A = (pulse_width * 2) / 8; // Calculate compare match value for high time
		state = 1;
	} else {
		// End of pulse
		PORTB &= ~(1 << PB4);
		OCR0A = (20000 - pulse_width) * 2 / 8; // Calculate compare match value for low time
		state = 0;
	}
}

// Interrupt Service Routine for Timer 1 Overflow
ISR(TIMER1_OVF_vect) {
	if (fireTriggerState) {   // Check if the switch is still pressed
		pushState = !pushState; // Toggle pushState
		if (pushState) {
			PORTB |= (1 << 2); // power solenoid
			} else {
			PORTB &= ~(1 << 2); // unpower solenoid
		}
		// Recalculate preload value for the next cycle based on fireFrequency
		volatile uint16_t preload = 65536 - (F_CPU / 64 / DPS);
		TCNT1 = preload; // Reload Timer 1 with the new preload value
	}
	
}

ISR(PCINT1_vect){ //BETA
	if(DPS<15){
		if((PINC &(1<<0))==1){
			DPS+=1;
			update_lcd();
		}
	}
}

ISR(PCINT0_vect){  //BETA
	if(DPS>1){
		if((PINB &(1<<0))==1){
			DPS-=1;
			update_lcd();
		}
	}
}
