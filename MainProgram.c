#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.h"

//necessary variables for the LCD
unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char sharp, distance, adc_reading;
unsigned int value;
float BATT_Voltage, BATT_V;

//Function to configure the pins for motion
void motion_pin_config (void){
	DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
	PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

//Function to configure LCD port
void lcd_port_config (void){
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void){
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}

//Function to configure Buzzer port
void buzzer_pin_config (void){
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to initialize ports
void port_init(){
 	motion_pin_config();
	lcd_port_config();
	adc_pin_config();
	buzzer_pin_config();
}

//Function to Initialize ADC
void adc_init(){
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}


//Function used for setting motor's direction
void motion_set (unsigned char Direction){
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 			// removing upper nibble as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibble to 0
	PortARestore |= Direction; 	// adding lower nibble for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}


void forward (void){     //both wheels forward
	motion_set(0x06);
}

void back (void){       //both wheels backward
	motion_set(0x09);
}

void left (void){       //Left wheel backward, Right wheel forward
	motion_set(0x05);
}

void right (void){      //Left wheel forward, Right wheel backward
	motion_set(0x0A);
}

void soft_left (void){  //Left wheel stationary, Right wheel forward
	motion_set(0x04);
}

void soft_right (void){ //Left wheel forward, Right wheel is stationary
	motion_set(0x02);
}

void soft_left_2 (void){//Left wheel backward, right wheel stationary
	motion_set(0x01);
}

void soft_right_2 (void){ //Left wheel stationary, Right wheel backward
	motion_set(0x08);
}

void stop (void){       //hard stop
	motion_set(0x00);
}

//This Function accepts the Channel Number and returns the corresponding Analog Value 
unsigned char ADC_Conversion(unsigned char Ch){
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location. 
void print_sensor(char row, char coloumn,unsigned char channel){
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function to update LCD
//LCD in this program will display the distance from the front SHARP sensor
void updateLCD(){
	sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
	lcd_print(1,6,value,3);						//Prints Value Of Distanc in MM measured by Sharp Sensor.
}


// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor. 
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading){
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

//Function to switch on buzzer
void buzzer_on (void){
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

//Function to switch off buzzer
void buzzer_off (void){
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

//Function to initialize the device
void init_devices (void){
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	sei(); //Enables the global interrupts
}


//Main Function
int main(){

	//initialization
	init_devices();
	lcd_set_4bit();
	lcd_init();
		
	while(1){//infinite while loop

		updateLCD();//display distance related from the sharp sensor on the LCD
	
		forward(); //both wheels forward
		_delay_ms(1000);

		stop();						
		_delay_ms(500);
	
		back(); //bpth wheels backward						
		_delay_ms(1000);

		stop();						
		_delay_ms(500);
		 
	}
}

