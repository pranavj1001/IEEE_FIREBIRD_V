/*
 * MainProgram.c
 *
 * Created: 02/02/2017 11:48:14 AM
 *  Author: Pranav Jain
 */ 


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

int delayTime;
int stopped = 0;
int runThisCode = 0;
int timerCount = 0;
int insideAlternatePath = 0;
int exclusiveForLoopVariable = 0;
int alternatePathDelaytTime = 0;
int servoPositionCounter = 0;
int servoPath = 10;
int servoCounter1 = 0;
int servoCounter2 = 0;

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

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
	DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
	PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Function to initialize ports
void port_init(){
 	motion_pin_config();
	lcd_port_config();
	adc_pin_config();
	buzzer_pin_config();
	servo2_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
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

//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	float PositionTiltServo = 0;
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionTiltServo;
}

//servo_free functions unlocks the servo motors from the any angle
//and make them free by giving 100% duty cycle at the PWM. This function can be used to
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_2_free (void) //makes servo 2 free rotating
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
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

//Function to update LCD
//LCD in this program will display the distance from the front SHARP sensor
void updateLCD(){
	sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calculated in a variable "value".
	lcd_print(1,6,value,3);						//Prints Value Of Distance in MM measured by Sharp Sensor.
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

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init(){
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void){
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor){
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function to initialize the device
void init_devices (void){
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	timer1_init();
	sei(); //Enables the global interrupts
}

//Function contains our alternate path which will be used in the case where obstacle is immovable
//Note that this path is still in beta, therefore user caution is advised
void addPath(){
	
	
	updateLCD();
	
	right();
	_delay_ms(630);
	updateLCD();
	
	forward();
	alternatePathDelaytTime = 2;
	for(int i = 0; i < alternatePathDelaytTime*4; i++){
		updateLCD();
		runTheRobot(i, 1, 1000);
	}
	updateLCD();
	
	left();
	_delay_ms(630);
	updateLCD();
	
	forward();
	alternatePathDelaytTime = 2;
	for(int i = 0; i < alternatePathDelaytTime*4; i++){
		updateLCD();
		runTheRobot(i, 1, 1000);
	}
	updateLCD();
	
	left();
	_delay_ms(630);
	updateLCD();
	
	forward();
	alternatePathDelaytTime = 2;
	for(int i = 0; i < alternatePathDelaytTime*4; i++){
		updateLCD();
		runTheRobot(i, 1, 1000);
	}
	updateLCD();
	
	right();
	_delay_ms(630);
	updateLCD();
	
}

void runTheRobot(int i, int insideAlternatePath, int servoPositionCounter){

	if(value <= 200 && value >=1){
				stop();
				servo_2_free();
				stopped = 1;
				i--;
				//buzzer_on();
				runThisCode = 1;
				while(runThisCode){
					updateLCD();
					if(value > 200){
						runThisCode = 0;
					}
					_delay_ms(250);
					if(!insideAlternatePath){
						timerCount++;
						if(timerCount > 16){
							addPath();
							delayTime -= 2;
							runThisCode = 0;		
						}
					}										
				}
			}else if(stopped || value > 200 || value == 0){
				if(value > 200 || value == 0){
					stopped = 0;
					forward();
					if(servoPositionCounter != 1000)
						servo_2(servoPositionCounter);
					//Experimental Section------------------------//
					if(servoPositionCounter == 90){
						if(servoCounter1 != 80){
							servoPositionCounter = 90;
							servo_2(servoPositionCounter);
						}						
						servoCounter1++;
						if(servoCounter1 == 81){
							servoCounter1 = 0;
						}	
					}
					if(servoPositionCounter == 180){
						if(servoCounter1 != 40){
							servoPositionCounter = 180;
							servo_2(servoPositionCounter);
						}
						servoCounter1++;
						if(servoCounter1 == 41){
							servoCounter1 = 0;
						}
					}
					if(servoPositionCounter == 0){
						if(servoCounter1 != 40){
							servoPositionCounter = 0;
							servo_2(servoPositionCounter);
						}
						servoCounter1++;
						if(servoCounter1 == 41){
							servoCounter1 = 0;
						}
					}
					//---------------------------------------------//
					timerCount = 0;
					//buzzer_off();
				}
				_delay_ms(25);
			}
}

//Main Function
int main(){

	//initialization
	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	int checkServoPath = 1;
		
	while(1){//infinite while loop

		velocity(255,251);
		
		updateLCD();
		
		forward(); //both wheels forward
		delayTime = 15;
		for(int i = 0; i < delayTime*40; i++){
			updateLCD();
			runTheRobot(i, 0, servoPositionCounter);			
			if(checkServoPath == 1){
				servoPositionCounter++;
				if(servoPositionCounter == 180)
					checkServoPath = 0;
			}else {
				servoPositionCounter--;
				if(servoPositionCounter == 0)
					checkServoPath = 1;
			}			
		}
		
		right();
		_delay_ms(630);
		
		/*delayTime = 4;
		for(int i = 0; i < delayTime*4; i++){
			updateLCD();
			runTheRobot(i);
		}
		
		right();
		_delay_ms(630);*/
		
	}
}
