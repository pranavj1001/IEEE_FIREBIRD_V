/*
 * ServoTrial.c
 *
 * Created: 2/15/2017 10:32:08 AM
 *  Author: Pranav Jain
 */ 

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void){
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}

//Initialize the ports
void port_init(void){
 servo3_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
}

//TIMER1 initialization in 10 bit fast PWM mode  
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz 
void timer1_init(void){
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


//Function to initialize all the peripherals
void init_devices(void){
 cli(); //disable all interrupts
 port_init();
 timer1_init();
 sei(); //re-enable interrupts 
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(int degrees){
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_3_free (void) //makes servo 3 free rotating{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
} 

//Main function
void main(void){
	
 int i = 0;
 init_devices();
 
 while(1){
	 
	 for (i = 0; i < 90; i++)
	 {
	  servo_3(i);
	  _delay_ms(15);
	 }
	 
	 servo_3_free();
	 _delay_ms(2000);
	 
	 for(i = 90; i < 200; i++){
		 servo_3(i);
		 _delay_ms(15);
	 }
	 
	 servo_3_free();
	 _delay_ms(1000);
	 
	 for(i = 199; i > 90; i--){
		 servo_3(i);
		 _delay_ms(15);
	 }
	 
	 servo_3_free();
	 _delay_ms(2000);
	 
	 for(i = 90; i > 0; i--){
		 servo_3(i);
		 _delay_ms(15);
	 }
	 
	 servo_3_free();
	 _delay_ms(1000);
	 
 }
}
