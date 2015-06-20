#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.h"

void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
void SetTunings();

signed int PID(signed int position);


unsigned char data; //to store received data from UDR1

unsigned int flag = 0;
unsigned int data_received [7];
unsigned int sensor_value[7];

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;

signed int senser_value_sum;
signed int reading;
signed int value_on_line;
signed int position,last_proportional,setpoint=0;
signed int speed_L,speed_R;
signed int proportional,avg_senser;
signed int correction,pid,weight;

float Kp=3, Ki=0 ,Kd=1 ,integral,derivative ;

void spi_pin_config (void)
{
	DDRB = DDRB | 0x07;
	PORTB = PORTB | 0x07;
}

//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00; 
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();	adc_pin_config();
	motion_pin_config();
	spi_pin_config();	
}

//Function To Initialize SPI bus
// clock rate: 921600hz
void spi_init(void)
{
	SPCR = 0x53; //setup SPI
	SPSR = 0x00; //setup SPI
	SPDR = 0x00;
}

/*
  //Function Name -  spi_master_tx_and_rx
  //Function to send byte to the slave microcontroller and get ADC channel data from the slave microcontroller
*/
unsigned char spi_master_tx_and_rx (unsigned char data)
{
	unsigned char rx_data = 0;

	PORTB = PORTB & 0xFE; // make SS pin low
	SPDR = data;
	while(!(SPSR & (1<<SPIF))); //wait for data transmission to complete

	_delay_ms(1); //time for ADC conversion in the slave microcontroller
	
	SPDR = 0x50; // send dummy byte to read back data from the slave microcontroller
	while(!(SPSR & (1<<SPIF))); //wait for data reception to complete
	rx_data = SPDR;
	PORTB = PORTB | 0x01; // make SS high
	return rx_data;
}
/*
  //Function Name -  Timer 5
  //Timer 5 initialized in PWM mode for velocity control
  // Prescale:256
  // PWM 8bit fast, TOP=0x00FF
  // Timer Frequency:225.000Hz
*/
void timer5_init()
{
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

/*
  //Function Name -  adc_init
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/*
  //Function Name -  ADC_Conversion
  //Logic - convert sensor values to digital
*/
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
 		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

/*
  //Function Name -  print_sensor
  //Logic - print values on desired row and column
*/
int print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
	
	return ADC_Value ;
}

/*
  //Function Name -  velocity
  //Logic - control velocity of motors
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

/*
  //Function Name -  motion set
  //Logic - assign motion direction
*/
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}

void forward (void) 
{
	motion_set (0x06);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void stop (void)
{
  motion_set (0x00);
}

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}


SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable

	UDR0 = data; 				//echo data back to PC

	if(data == 0x50) //ASCII value of P
	{
		Kp=Kp+0.1;
	}

	if(data == 0x70) //ASCII value of p
	{
		Kp=Kp-0.1;
	}

	if(data == 0x49) //ASCII value of I
	{
		Ki=Ki+0.1;
	}

	if(data == 0x69) //ASCII value of i
	{
		Ki=Ki-0.1;
	}

	if(data == 0x44) //ASCII value of D
	{
		Kd=Kd+0.1;
	}

	if(data == 0x64) //ASCII value of d
	{
		Kd=Kd-0.1;
	}
}

/*
  //Function Name -  init_devices
  //Logic - intitialization
*/
void init_devices (void)
{
 	cli(); //Clears the global interrupts
    spi_init();
	port_init();
	adc_init();
	timer5_init();
	uart0_init(); //Initailize UART1 for serial communiaction
	sei();   //Enables the global interrupts
}
/*
  //Function Name -  sensor_on_line
  //Input - semsor raw values
  //Output - 0 or 1
  //Logic - compare with threshold value of white line if it is online then return 1 else 0
*/
/*sensor_on_line(int sensor)
{
	reading = sensor/10;
	reading = 10*reading;
	return 	reading;
}*/

/*
  //Function Name -  PID
  //Input - error corresponding to sensor which is on line
  //Output - correction value which will be added in left and right motor speed
  //Logic - calculate P,I and D errors individualy and add them.
*/
signed int PID(signed int position)
{
	
	proportional = position - setpoint; // The "proportional" term should be 0 when we are on the white line.
	integral += proportional;  // Compute the integral (sum) of the position using proportional error.
	derivative = proportional - last_proportional; //compute derivative using past and present proportional value.
	
	last_proportional = proportional; // Remember the last position.
	
	//lcd_print(2,1,proportional,3);
	//lcd_print(2,5,integral,3);
	//lcd_print(2,9,derivative,3);
	
	correction = proportional*Kp + integral*Ki + derivative*Kd ;
	
	return correction ;
	
}

/*
  //Function Name -  SetTunings()
  //Input - Nothing
  //Output - set Kp,Ki and Kd values for further use.
  //Logic - assign Kp, ki and Kd values manually.
*/
void SetTunings()
{
	lcd_print(1,1,10*Kp,2);
	lcd_print(1,4,10*Ki,2);
	lcd_print(1,7,10*Kd,2);
}

/*
  //Function Name -  Main
  //Input - Nothing
  //Output - control left and right Motor speeds
  //Logic - add pid correction error here.
*/
int main()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	signed int max = 250 ; 
	speed_L = 255;
	speed_R = 255;
	
	while(1)
	{

		data_received [0] = ADC_Conversion(3);	//Getting data of sensor-0 WL Sensor
		data_received [1] = ADC_Conversion(2);	//Getting data of sensor-1 WL Sensor
		data_received [2] = ADC_Conversion(1);	//Getting data of sensor-2 WL Sensor
        data_received [3] = spi_master_tx_and_rx(0); //Getting data of sensor-3 WL sensor connected to slave microcontroller.
        data_received [4] = spi_master_tx_and_rx(1); //Getting data of sensor-4 WL sensor connected to slave microcontroller.
        data_received [5] = spi_master_tx_and_rx(2); //Getting data of sensor-5 WL sensor connected to slave microcontroller.
		data_received [6] = spi_master_tx_and_rx(3); //Getting data of sensor-6 WL sensor connected to slave microcontroller.
		
		/*lcd_print(1, 1,data_received [0], 1);
		lcd_print(1, 3,data_received [1], 1);
		lcd_print(1, 5,data_received [2], 1);
		lcd_print(1, 7,data_received [3], 1);
		lcd_print(1, 9,data_received [4], 1);
		lcd_print(1, 11,data_received [5], 1);
		lcd_print(1, 13,data_received [6], 1);
        */
		SetTunings();
		/*
		sensor_value[0] = sensor_on_line(data_received [0]);
		sensor_value[1] = sensor_on_line(data_received [1]);
		sensor_value[2] = sensor_on_line(data_received [2]);
		sensor_value[3] = sensor_on_line(data_received [3]);
		sensor_value[4] = sensor_on_line(data_received [4]);
		sensor_value[5] = sensor_on_line(data_received [5]);
		sensor_value[6] = sensor_on_line(data_received [6]);
		*/
		
		senser_value_sum = data_received [0] + data_received [1] + data_received [2] + data_received [3] + data_received [4] + data_received [5] + data_received [6] ;
		
		weight = ((-3)*data_received [0] + (-2)*data_received [1] + (-1)*data_received [2] + (0)*data_received [3] + (1)*data_received [4] + (2)*data_received [5] + (3)*data_received [6]);
		
		
		/*senser_value_sum = sensor_value[0] + sensor_value[1] + sensor_value[2] + sensor_value[3] + sensor_value[4] + sensor_value[5] + sensor_value[6] ;
		
		weight = 10*((-3)*sensor_value[0] + (-2)*sensor_value[1]+ (-1)*sensor_value[2] + (0)*sensor_value[3] + (1)*sensor_value[4] + (2)*sensor_value[5] + (3)*sensor_value[6]);
		*/
		//control variable
		
		value_on_line = weight/10 ;
		
		lcd_print(1, 14,500-value_on_line, 3);
		
		/*lcd_print(1, 1,sensor_value[0], 1);
		lcd_print(1, 3,sensor_value[1], 1);
		lcd_print(1, 5,sensor_value[2], 1);
		lcd_print(1, 7,sensor_value[3], 1);
		lcd_print(1, 9,sensor_value[4], 1);
		lcd_print(1, 11,sensor_value[5], 1);
		lcd_print(1, 13,sensor_value[6], 1);
		*/
		
		pid  =	PID(value_on_line) ;
		//pid = PID(weight); 
		 
		if (pid <= -max)
		{
			pid = -max ;
		}
		
		if (pid >= max)
		{
			pid = max;
		}
		
		if (senser_value_sum>750)
		{
			stop();
		}
		else
		{
			if (pid == 0)
			{
				forward();
				velocity(speed_L,speed_R);
				lcd_print(2,1,speed_L,3);
				lcd_print(2,5,speed_R,3);
				lcd_print(2,10,2000-pid,4);
			}
			
			if(pid>0)
			{
				if(pid < 240)
				{
					forward();
					velocity(speed_L-pid,speed_R);
					lcd_print(2,1,speed_L-pid,3);
					lcd_print(2,5,speed_R,3);
					lcd_print(2,10,2000-pid, 4);
				}
				else
				{
					left();
					velocity(speed_L,speed_R);
					lcd_print(2,1,speed_L,3);
					lcd_print(2,5,speed_R,3);
					lcd_print(2,10,2000-pid, 4);
				}
				
			}
			
			if (pid<0)
			{
				if(pid>-240)
				{
					forward();
					velocity(speed_L,speed_R+pid);
					lcd_print(2,1,speed_L,3);
					lcd_print(2,5,speed_R+pid,3);
					lcd_print(2,10,2000+pid, 4);
				}
				else
				{
					right();
					velocity(speed_L,speed_R);
					lcd_print(2,1,speed_L,3);
					lcd_print(2,5,speed_R,3);
					lcd_print(2,10,2000-pid, 4);
				}
			}
		}					
	}
}
