/*
 * RX_v5_0.c
 * For Hardware version 3.10 
 * not compatible with previous hardware version

 * version 5.0 - SENSORS&actuators

 * based on v.4.0 for old hardware
 * Created: Jul.28, 2022 1:11:11 PM
 * Modified: July 2x, 2019  2:22:22 PM
 * Author: dmitry
 */ 
 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
// #include <util/twi.h>
#include <string.h>
#include <util/delay.h>
#include "init_290.h"
#define DEBUG //this turns on the UART***
//#define USE_US //un comment this for the US sensor on P13***
#define BAT_min 108
//Vbatt min~=13.5V (ADC=108)
#define BAT_warn 133
//Vbatt warn~=14V (ADC=133)


// Averaging filter
// If you use an ultrasonic sensor (US) in analog mode, this value MUST be set to 1 as per the sensor's manufacturer application notes. **all our US values are digital so we leave it at 4
// If you wish to filter the values from US sensor, you should use median or mode filter.
// If you use an IR sensor, you can set it to a reasonable value, something between 4 and 10 should work well.
#define ADC_sample_max 4

// Distance threshold for IR sensor. Change it for the US one.
// Note that IR and US sensors have different distance-voltage curves.
// You can use the "ADC_DEBUG" option (see below) to get the corresponding ADC reading.
#define DIST_TH 76
// 1.5/5*255

// **Constants
#define D1_CM 16
#define D2_CM 49
#define IR_C  12 //given by the formula raw= K/d +C, from raw ADC=V*256/3.2 (thats why we define IR_C and IR_K its the two constants of the IR formula derived from the graphs in the datasheets)
#define IR_K  1808
#define T1_PERIOD 5000 // this is just how many timer1 ticks fit in 20ms (since we put ICR1=4999, 5000 ticks fit-> 5000 ticks x 4 micro sec=20ms)
//***************************************************************************************************************
//un-comment the line below to enable printing of ADC readings through the serial port
//#define ADC_DEBUG
//COM port settings: 9600, 8-N-1, None
// Note 1: for this connection, the flow control must be set to "None".
// Note 2: Use any terminal software to display the data (e.g. Hyperterminal)
// Note 3: Arduino IDE must be closed before launching the terminal, 
// and the terminal must be closed before launching Arduino IDE. I.e. only one software should access the serial port.
// Note 4: Since unconnected ADC inputs are floating, the displayed values of unconnected channels are random.
//***************************************************************************************************************

// custom delay function that uses Timer1 IRQ. 20ms increments.
#define DELAY_20ms(x)   TIMSK1&=~(1<<ICIE1);\
                      delay_ms=0;\
                      TIMSK1|=(1<<ICIE1);\
                      while (delay_ms<=x)

#define BLINK(x) PORTB^=(1<<PB3);\
 				_delay_ms(x)

static volatile uint8_t RX_buff, servo_idx, ADC_sample, V_batt;
static volatile uint16_t time, delay_ms, ADC_acc; 
static uint8_t L_ticks, trig_ticks;

volatile struct {
  uint8_t TX_finished:1;
  uint8_t sample:1;
  uint8_t mode:1;
  uint8_t stop:1;
//  uint8_t ADC_ready:1;
  uint8_t T1_ovf0:2;
  uint8_t T1_ovf1:2;
} flags;

extern const uint16_t Servo_angle [256];



ISR(__vector_default) { //capture it all
}

ISR(TIMER1_CAPT_vect){ //system tick: 50Hz, 20ms
  time++;
  delay_ms+=20;
  flags.sample=1; // starts IMU sampling
  flags.T1_ovf0++;
  flags.T1_ovf1++;
}


#ifdef DEBUG
static const char CRLF[3]={13, 10};
static volatile uint8_t *msg, TX_buffer1[10], TX_buffer2[10];

//=============================================================
// UART TX ISR
// it loads UDR as long as the "msg" contains non-zero characters
// once it finds a zero character, it sets TX_finished flag, so a new process can start transmission
ISR (USART_TX_vect) {
  msg++;
  if (*msg) UDR0=*msg;
  else flags.TX_finished=1;
}

//========================================================================
// 
void send_int(int16_t data, uint8_t base, uint8_t crlf){
	while (!flags.TX_finished); //waiting for other transmission to complete
	flags.TX_finished=0;
	itoa(data, (char*) &TX_buffer1[0], base);
	if (crlf) strcat(TX_buffer1, CRLF);
	msg=TX_buffer1;
	UDR0=*msg;
}

void send_reading (int16_t value, char label[], uint8_t crlf) {
	while (!flags.TX_finished); //waiting for other transmission to complete
	flags.TX_finished=0;
	strcpy (TX_buffer1, label);
	itoa(value, &TX_buffer2[0], 10);
	strcat(TX_buffer1, TX_buffer2);
	if (crlf) strcat(TX_buffer1, CRLF); // add CR LF if necesary
	msg=TX_buffer1;
	UDR0=*msg;
}

void send_string (uint8_t *str) {
	while (!flags.TX_finished); //waiting for other transmission to complete
	msg=str;
	UDR0=*msg; 
	flags.TX_finished=0;
}
#endif
//=========================================================================


static volatile struct {
  uint8_t ADC0;
  uint8_t ADC1;
  uint8_t ADC2;
  uint8_t ADC3;
  uint8_t ADC6;
  uint8_t ADC7;
} ADC_data; 

static volatile struct {
  uint16_t pulse0;
  uint16_t pulse1;
//  uint16_t pulse3;
  uint16_t t_start0;
  uint16_t t_end0;
  uint16_t t_start1;
  uint16_t t_end1;
} PULSE_data; 



//=========================================================================
// For ADC readings printing ===================================
#ifdef DEBUG
#define PPS 50
// defines the frequency of printing of ADC readings. 
// Current value (50) corresponds to one set per second. 
// If you set it to 10, it will print 5 sets of readings per second.
// Do not set it too low because you might overload the TX ISR.
static volatile char data_str[]="   ;   ;   .\r\n";
static volatile uint8_t TX_delay=PPS; 
#endif
//=========================================================================


ISR (INT0_vect) {// timing capture for US sensor
	if (PIND&(1<<PD2)) { //raising edge IRQ - start of pulse
		flags.T1_ovf0=0;
		PULSE_data.t_start0=TCNT1; // There is no need to disable global interrupts as they are already disabled while and ISR is being executed.
	}
	if (!(PIND&(1<<PD2))) { //falling edge IRQ - end of pulse
		PULSE_data.t_end0=TCNT1;
		if (flags.T1_ovf0==1) PULSE_data.pulse0=PWM_TOP-PULSE_data.t_start0+PULSE_data.t_end0; //one ovf
			else if (flags.T1_ovf0==2) PULSE_data.pulse0=PWM_TOP-PULSE_data.t_start0+PULSE_data.t_end0+PWM_TOP; //two ovfs
			else if (flags.T1_ovf0==0) PULSE_data.pulse0=PULSE_data.t_end0-PULSE_data.t_start0; //no ovf
			else PULSE_data.pulse0=0xFFFF; // something went wrong		
	}
}
ISR (INT1_vect) { //added for the P13**
  if (PIND&(1<<PD3)) {// rising edge
    flags.T1_ovf1=0;
    PULSE_data.t_start1=TCNT1;
  } else {// falling edge
    PULSE_data.t_end1=TCNT1;
    if (flags.T1_ovf1==0) PULSE_data.pulse1=PULSE_data.t_end1-PULSE_data.t_start1;
    else if (flags.T1_ovf1==1) PULSE_data.pulse1=T1_PERIOD-PULSE_data.t_start1+PULSE_data.t_end1;
    else if (flags.T1_ovf1==2) PULSE_data.pulse1=2*T1_PERIOD-PULSE_data.t_start1+PULSE_data.t_end1; //same as before but add another 5000
    else PULSE_data.pulse1=0xFFFF;
  }
}
// >>>>>>>>>>>>>>>>>>>>>>>>> ADC <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
ISR (ADC_vect){ // the ADC runs on interrupt and populates ADC_data structure.
// You can use the ADC readings in the structure.
//  PORTB^=(1<<PB3); // to check ISR timing
  if (ADC_sample==0) { // scraping the first reading after the channel was changed
    ADC_acc=0; //reset the accumulator
    ADC_sample++;
    return; 
  }
  if (ADC_sample<=ADC_sample_max){ // averaging filter: accumulating readings
    ADC_acc+=ADCH;
    ADC_sample++;
    return;
  }
  if (ADC_sample>ADC_sample_max){
    ADC_sample=0;
	ADC_acc=(ADC_acc/ADC_sample_max); // averaging filter: dividing accumulated readings by the number of readings
    switch (ADMUX&7) {  // checking, which ADC channel was read. NOTE: the ADC multiplexer register is used - no need for a variable to keep track.
      case 0: { // channel 0
        ADMUX=(ADMUX&0xF0)|0x01; //switching to the next channel
// If your next sensor uses different Aref, you can change it here.
//		ADMUX=...?
        ADC_data.ADC0=(uint8_t)(ADC_acc); // storing the filtered reading in ADC_data.
        return; 
      }
      case 1: { 
        ADMUX=(ADMUX&0xF0)|0x02; 
        ADC_data.ADC1=(uint8_t)(ADC_acc);
        return; 
      }
      case 2: { 
        ADMUX=(ADMUX&0xF0)|0x03; 
        ADC_data.ADC2=(uint8_t)(ADC_acc);
        return; 
      }
      case 3: { 
        ADMUX=(ADMUX&0xF0)|0x06; 
        ADC_data.ADC3=(uint8_t)(ADC_acc);
        return; 
      }

      case 6: { 
        ADMUX=(ADMUX&0xF0)|0x07; 
        ADC_data.ADC6=(uint8_t)(ADC_acc);
        return; 
      }

      case 7: { 
        ADMUX=(ADMUX&0xF0)|0x00; 
        ADC_data.ADC7=(uint8_t)(ADC_acc);
        return; 
      }
      default: { //if something goes wrong - switching to Channel 0
        ADMUX=(ADMUX&0xF0)|0x00;    
      }
    }
  }
}
//================================================================================



int main(void)
{
  gpio_init(); // initialise GPIOs

#ifdef DEBUG
  uart_tx_init();
  flags.TX_finished=1;
  uint8_t st3[]="DEBUG mode:\n\r";
  send_string(st3);
#endif


// -------- Timer/PWM init
// ======= PWM2 and D4 control (8-bit timer0) ===================
  timer0_init();

// ======= PWM0 and PWM1 control (16-bit timer1) ===================
  timer1_50Hz_init(1);
TCCR1A |= (1<<WGM11);  //the two lines here make the board select fast pwm, making it jump to 5000 ticks instead of the original 2500 which would have been too small
TCCR1B |= (1<<WGM12);
ICR1 = T1_PERIOD - 1;   //5000 ticks × 4 µs =still 20 ms. 5000-1=4999 ticks
TCCR2A = (1<<COM2A1)|(1<<COM2A0)|(1<<WGM20);  //D3 PWM: inverting (active-low), phase-correct with WGM20
TCCR2B = (1<<CS22); //this is the prescaler 64
DDRB |= (1<<PB5);   //we want the pb5 as an output for L and the US sensor

// ======= Timer2 is not used for PWM ===========


// ADC init
  adc_init (7, 1); // channel 7, ADC IRQ enabled

#ifdef USE_US
  EICRA = (1<<ISC10); //makes the US catch both the end and start of the echo
  EIMSK = (1<<INT1);   //turns INT1 on
#endif
BLINK(500);

  sei(); 

	flags.sample=1;

	while (1) { //infinie loop		

		if (flags.sample) {
			flags.sample=0;

#ifdef ADC_DEBUG
	uint8_t *ADC_ptr=&ADC_data.ADC0;
	for (uint8_t i=0; i<6; i++){
		send_reading (*ADC_ptr, "ADC=", 1);	
		ADC_ptr++;
	} 
	send_string("===========\r\n");
#endif


/*		servo_idx+=3;
			if (servo_idx>=255) servo_idx=0;
			OCR1A=Servo_angle[servo_idx];
*/
      
      // OCR1A=Servo_angle[ADC_data.ADC6];

      // ****Read and convert to cm
      uint16_t raw, dist;
      #ifdef USE_US
        cli(); raw = PULSE_data.pulse1; sei();// 4 µs ticks, cli and sei turns the interrupts off while we measure
        dist = (uint32_t)raw * 2 / 29; // ticks×4 µs/58=cm, came from the US datasheet
      #else
        raw = ADC_data.ADC3;//IR on P16
        dist = (raw > IR_C) ? IR_K / (raw - IR_C) : 999; //if the ADC value is larger than the C constant, the value will give a negative number, so we check if that is the case and return 999 to make sure we know that something went wrong
      #endif

            //**D3 brightness
      uint8_t duty;
      if (dist <= D1_CM) duty = 255; //255 is 100% brightness
      else if (dist >= D2_CM) duty = 0;
      else duty = 255 * (D2_CM - dist) / (D2_CM - D1_CM); //gradually increases or decreases the brightness depending on where the object is between d1 and d2
      OCR2A = duty;
      uint8_t out_of_range = (dist < D1_CM || dist > D2_CM); // this is the line that makes L start blinking if D3 is outstide d1 and d2

      //**L blinks with T = 1.5 s (75 ticks) when out of range
      if (out_of_range) {
        if (++L_ticks >= 75) L_ticks = 0; //start with tick 1
        if (L_ticks < 38) PORTB |= (1<<PB5); else PORTB &= ~(1<<PB5); //when ticks are between 0-37, L is ON and it is off 38-75, it then resets back to 0
      } else { L_ticks = 0; PORTB &= ~(1<<PB5); } //turns the L on or off, else: Inrange so we reset counter and turn L off.


      //**US trigger every 60 ms
      #ifdef USE_US
      if (++trig_ticks >= 3 && !(PORTB & (1<<PB5))) { //3 ticks =60ms (from datasheet, it recommends at least 60ms)
        trig_ticks = 0;
        PORTB |= (1<<PB5); _delay_us(10); PORTB &= ~(1<<PB5); //we need L to be off since it shares the same pb5 with this pin
      }
      #endif

      //**UART output (labels ≤ 3 chars so the 10-byte buffers don't overflow)
      if (--TX_delay == 0) {
        TX_delay = PPS; //counts PPS(50), about 1 print per s
        send_reading(raw, "R=", 0);    //ADC value (IR) or time in 4 µs ticks(US)
        send_reading(dist, " D=", 1);  //distance in cm
      }


			//OCR0A=ADC_data.ADC6;
			//OCR0B=ADC_data.ADC6;

			//if (ADC_data.ADC6>200) PORTD|=(1<<PD4);
			//	else PORTD&=~(1<<PD4);
			//if (ADC_data.ADC6>180) PORTD|=(1<<PD7);
			//	else PORTD&=~(1<<PD7);

			//_delay_ms(10);
			//flags.sample=1;
		
		} //end if-sample

	} // end while(1)

  
} // end main()



//====================== DUMP ===========================
/*
#ifdef DEBUG  
  uint8_t st3[]="ZZZZZZZ:\n\r";
  send_string(st3);
#endif

#ifdef DEBUG  
  uint8_t st1[]="XXXXXXX:\n\r";
  send_string(st1);
#endif
			_delay_ms(1000);
*/
