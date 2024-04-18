/*************************************************************
*       at328-0.cpp - Demonstrate simple I/O functions of ATmega328
*
*       Program loops turning PC0 on and off as fast as possible.
*
* The program should generate code in the loop consisting of
*   LOOP:   SBI  PORTC,0        (2 cycles)
*           CBI  PORTC,0        (2 cycles)
*           RJMP LOOP           (2 cycles)
*
* PC0 will be low for 4 / XTAL freq
* PC0 will be high for 2 / XTAL freq
* A 9.8304MHz clock gives a loop period of about 600 nanoseconds.
*
* Revision History
* Date     Author      Description
* 09/14/12 A. Weber    Initial Release
* 11/18/13 A. Weber    Renamed for ATmega328P
*************************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "lcd.h"
#include "uart.h"
#include "motors.h"
#include "coordinates.h"


// Calculate the register value for the USART baud rate
#define FOSC 7372800           // Clock frequency
#define BAUD 9600               // Baud rate used
#define MYUBRR FOSC/16/BAUD-1   // Value for UBRR0 register
#define SERVO_RIGHT 4
#define SERVO_MIDDLE 10
#define SERVO_LEFT 18

//STATES
enum states { START, SELECT, GO_DESTINATION };

//BUTTONS
enum buttons { RED, WHITE, BLUE };

// destinations
enum destinations { EEB, OHE, GFS, BOOKSTORE, TCC, VHE};

void timer0_init();
void timer1_init();
void debounce(uint8_t, uint8_t);
void updateSelectionLeft();
void updateSelectionRight();

volatile uint8_t got_time = 0;
volatile uint8_t running = 0;
volatile uint16_t range_count;
volatile char first_time = 1;
volatile char curr_selection = 0;
volatile char driving = 0;

#define OUTBUFSIZE 17
char outbuf[OUTBUFSIZE];        // buffer for outputting messages

volatile Location targetDestination;
volatile Location oldLocation = {0,0};
volatile Location currentLocation;


int main(void)
{
  lcd_init();                 // Initialize lcd
  serial_init(MYUBRR);        // Initialize the serial port
  sei();                      // Enable interrupts

  lcd_moveto(0,0);
  lcd_stringout("HELLO ");
  lcd_moveto(1,0);
  lcd_stringout("MY NAME IS GRAWP");
  _delay_ms(1000);
  lcd_writecommand(1);
  _delay_ms(1000);

  // initialize MUX
  DDRD |= (1 << PD7);
  //Output a 0 for GPS usage
  PORTD &= ~(1 << PD7);


  // initialize Servo
  DDRD |= (1 << PD6);
  timer0_init();
  timer1_init();

  // initialize rangefinder
  PCICR |= (1 << PCIE2); // enable PORT D interrupts
  PCMSK2 |= (1 << PCINT18);
  DDRC |= (1 << PC5);
  uint32_t local_mm = 0;  // Distance calculated in mm
  uint32_t local_int;    // Distance integer part
  uint8_t local_frac;    // Distance fractional part

  //Buttons Init, Enable pull-up resistors
  PORTD |= (1 << PD4); //Button press sends logical 0 ... White Button
  PORTB |= (1 << PB7); //Red Button
  PORTD |= (1 << PD5); //Blue Button

  //For the Red LED
  DDRC |= (1 << PC0);

  //initialize H-bridge
  DDRC |= (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4); 
  PORTC &= ~(  (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4)  ); //Turn off motors

  // H bridge enable initialization
  DDRD |= (1 << PD3);
  timer2_init();

  //Initialize State
  int state = START;

  int motor_flag = 1;

  // initialize locations
  Location locations[6] = {
        {1940, 4192}, //EEB
        {2576, 3783}, //OHE
        {2897, 2729}, //GFS
        {2519, 1828}, //Bookstore
        {2393, 1544}, //TCC
        {2031, 2798}  //VHE
    };

    //Used to turn float latitude and longitude into strings for lcd display
    char buffer[32];

  
    while (1) {
      //STATE MACHINE --------------------------------------------------------------------------------------------------
      if(state == START){
        if(first_time) {
            lcd_writecommand(1);
            lcd_stringout("Press any button");
            lcd_moveto(1,0);
            lcd_stringout("to begin");
            first_time = 0;
        }

        //State Transitions
        if((PIND & (1 << PD5)) == 0){ //blue press 
            state = SELECT;
            debounce(PD5, BLUE);
            first_time = 1;
        }
        else if( (PINB & (1 << PB7)) == 0){ //red press
            state = SELECT;
            debounce(PB7, RED);
            first_time = 1;
        }
        else if((PIND & (1 << PD4)) == 0) { // white press
            state = SELECT;
            debounce(PD4, WHITE);
            first_time = 1;
        }
      }

      else if(state == SELECT){
        if(first_time) {
            lcd_writecommand(1);
            lcd_stringout(">EEB  OHE  GFS");
            lcd_moveto(1,0);
            lcd_stringout(" BST  TCC  VHE");
            first_time = 0;
            curr_selection = EEB;
        }

        //State Transitions
        if( (PIND & (1 << PD5)) == 0){ //blue press 
            updateSelectionRight();
            debounce(PD5, BLUE);
        }
        else if( (PINB & (1 << PB7)) == 0){ //red press
            updateSelectionLeft();
            debounce(PB7, RED);
        }
        else if( (PIND & (1 << PD4)) == 0){ //white press
            targetDestination.latitude = locations[curr_selection].latitude;
            targetDestination.longitude = locations[curr_selection].longitude;
            state = GO_DESTINATION;
            debounce(PD4, WHITE);
            first_time = 1;
        }

      }

      else if(state == GO_DESTINATION){
        if(!driving) {
            lcd_writecommand(1);
            if(first_time) {
                snprintf(buffer, 16, "Target Lat: %4d", targetDestination.latitude);
                lcd_moveto(0,0);
                lcd_stringout(buffer);
                lcd_moveto(1,0);
                snprintf(buffer, 16, "Target Lon: %4d", targetDestination.longitude);
                lcd_stringout(buffer);
                first_time = 0;
                _delay_ms(500);
            }
            int sats = check_sats();
            lcd_moveto(0,0);
            lcd_stringout("waiting for sats");
            if(sats > 0) {
                lcd_moveto(1,0);
                driving = 1;
                snprintf(buffer, 16, "SATS: %d", sats);
                lcd_stringout(buffer);
            }
        }
        else {
            currentLocation = get_coord();
            if(currentLocation.latitude != 0) {
                if((currentLocation.latitude != oldLocation.latitude) || (currentLocation.longitude != currentLocation.longitude)) {
                    snprintf(buffer, 16, "Curr Lat: %4d", currentLocation.latitude);
                    lcd_moveto(0,0);
                    lcd_stringout(buffer);
                    lcd_moveto(1,0);
                    snprintf(buffer, 16, "Curr Lon: %4d", currentLocation.longitude);
                    lcd_stringout(buffer);
                    oldLocation.latitude = currentLocation.latitude;
                    oldLocation.longitude = currentLocation.longitude;
                }
            }
        }
        
      }
      //----------------------------------------------------------------------------------------------------------------
      
    }

    return 0;   // Never reached
}

// for servo
void timer0_init() {
    // Set Timer0 to Fast PWM mode
    TCCR0A = (1 << WGM01) | (1 << WGM00);
    // Clear OC0A on Compare Match, set OC0A at BOTTOM (non-inverting mode)
    TCCR0A |= (1 << COM0A1);
    // Set prescaler to 1024
    TCCR0B = (1 << CS02) | (1 << CS00);
    // Enable Fast PWM mode with max value of 255
    OCR0A = SERVO_MIDDLE; // Start servo in the middle
}

// for rangefinder
void timer1_init()
{
    TCCR1B |= (1 << WGM12);     // Set for CTC mode using OCR1A for the modulus
    TIMSK1 |= (1 << OCIE1A);    // Enable CTC interrupt
    // 7,372,800 / 8 *  0.0232 = 21,381
    OCR1A = 21381 - 1;          // Set counter TOP value for 23.2ms delay
}


// rangefinder ISR
ISR(PCINT2_vect)
{
    uint8_t rout;

    if (running) {
      rout = PIND & (1 << PD2);   // Read the output from the sensor

      if (rout != 0) {            //  Measurement has started
          TCNT1 = 0;              // Make timer1 start on zero count
          TCCR1B |= (0b010 << CS10); // Set prescaler for /8, starts timer
          got_time = 0;	 // Clear flag for valid time value
      }
      else {			 // Only check for stop if counter running
          TCCR1B &= ~(0b111 << CS10); // Set prescaler for none, stops timer1
          got_time = 1;	  // Set flag that time value is valid
          range_count = TCNT1;  // Get the timer count
          running = 0;
      }
    }
}

// rangefinder timer
ISR(TIMER1_COMPA_vect)
{
    TCCR1B &= ~(0b111 << CS10); // Set prescalar for none, stops timer1
    got_time = 1;
    range_count = 0;
    running = 0;
}

// for H bridge
void timer2_init() {
    // Set Timer0 to Fast PWM mode
    TCCR2A = (1 << WGM21) | (1 << WGM20);
    // Clear OC0A on Compare Match, set OC0A at BOTTOM (non-inverting mode)
    TCCR2A |= (1 << COM2B1);
    // Set prescaler to 8
    TCCR2B = ((1 << CS22) | (1 << CS21) | (1 << CS20));
    // Enable Fast PWM mode with max value of 255
    OCR2B = 212; 
}

void debounce(uint8_t bit, uint8_t button){
    _delay_ms(5);
    if(button == WHITE || button == BLUE) {
        while((PIND & (1 << bit)) == 0);
    }
    else if(button == RED) {
        while((PINB & (1 << bit)) == 0);
    }
    _delay_ms(5);
}

void updateSelectionLeft() {
    if(curr_selection != EEB) {
        char column = curr_selection%3;
        char row = curr_selection/3;
        lcd_moveto(row, column*5);
        lcd_stringout(" ");
        curr_selection--;
        column = curr_selection%3;
        row = curr_selection/3;
        lcd_moveto(row, column*5);
        lcd_stringout(">");
        lcd_moveto(1,15);
    } 
}

void updateSelectionRight() {
    if(curr_selection != VHE) {
        char column = curr_selection%3;
        char row = curr_selection/3;
        lcd_moveto(row, column*5);
        lcd_stringout(" ");
        curr_selection++;
        column = curr_selection%3;
        row = curr_selection/3;
        lcd_moveto(row, column*5);
        lcd_stringout(">");
        lcd_moveto(1,15);
    } 
}
