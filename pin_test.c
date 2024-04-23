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
#include <math.h>
#include "lcd.h"
#include "uart.h"
#include "motors.h"
#include "coordinates.h"
#include "corners.h"
#include "gps.h"

// Calculate the register value for the USART baud rate
#define FOSC 7372800                // Clock frequency
#define BAUD 9600                   // Baud rate used
#define MYUBRR FOSC / 16 / BAUD - 1 // Value for UBRR0 register
#define SERVO_RIGHT 4
#define SERVO_MIDDLE 10
#define SERVO_LEFT 18

// STATES
enum states
{
    START,
    SELECT,
    WAIT_FOR_SIGNAL,
    DRIVE
};

// BUTTONS
enum buttons
{
    RED,
    WHITE,
    BLUE
};

// destinations
enum destinations
{
    EEB,
    OHE,
    GFS,
    BOOKSTORE,
    TCC,
    VHE
};

// directions
enum directions
{
    UP,
    RIGHT,
    DOWN,
    LEFT
};

void timer0_init();
void timer1_init();
void timer2_init();
void debounce(uint8_t, uint8_t);
void updateSelectionLeft();
void updateSelectionRight();
void fetchNextCorner();
void fetchFirstCorner();
uint32_t calculateDistance(uint32_t, uint32_t, uint32_t, uint32_t);
void avoid_obstacle();
void go_around_obstacle();

volatile uint8_t got_time = 0;
volatile uint8_t running = 0;
volatile uint16_t range_count;
volatile char first_time = 1;
volatile unsigned char curr_selection = 0;
volatile char driving = 0;
volatile char shutdown_motors = 0;
volatile char first_measurement = 1;

#define OUTBUFSIZE 17
char outbuf[OUTBUFSIZE]; // buffer for outputting messages

volatile Location targetDestination;
volatile Location oldLocation = {0, 0};
volatile Location currentLocation;

// initializing corners, 1 = bottom left, counting up first then to the right.
// structure: upper, right, down, left
volatile Corner first_corner = {1992, 4453, NULL, NULL, NULL, NULL, 0};
volatile Corner second_corner = {2331, 4042, NULL, NULL, NULL, NULL, 0};
volatile Corner third_corner = {2700, 3652, NULL, NULL, NULL, NULL, 0};
volatile Corner fourth_corner = {3254, 3417, NULL, NULL, NULL, NULL, 0};
volatile Corner fifth_corner = {1350, 3222, NULL, NULL, NULL, NULL, 0};
volatile Corner sixth_corner = {1806, 2939, NULL, NULL, NULL, NULL, 0};
volatile Corner seventh_corner = {2231, 2617, NULL, NULL, NULL, NULL, 0};
volatile Corner eighth_corner = {1726, 1494, NULL, NULL, NULL, NULL, 0};
volatile Corner ninth_corner = {2224, 1162, NULL, NULL, NULL, NULL, 0};

volatile Corner eeb_corner = {1940, 4192, NULL, NULL, NULL, NULL, 0};
volatile Corner ohe_corner = {2576, 3783, NULL, NULL, NULL, NULL, 0};
volatile Corner gfs_corner = {2897, 2729, NULL, NULL, NULL, NULL, 0};
volatile Corner bookstore_corner = {2519, 1828, NULL, NULL, NULL, NULL, 0};
volatile Corner tcc_corner = {2393, 1544, NULL, NULL, NULL, NULL, 0};
volatile Corner vhe_corner = {2031, 2798, NULL, NULL, NULL, NULL, 0};
/*
CORNER MAPPINGS:
4th -  -  - 9th
|            |
3rd - 7th - 8th
|      |
2nd - 6th
|      |
1st - 5th
*/

volatile Corner *closest_corner = NULL;
volatile Corner *next_corner;
volatile char current_driving_direction = UP;
volatile char next_driving_direction = UP;
volatile GPS gpsInfo;

#define SQUARE(x) ((x) * (x))

int main(void)
{
    // attach mappings to corners. See map above for reference
    first_corner.upper = &second_corner;
    first_corner.right = &eeb_corner;

    eeb_corner.left = &first_corner;
    eeb_corner.right = &fifth_corner;

    second_corner.upper = &ohe_corner;
    second_corner.right = &sixth_corner;
    second_corner.down = &first_corner;

    ohe_corner.down = &second_corner;
    ohe_corner.upper = &third_corner;

    third_corner.upper = &fourth_corner;
    third_corner.right = &seventh_corner;
    third_corner.down = &ohe_corner;

    fourth_corner.right = &gfs_corner;
    fourth_corner.down = &third_corner;

    gfs_corner.left = &fourth_corner;
    gfs_corner.right = &bookstore_corner;

    bookstore_corner.left = &gfs_corner;
    bookstore_corner.right = &tcc_corner;

    tcc_corner.left = &bookstore_corner;
    tcc_corner.right = &ninth_corner;

    fifth_corner.upper = &sixth_corner;
    fifth_corner.left = &eeb_corner;

    sixth_corner.upper = &vhe_corner;
    sixth_corner.down = &fifth_corner;
    sixth_corner.left = &second_corner;

    vhe_corner.upper = &seventh_corner;
    vhe_corner.down = &sixth_corner;

    seventh_corner.right = &eighth_corner;
    seventh_corner.down = &vhe_corner;
    seventh_corner.left = &third_corner;

    eighth_corner.upper = &ninth_corner;
    eighth_corner.left = &seventh_corner;

    ninth_corner.down = &eighth_corner;
    ninth_corner.left = &tcc_corner;

    lcd_init();          // Initialize lcd
    serial_init(MYUBRR); // Initialize the serial port
    sei();               // Enable interrupts

    lcd_moveto(0, 0);
    lcd_stringout("HELLO ");
    lcd_moveto(1, 0);
    lcd_stringout("MY NAME IS GRAWP");
    _delay_ms(1000);
    lcd_writecommand(1);
    _delay_ms(1000);

    // initialize MUX
    DDRD |= (1 << PD7);
    // Output a 0 for GPS usage
    PORTD &= ~(1 << PD7);

    // initialize Servo
    DDRD |= (1 << PD6);
    timer0_init();
    timer1_init();

    // initialize rangefinder
    PCICR |= (1 << PCIE2); // enable PORT D interrupts
    PCMSK2 |= (1 << PCINT18);
    DDRC |= (1 << PC5);
    uint32_t local_mm = 0; // Distance calculated in mm
    uint32_t local_int;    // Distance integer part

    // Buttons Init, Enable pull-up resistors
    PORTD |= (1 << PD4); // Button press sends logical 0 ... White Button
    PORTB |= (1 << PB7); // Red Button
    PORTD |= (1 << PD5); // Blue Button

    // For the Red LED
    DDRC |= (1 << PC0);

    // initialize H-bridge
    DDRC |= (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4);
    PORTC &= ~((1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4)); // Turn off motors

    // H bridge enable initialization
    DDRD |= (1 << PD3);
    timer2_init();

    // Initialize State
    int state = START;

    // initialize locations
    Location locations[6] = {
        {1940, 4192}, // EEB
        {2576, 3783}, // OHE
        {2897, 2729}, // GFS
        {2519, 1828}, // Bookstore
        {2393, 1544}, // TCC
        {2031, 2798}  // VHE
    };

    // Used to turn float latitude and longitude into strings for lcd display
    char buffer[32];

    while (1)
    {
        // STATE MACHINE --------------------------------------------------------------------------------------------------
        if (state == START)
        {
            if (first_time)
            {
                lcd_writecommand(1);
                lcd_stringout("Press any button");
                lcd_moveto(1, 0);
                lcd_stringout("to begin");
                first_time = 0;
            }

            // State Transitions
            if ((PIND & (1 << PD5)) == 0)
            { // blue press
                state = SELECT;
                debounce(PD5, BLUE);
                first_time = 1;
            }
            else if ((PINB & (1 << PB7)) == 0)
            { // red press
                state = SELECT;
                debounce(PB7, RED);
                first_time = 1;
            }
            else if ((PIND & (1 << PD4)) == 0)
            { // white press
                state = SELECT;
                debounce(PD4, WHITE);
                first_time = 1;
            }
        }

        else if (state == SELECT)
        {
            if (first_time)
            {
                lcd_writecommand(1);
                lcd_stringout(">EEB  OHE  GFS");
                lcd_moveto(1, 0);
                lcd_stringout(" BST  TCC  VHE");
                first_time = 0;
                curr_selection = EEB;
            }

            // State Transitions
            if ((PIND & (1 << PD5)) == 0)
            { // blue press
                updateSelectionRight();
                debounce(PD5, BLUE);
            }
            else if ((PINB & (1 << PB7)) == 0)
            { // red press
                updateSelectionLeft();
                debounce(PB7, RED);
            }
            else if ((PIND & (1 << PD4)) == 0)
            { // white press
                targetDestination.latitude = locations[curr_selection].latitude;
                targetDestination.longitude = locations[curr_selection].longitude;
                state = WAIT_FOR_SIGNAL;
                debounce(PD4, WHITE);
                first_time = 1;
            }
        }

        else if (state == WAIT_FOR_SIGNAL)
        {
            lcd_writecommand(1);
            if (first_time)
            {
                snprintf(buffer, 16, "T Lat: %4ld", targetDestination.latitude);
                lcd_moveto(0, 0);
                lcd_stringout(buffer);
                lcd_moveto(1, 0);
                snprintf(buffer, 16, "T Lon: %4ld", targetDestination.longitude);
                lcd_stringout(buffer);
                first_time = 0;
                _delay_ms(500);
            }
            int sats = check_sats();
            lcd_moveto(0, 0);
            lcd_stringout("waiting for sats");
            if (sats > 0)
            {
                lcd_moveto(1, 0);
                driving = 1;
                snprintf(buffer, 16, "SATS: %d", sats);
                lcd_stringout(buffer);
                state = DRIVE;
                first_time = 1;
                gpsInfo = process_gps_data();
                while (gpsInfo.sats == 0)
                {
                    gpsInfo = process_gps_data();
                }
            }
        }
        else if (state == DRIVE)
        {
            if (first_time)
            {
                first_time = 0;
                snprintf(buffer, 16, "Lat: %4ld", gpsInfo.latitude);
                lcd_moveto(0, 0);
                lcd_stringout(buffer);
                lcd_moveto(1, 0);
                snprintf(buffer, 16, "Lon: %4ld", gpsInfo.longitude);
                lcd_stringout(buffer);
                _delay_ms(8000);
            }
            else
            {
                gpsInfo = process_gps_data();
            }
            if (gpsInfo.sats > 0)
            {
                currentLocation.latitude = gpsInfo.latitude;
                currentLocation.longitude = gpsInfo.longitude;
            }
            if (closest_corner == NULL)
            {
                fetchFirstCorner();
                snprintf(buffer, 16, "Curr: %4ld", closest_corner->latitude);
                lcd_moveto(0, 0);
                lcd_stringout(buffer);
                lcd_moveto(1, 0);
                snprintf(buffer, 16, "Curr: %4ld", closest_corner->longitude);
                lcd_stringout(buffer);
                _delay_ms(8000);
                fetchNextCorner();
                snprintf(buffer, 16, "%ld %ld", next_corner->latitude, next_corner->longitude);
                lcd_writecommand(1);
                lcd_stringout(buffer);
                _delay_ms(8000);
            }
            if (currentLocation.latitude != 0)
            {
                if ((currentLocation.latitude != oldLocation.latitude) || (currentLocation.longitude != currentLocation.longitude))
                {
                    snprintf(buffer, 16, "Curr Lat: %4ld", currentLocation.latitude);
                    lcd_moveto(0, 0);
                    lcd_stringout(buffer);
                    lcd_moveto(1, 0);
                    snprintf(buffer, 16, "Curr Lon: %4ld", currentLocation.longitude);
                    lcd_stringout(buffer);
                    oldLocation.latitude = currentLocation.latitude;
                    oldLocation.longitude = currentLocation.longitude;
                    if (currentLocation.latitude >= targetDestination.latitude - 100 && currentLocation.latitude <= targetDestination.latitude + 100)
                    {
                        if (currentLocation.longitude >= targetDestination.longitude - 100 && currentLocation.longitude <= targetDestination.longitude + 100)
                        {
                            motors_off();
                            lcd_writecommand(1);
                            lcd_stringout("ARRIVED!!!!");
                            _delay_ms(10000);
                            return 0;
                        }
                    }
                }
            }
            if (next_driving_direction != current_driving_direction)
            {
                if (current_driving_direction == UP && next_driving_direction == LEFT) 
                {
                    left();
                    current_driving_direction = LEFT;
                    lcd_writecommand(1);
                    lcd_stringout("LEFT");
                    _delay_ms(2000);
                    lcd_writecommand(1);
                }
                else if (current_driving_direction == LEFT && next_driving_direction == UP) {
                    right();
                    current_driving_direction = UP;
                    lcd_writecommand(1);
                    lcd_stringout("RIGHT");
                    _delay_ms(2000);
                    lcd_writecommand(1);
                }
                while (next_driving_direction < current_driving_direction)
                {
                    left();
                    current_driving_direction--;
                    lcd_writecommand(1);
                    lcd_stringout("LEFT");
                    _delay_ms(2000);
                    lcd_writecommand(1);
                }
                while (next_driving_direction > current_driving_direction)
                {
                    right();
                    current_driving_direction++;
                    lcd_writecommand(1);
                    lcd_stringout("RIGHT");
                    _delay_ms(2000);
                    lcd_writecommand(1);
                }
            }
            else
            {
                if (!shutdown_motors)
                {
                    forward();
                }
                if (currentLocation.latitude >= next_corner->latitude - 75 && currentLocation.latitude <= next_corner->latitude + 75)
                {
                    if (currentLocation.longitude >= next_corner->longitude - 75 && currentLocation.longitude <= next_corner->longitude + 75)
                    {
                        closest_corner = next_corner;
                        closest_corner->visited = 1;
                        lcd_writecommand(1);
                        lcd_stringout("CORNERRR!!!");
                        motors_off();
                        fetchNextCorner();
                    }
                }
                if (first_measurement)
                {
                    while (running);
                    PORTC |= (1 << PC5);
                    _delay_us(10);
                    PORTC &= ~(1 << PC5);
                    first_measurement = 0;
                    running = 1;
                }
                if (!running && got_time)
                {
                    got_time = 0; // Reset flag
                    if (range_count == 0)
                    { // Check for out of range
                        lcd_stringout(">400.0");
                    }
                    else
                    {
                        local_mm = range_count / 5; // Copy to a long
                        local_int = local_mm / 10;
                        if (local_int <= 90 && local_int > 0)
                        {
                            lcd_writecommand(1);
                            lcd_stringout("OBSTACLE");
                            snprintf(buffer, 16, "%ld", local_int);
                            lcd_moveto(0, 9);
                            lcd_stringout(buffer);
                            _delay_ms(100);
                            motors_off();
                            avoid_obstacle();
                        }
                        else
                        {
                            // lcd_writecommand(1);
                            // lcd_stringout("NO OBSTACLE");
                        }
                    }
                    // start new measurement
                    while (running);
                    PORTC |= (1 << PC5);
                    _delay_us(10);
                    PORTC &= ~(1 << PC5);
                    running = 1;
                }
            }
        }

        //----------------------------------------------------------------------------------------------------------------
    }

    return 0; // Never reached
}

// for servo
void timer0_init()
{
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
    TCCR1B |= (1 << WGM12);  // Set for CTC mode using OCR1A for the modulus
    TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt
    // 7,372,800 / 8 *  0.0232 = 21,381
    OCR1A = 21381 - 1; // Set counter TOP value for 23.2ms delay
}

// rangefinder ISR
ISR(PCINT2_vect)
{
    uint8_t rout;

    if (running)
    {
        rout = PIND & (1 << PD2); // Read the output from the sensor

        if (rout != 0)
        {                              //  Measurement has started
            TCNT1 = 0;                 // Make timer1 start on zero count
            TCCR1B |= (0b010 << CS10); // Set prescaler for /8, starts timer
            got_time = 0;              // Clear flag for valid time value
        }
        else
        {                               // Only check for stop if counter running
            TCCR1B &= ~(0b111 << CS10); // Set prescaler for none, stops timer1
            got_time = 1;               // Set flag that time value is valid
            range_count = TCNT1;        // Get the timer count
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
void timer2_init()
{
    // Set Timer0 to Fast PWM mode
    TCCR2A = (1 << WGM21) | (1 << WGM20);
    // Clear OC0A on Compare Match, set OC0A at BOTTOM (non-inverting mode)
    TCCR2A |= (1 << COM2B1);
    // Set prescaler to 8
    TCCR2B = ((1 << CS22) | (1 << CS21) | (1 << CS20));
    // Enable Fast PWM mode with max value of 255
    OCR2B = 235;
}

void debounce(uint8_t bit, uint8_t button)
{
    _delay_ms(5);
    if (button == WHITE || button == BLUE)
    {
        while ((PIND & (1 << bit)) == 0);
    }
    else if (button == RED)
    {
        while ((PINB & (1 << bit)) == 0);
    }
    _delay_ms(5);
}

void updateSelectionLeft()
{
    if (curr_selection != EEB)
    {
        char column = curr_selection % 3;
        char row = curr_selection / 3;
        lcd_moveto(row, column * 5);
        lcd_stringout(" ");
        curr_selection--;
        column = curr_selection % 3;
        row = curr_selection / 3;
        lcd_moveto(row, column * 5);
        lcd_stringout(">");
        lcd_moveto(1, 15);
    }
}

void updateSelectionRight()
{
    if (curr_selection != VHE)
    {
        char column = curr_selection % 3;
        char row = curr_selection / 3;
        lcd_moveto(row, column * 5);
        lcd_stringout(" ");
        curr_selection++;
        column = curr_selection % 3;
        row = curr_selection / 3;
        lcd_moveto(row, column * 5);
        lcd_stringout(">");
        lcd_moveto(1, 15);
    }
}

void fetchNextCorner()
{
    // compare location of adjacent corners with destination
    uint32_t min_distance = 65000;
    uint32_t distance;
    volatile Corner *future_corner;
    if (closest_corner->upper != NULL)
    {
        if (closest_corner->upper->visited == 0)
        {
            distance = calculateDistance(closest_corner->upper->latitude, closest_corner->upper->longitude, targetDestination.latitude, targetDestination.longitude);
            if (distance < min_distance)
            {
                min_distance = distance;
                future_corner = closest_corner->upper;
                next_driving_direction = UP;
            }
        }
    }
    if (closest_corner->right != NULL)
    {
        if (closest_corner->right->visited == 0)
        {
            distance = calculateDistance(closest_corner->right->latitude, closest_corner->right->longitude, targetDestination.latitude, targetDestination.longitude);
            if (distance < min_distance)
            {
                min_distance = distance;
                future_corner = closest_corner->right;
                next_driving_direction = RIGHT;
            }
        }
    }
    if (closest_corner->down != NULL)
    {
        if (closest_corner->down->visited == 0)
        {
            distance = calculateDistance(closest_corner->down->latitude, closest_corner->down->longitude, targetDestination.latitude, targetDestination.longitude);
            if (distance < min_distance)
            {
                min_distance = distance;
                future_corner = closest_corner->down;
                next_driving_direction = DOWN;
            }
        }
    }
    if (closest_corner->left != NULL)
    {
        if (closest_corner->left->visited == 0)
        {
            distance = calculateDistance(closest_corner->left->latitude, closest_corner->left->longitude, targetDestination.latitude, targetDestination.longitude);
            if (distance < min_distance)
            {
                min_distance = distance;
                future_corner = closest_corner->left;
                next_driving_direction = LEFT;
            }
        }
    }
    next_corner = future_corner;
}

void fetchFirstCorner()
{
    uint32_t min_distance = 65000;
    uint32_t distance;
    distance = calculateDistance(currentLocation.latitude, currentLocation.longitude, first_corner.latitude, first_corner.longitude);
    if (distance < min_distance)
    {
        min_distance = distance;
        closest_corner = &first_corner;
    }
    distance = calculateDistance(currentLocation.latitude, currentLocation.longitude, second_corner.latitude, second_corner.longitude);
    if (distance < min_distance)
    {
        min_distance = distance;
        closest_corner = &second_corner;
    }
    distance = calculateDistance(currentLocation.latitude, currentLocation.longitude, third_corner.latitude, third_corner.longitude);
    if (distance < min_distance)
    {
        min_distance = distance;
        closest_corner = &third_corner;
    }
    distance = calculateDistance(currentLocation.latitude, currentLocation.longitude, fourth_corner.latitude, fourth_corner.longitude);
    if (distance < min_distance)
    {
        min_distance = distance;
        closest_corner = &fourth_corner;
    }
    distance = calculateDistance(currentLocation.latitude, currentLocation.longitude, fifth_corner.latitude, fifth_corner.longitude);
    if (distance < min_distance)
    {
        min_distance = distance;
        closest_corner = &fifth_corner;
    }
    distance = calculateDistance(currentLocation.latitude, currentLocation.longitude, sixth_corner.latitude, sixth_corner.longitude);
    if (distance < min_distance)
    {
        min_distance = distance;
        closest_corner = &sixth_corner;
    }
    distance = calculateDistance(currentLocation.latitude, currentLocation.longitude, seventh_corner.latitude, seventh_corner.longitude);
    if (distance < min_distance)
    {
        min_distance = distance;
        closest_corner = &seventh_corner;
    }
    distance = calculateDistance(currentLocation.latitude, currentLocation.longitude, eighth_corner.latitude, eighth_corner.longitude);
    if (distance < min_distance)
    {
        min_distance = distance;
        closest_corner = &eighth_corner;
    }
    distance = calculateDistance(currentLocation.latitude, currentLocation.longitude, ninth_corner.latitude, ninth_corner.longitude);
    if (distance < min_distance)
    {
        min_distance = distance;
        closest_corner = &ninth_corner;
    }
    closest_corner->visited = 1;
}

void avoid_obstacle()
{
    // when we call this, motors are already off
    // wait a second to see if obstacle will move.
    lcd_moveto(1, 0);
    lcd_stringout("AVOIDING");
    _delay_ms(1000);
    // start new measurement
    while (running);
    PORTC |= (1 << PC5);
    _delay_us(10);
    PORTC &= ~(1 << PC5);
    running = 1;
    // while waiting for a measurement to be complete
    while (1)
    {
        uint32_t local_mm;
        uint32_t local_int;
        if (!running && got_time)
        {
            got_time = 0; // Reset flag
            if (range_count == 0)
            { // Check for out of range
                lcd_writecommand(1);
                lcd_stringout(">400.0");
                return;
            }
            else
            {
                local_mm = range_count / 5; // Copy to a long
                local_int = local_mm / 10;
                if (local_int <= 90 && local_int > 0)
                {
                    lcd_writecommand(1);
                    lcd_stringout("MUST GO AROUND");
                    _delay_ms(2000);
                    // if obstacle still in sight, go around
                    go_around_obstacle();
                    return;
                }
                else
                {
                    return;
                }
            }
        }
    }
}

void go_around_obstacle()
{
    // look left
    OCR0A = SERVO_LEFT;
    _delay_ms(2000);
    // start new measurement
    while (running);
    PORTC |= (1 << PC5);
    _delay_us(10);
    PORTC &= ~(1 << PC5);
    running = 1;
    uint32_t dist_left;
    // measure how far object is
    // wait for valid measurement
    while (running || !got_time);
    // process measurement
    got_time = 0; // Reset flag
    if (range_count == 0)
    { // Check for out of range
        lcd_stringout(">400.0");
        dist_left = 400;
    }
    else
    {
        dist_left = range_count / 5; // Copy to a long
        dist_left = dist_left / 10;
    }

    // look right
    OCR0A = SERVO_RIGHT;
    _delay_ms(2000);

    // start new measurement
    while (running);
    PORTC |= (1 << PC5);
    _delay_us(10);
    PORTC &= ~(1 << PC5);
    running = 1;

    uint32_t dist_right;
    // measure how far object is
    // wait for valid measurement
    while (running || !got_time);
    // process measurement
    got_time = 0; // Reset flag
    if (range_count == 0)
    { // Check for out of range
        lcd_stringout(">400.0");
        dist_right = 400;
    }
    else
    {
        dist_right = range_count / 5; // Copy to a long
        dist_right = dist_right / 10;
    }

    if (dist_right > dist_left && dist_left != 0)
    {
        right();
        OCR0A = SERVO_LEFT;
        _delay_ms(2000);
        forward();
        _delay_ms(2000);
        while (1)
        {
            uint32_t dist;
            while (running);
            PORTC |= (1 << PC5);
            _delay_us(10);
            PORTC &= ~(1 << PC5);
            running = 1;
            // wait for valid measurement
            while (running || !got_time);
            // process measurement
            got_time = 0; // Reset flag
            if (range_count == 0)
            { // Check for out of range
                lcd_stringout(">400.0");
                motors_off();
                left();
                OCR0A = SERVO_MIDDLE;
                _delay_ms(2000);
                forward();
                return;
            }
            else
            {
                dist = range_count / 5; // Copy to a long
                dist = dist / 10;
                if (dist > 90 || dist == 0)
                {
                    motors_off();
                    left();
                    OCR0A = SERVO_MIDDLE;
                    _delay_ms(2000);
                    forward();
                    return;
                }
            }
        }
    }
    else
    {
        left();
        OCR0A = SERVO_RIGHT;
        _delay_ms(2000);
        forward();
        _delay_ms(2000);
        while (1)
        {
            uint32_t dist;
            while (running);
            PORTC |= (1 << PC5);
            _delay_us(10);
            PORTC &= ~(1 << PC5);
            running = 1;
            // wait for valid measurement
            while (running || !got_time);
            // process measurement
            got_time = 0; // Reset flag
            if (range_count == 0)
            { // Check for out of range
                lcd_stringout(">400.0");
                motors_off();
                right();
                OCR0A = SERVO_MIDDLE;
                _delay_ms(2000);
                forward();
                return;
            }
            else
            {
                dist = range_count / 5; // Copy to a long
                dist = dist / 10;
                if (dist > 90 || dist == 0)
                {
                    motors_off();
                    right();
                    OCR0A = SERVO_MIDDLE;
                    _delay_ms(2000);
                    forward();
                    return;
                }
            }
        }
    }
}

uint32_t calculateDistance(uint32_t latitude1, uint32_t longitude1, uint32_t latitude2, uint32_t longitude2)
{
    if (latitude2 > latitude1)
    {
        uint32_t latitude3 = latitude2;
        latitude2 = latitude1;
        latitude1 = latitude3;
    }
    if (longitude2 > longitude1)
    {
        uint32_t longitude3 = longitude2;
        longitude2 = longitude1;
        longitude1 = longitude3;
    }
    return sqrt(SQUARE(latitude1 - latitude2) + SQUARE(longitude1 - longitude2));
}