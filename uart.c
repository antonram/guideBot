// #include <avr/io.h>
// #include <util/delay.h>
// #include <avr/interrupt.h>



// void serial_init() {
    
//     uint16_t ubrr = 47; //(7372800 / (16 * 9600)) - 1; // Calculate UBRR value

//     UBRR0 = ubrr;
//     UCSR0B = (1 << TXEN0) | (1 << RXEN0); // Enable transmitter and receiver
    
//     UCSR0C = (3 << UCSZ00); // 8 data bits, no parity, 1 stop bit
//     //UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
// }



#include <avr/io.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>

#include <stdio.h> // Include this for sprintf
#include "lcd.h"

#include "serial.h"
#include "coordinates.h"
#include "gps.h"

#define SERIAL_START '$'
#define SERIAL_END   '\r'

//void serial_txchar(char);

#define RCVD_BUF_SIZE 100        // Maximum size of temperature data:
                                // +/-, three digits, and '\0'
static volatile unsigned char rcount, recv_full, recv_start;
static volatile char rbuf[RCVD_BUF_SIZE];
static volatile char loopCount = 0;

static volatile char rbufGPRMC[RCVD_BUF_SIZE];  // Buffer for GPRMC sentences
static volatile char rbufGPGSA[RCVD_BUF_SIZE];  // Buffer for GPGSA sentences
static volatile unsigned char recv_full_GPRMC = 0;
static volatile unsigned char recv_full_GPGSA = 0;
static char buffer[RCVD_BUF_SIZE]; // Temporary buffer for assembling sentences
static uint8_t buf_pos = 0;
static unsigned char sentence_type = 0; // 0: unknown, 1: GPRMC, 2: GPGSA

void serial_init(unsigned short ubrr_value)
{
    UBRR0 = ubrr_value;         // Set baud rate
    UCSR0C = (3 << UCSZ00);     // Set for asynchronous operation, no parity,
                                // one stop bit, 8 data bits
    UCSR0B |= (1 << TXEN0);     // Turn on transmitter
    UCSR0B |= (1 << RXEN0);     // Turn on receiver

    rcount = 0;                 // Count of characters received
    recv_full = 0;              // Flag for received buffer full
    recv_start = 0;             // Flag for start character received

    UCSR0B |= (1 << RXCIE0);    // Enable receiver interrupts

    DDRD |= (1 << PD1);

    // DDRC |= (1 << PC4);         // Tri-state buffer control to output
    // PORTC &= ~(1 << PC4);       // PC4=0 enables the buffer, RX data comes thru
}

//serial_in - Read a byte from the USART0 and return it
char serial_in(){
    while ( !(UCSR0A & (1 << RXC0)) );
    return UDR0;
 }

void serial_txchar(char ch)
{
    while ((UCSR0A & (1<<UDRE0)) == 0);
    UDR0 = ch;
}

void serial_stringout(char *s)
{
    while (*s != '\0') {
	serial_txchar(*s);
	s++;
    }
}

/*
  recv_string - Returns 1 if data available, 0 otherwise.
  If data available, copy the string to the "rp" pointer.
*/
unsigned char recv_string(char *rp)
{
    unsigned char status;
    char ch, *p;

    cli();
    if (recv_full) {		// See if new data in rbuf
	// Could use strcpy(rp, rbuf);
	p = (char *) rbuf;
	while ((ch = *p++) != '\0')
	    *rp++ = ch;
	*rp = '\0';
	status = 1;             // Return status = 1
	recv_full = 0;	        // Clear flag for rbuf full
    }
    else
	status = 0;             // If nothing, return 0
    sei();
    return(status);
}

ISR(USART_RX_vect) {
    char ch = UDR0; // Read the received character, which also clears the RXC flag

    if (ch == SERIAL_START) {
        buf_pos = 0; // Reset position for new sentence
        sentence_type = 0; // Reset sentence type
    } else if (ch == SERIAL_END) {
        buffer[buf_pos] = '\0'; // Null-terminate the string
        if (sentence_type == 1) { // GPRMC
            strncpy((char*)rbufGPRMC, buffer, RCVD_BUF_SIZE);
            recv_full_GPRMC = 1;
        } else if (sentence_type == 2) { // GPGSA
            strncpy((char*)rbufGPGSA, buffer, RCVD_BUF_SIZE);
            recv_full_GPGSA = 1;
        }
        buf_pos = 0; // Reset for next sentence
        sentence_type = 0; // Reset sentence type
    } else {
        if (buf_pos < RCVD_BUF_SIZE - 1) {
            buffer[buf_pos++] = ch;
        }
        // Determine the type of sentence as soon as possible
        if (buf_pos == 5) {
            if (strncmp(buffer, "GPRMC", 5) == 0) {
                sentence_type = 1;
            } else if (strncmp(buffer, "GPGSA", 5) == 0) {
                sentence_type = 2;
            }
        }
    }
}

// Call this function in your main loop to process received sentences
int check_sats() {
    char local_buffer[RCVD_BUF_SIZE];
    char display_buffer[32]; // Buffer to hold the display string
    int numsats = 0;

    if (recv_full_GPGSA) {
        // lcd_writecommand(1);
        // lcd_stringout("Checking for sats");
        cli();  // Disable interrupts to copy the buffer safely
        strncpy(local_buffer, rbufGPGSA, RCVD_BUF_SIZE);  // Copy the GPGSA buffer for processing
        local_buffer[RCVD_BUF_SIZE - 1] = '\0';  // Ensure null termination
        recv_full_GPGSA = 0;  // Reset the flag
        sei();  // Enable interrupts again

        // Now process the local_buffer to find the number of satellites
        if (strncmp(local_buffer, "GPGSA", 5) == 0) {
            // lcd_writecommand(1);
            //lcd_stringout("GPGSA is here");

            memcpy(display_buffer, local_buffer, 15);
            display_buffer[15] = '\0';

            // lcd_moveto(1,0);
            // lcd_stringout(display_buffer);
            // _delay_ms(2000);


            char *token;
            int field_count = 0;
            token = strtok(local_buffer, ",");
            while (token != NULL) {
                field_count++;
                if (field_count == 8) {  // Number of satellites is in 8th field
                    numsats = atoi(token);
                    // lcd_moveto(1, 0); // Move cursor to the start of the second row
                    sprintf(display_buffer, "Satellites: %2d", numsats);
                    // lcd_stringout(display_buffer);
                    // _delay_ms(2000);
                    break;
                }
                token = strtok(NULL, ",");
            }
        }
    }
    return numsats;
}

// Call this function in your main loop to process received sentences
Location get_coord() {
    char local_buffer[RCVD_BUF_SIZE];
    Location coordinates = {0,0};

    if (recv_full_GPRMC) {
        // lcd_writecommand(1);
        // lcd_stringout("Checking for coords");
        // _delay_ms(1000);
        cli();  // Disable interrupts to copy the buffer safely
        strncpy(local_buffer, rbufGPRMC, RCVD_BUF_SIZE);  // Copy the GPRMC buffer for processing
        local_buffer[RCVD_BUF_SIZE - 1] = '\0';  // Ensure null termination
        recv_full_GPRMC = 0;  // Reset the flag
        sei();  // Enable interrupts again

        if (strncmp(local_buffer, "GPRMC", 5) == 0) {
            //lcd_writecommand(1);
            //lcd_stringout("GPRMC is here");

            char *token;
            int field_count = 0;
            char *latitude = NULL, *longitude = NULL, *lat_ns = NULL, *lon_ns = NULL;

            token = strtok(local_buffer, ",");
            while (token != NULL) {
                field_count++;
                if (field_count == 4) {
                    latitude = token;
                } 
                else if (field_count == 5) {
                    lat_ns = token;
                }
                else if (field_count == 6) {
                    longitude = token;
                }
                else if (field_count == 7) {
                    lon_ns = token;
                    break; // No need to parse further once lon_northsouth is found
                }
                token = strtok(NULL, ",");
            }

            if (latitude && longitude) {
                float latitude_float = atof(latitude);
                float longitude_float = atof(longitude);
                coordinates.latitude = (int)((latitude_float - (int)latitude_float) * 10000);
                coordinates.longitude = (int)((longitude_float - (int)longitude_float) * 10000);
            }
        }
    }
    return coordinates;
}

GPS process_gps_data() {
    GPS returnData = {0, 0, 0};
    if (recv_full_GPRMC && recv_full_GPGSA) {
        Location coords = get_coord(); // Process coordinates from GPRMC
        int sats = check_sats(); // Process satellite count from GPGSA

        returnData.latitude = coords.latitude;
        returnData.longitude = coords.longitude;
        returnData.sats = sats;

        // Clear the flags
        recv_full_GPRMC = 0;
        recv_full_GPGSA = 0;
    }
    return returnData;
}
