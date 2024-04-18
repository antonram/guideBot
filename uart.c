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

#define SERIAL_START '$'
#define SERIAL_END   '\r'

//void serial_txchar(char);

#define RCVD_BUF_SIZE 100        // Maximum size of temperature data:
                                // +/-, three digits, and '\0'
static volatile unsigned char rcount, recv_full, recv_start;
static volatile char rbuf[RCVD_BUF_SIZE];
static volatile char loopCount = 0;

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

ISR(USART_RX_vect)
{
    // char ch = UDR0;
    // // lcd_moveto(0,0);
    // lcd_writedata(ch);
    // // _delay_ms(1000);

    // // ch = UDR0;			// Get the received charater
    // if (ch == SERIAL_START) {   // First character of string?
	// recv_start = 1;		// Flag that start character received
	// rcount = 0;		// Index for where next character goes in rbuf
	// recv_full = 0;          // Clear flag for rbuf full
    // }
    // else if (recv_start) {
	// if (ch == SERIAL_END) { // End of transmission?
	//     if (rcount > 1) {	// Anything received?/
	// 	rbuf[rcount] = '\0'; // Terminate the string
	// 	recv_full = 1;  // Set flag for data received
	//     }
	//     recv_start = 0;	// Packet complete
	// }
	// else if ((ch == '+' || ch == '-') && rcount == 0) { // Check for +/-
	//     rbuf[rcount++] = ch;      // Put in buffer
	// }
	// else if ((ch >= '0' && ch <= '9')) { // Check for 0-9
	//     if (rcount < RCVD_BUF_SIZE-1)    // Leave room for the '\0'
	// 	rbuf[rcount++] = ch;  // Put in buffer
	//     else
	// 	recv_start = 0;	// Too much data, reset the receiver
	// }
	// else
	//     recv_start = 0;	// Bad data so reset the receiver
    // }

//-------------------------------------------------------------------------------------------------------------
    char ch = UDR0; // Receiving register of microprocessor
    char isGPS = 1; //Used to be0
    if(isGPS) {
        //lcd_writedata(ch);  // For immediate display, might remove to save time

        if (ch == SERIAL_START) {  // Start of new message
            recv_start = 1;
            rcount = 0;
        } else if (recv_start && rcount < RCVD_BUF_SIZE - 1) {
            rbuf[rcount++] = ch;
            if (ch == SERIAL_END) {  // End of transmission?
                rbuf[rcount] = '\0'; // Terminate the string
                recv_full = 1;       // Set flag for data received
                recv_start = 0;      // Ready for next message
            }
        } else {
            recv_start = 0;  // Reset on overflow or other issues
            rcount = 0;
        }
    }
    else{
        //lcd_writecommand(1);
        //lcd_stringout("We are here");
        //_delay_ms(1500);
        if (loopCount == 0 && ch == 'A') {
            lcd_writecommand(1);
            lcd_stringout("Got something!");
            loopCount++;
            recv_start = 1;
            rcount = 0;
        } else if (recv_start && rcount < RCVD_BUF_SIZE - 1) {
            rbuf[rcount++] = ch;
            if(rcount == 14) {
                rbuf[rcount] = '\0'; // Terminate the string
                recv_full = 1;
            }
        } else {
            recv_start = 0;  // Reset on overflow or other issues
            rcount = 0;
            loopCount = 0;
        }
    }
//-------------------------------------------------------------------------------------------



}

// Call this function in your main loop to process received sentences
int check_sats() {
    char local_buffer[RCVD_BUF_SIZE];
    char display_buffer[32]; // Buffer to hold the display string
    int numsats = 0;

    if (recv_full) {
        // lcd_writecommand(1);
        // lcd_stringout("Checking for sats");
        cli();  // Disable interrupts to copy the buffer safely
        strncpy(local_buffer, rbuf, RCVD_BUF_SIZE);  // Copy the buffer for processing
        local_buffer[RCVD_BUF_SIZE - 1] = '\0';  // Ensure null termination
        recv_full = 0;  // Reset the flag
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
    char display_buffer[64]; // Buffer to hold the display string, increased size for latitude and longitude
    Location coordinates = {0,0};

    if (recv_full) {
        // lcd_writecommand(1);
        // lcd_stringout("Checking for coords");
        // _delay_ms(1000);
        cli();  // Disable interrupts to copy the buffer safely
        strncpy(local_buffer, rbuf, RCVD_BUF_SIZE);  // Copy the buffer for processing
        local_buffer[RCVD_BUF_SIZE - 1] = '\0';  // Ensure null termination
        recv_full = 0;  // Reset the flag
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
                // lcd_writecommand(1);
                // sprintf(display_buffer, "Lat: %s, %s", latitude, lat_ns);
                // lcd_stringout(display_buffer);
                float latitude_float = atof(latitude);
                float longitude_float = atof(longitude);
                coordinates.latitude = (int)((latitude_float - (int)latitude_float) * 10000);
                coordinates.longitude = (int)((longitude_float - (int)longitude_float) * 10000);

                // lcd_moveto(1,0);
                // sprintf(display_buffer, "Lon: %s, %s", longitude, lon_ns);
                // lcd_stringout(display_buffer);
                // _delay_ms(2000);
            }
        }
    }
    return coordinates;
}


// // Call this function in your main loop to process received sentences
// void check_vr() {
//     char local_buffer[32];
//     char display_buffer[32]; // Buffer to hold the display string

    
//         lcd_writecommand(1);
//         lcd_stringout("recv_full");
//         cli();  // Disable interrupts to copy the buffer safely
//         strncpy(local_buffer, rbuf, 32);  // Copy the buffer for processing
//         local_buffer[32 - 1] = '\0';  // Ensure null termination
//         recv_full = 0;  // Reset the flag
//         lcd_moveto(1,0);
//         lcd_stringout(local_buffer);
//         sei();  // Enable interrupts again

    
// }