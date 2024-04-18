#include <avr/io.h>
#include <avr/interrupt.h>
#include <ctype.h>

#include "serial.h"

#define SERIAL_START '@'
#define SERIAL_END   '#'

void serial_txchar(char);

#define RCVD_BUF_SIZE  5        // Maximum size of temperature data:
                                // +/-, three digits, and '\0'
static volatile unsigned char rcount, recv_full, recv_start;
static volatile char rbuf[RCVD_BUF_SIZE];

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

    DDRC |= (1 << PC4);         // Tri-state buffer control to output
    PORTC &= ~(1 << PC4);       // PC4=0 enables the buffer, RX data comes thru
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
    char ch;

    ch = UDR0;			// Get the received charater
    if (ch == SERIAL_START) {   // First character of string?
	recv_start = 1;		// Flag that start character received
	rcount = 0;		// Index for where next character goes in rbuf
	recv_full = 0;          // Clear flag for rbuf full
    }
    else if (recv_start) {
	if (ch == SERIAL_END) { // End of transmission?
	    if (rcount > 1) {	// Anything received?/
		rbuf[rcount] = '\0'; // Terminate the string
		recv_full = 1;  // Set flag for data received
	    }
	    recv_start = 0;	// Packet complete
	}
	else if ((ch == '+' || ch == '-') && rcount == 0) { // Check for +/-
	    rbuf[rcount++] = ch;      // Put in buffer
	}
	else if ((ch >= '0' && ch <= '9')) { // Check for 0-9
	    if (rcount < RCVD_BUF_SIZE-1)    // Leave room for the '\0'
		rbuf[rcount++] = ch;  // Put in buffer
	    else
		recv_start = 0;	// Too much data, reset the receiver
	}
	else
	    recv_start = 0;	// Bad data so reset the receiver
    }
}
