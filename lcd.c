#include <avr/io.h>
#include <util/delay.h>

#include "lcd.h"

#define DATA_BITS ((1 << PB2)|(1 << PB3)|(1 << PB4)|(1 << PB5))
#define CTRL_BITS ((1 << PB1)|(1 << PB0))

/* This function not declared in lcd.h since
   should only be used by the routines in this file. */
void lcd_writenibble(unsigned char);

/*
  lcd_init - Do various things to initialize the LCD display
*/
void lcd_init(void)
{
    DDRB |= DATA_BITS;          // Set the DDR register bits for port B
    DDRB |= CTRL_BITS;

    _delay_ms(15);              // Delay at least 15ms

    lcd_writenibble(0x30);      // Use lcd_writenibble to send 0b0011
    _delay_ms(5);               // Delay at least 4msec

    lcd_writenibble(0x30);      // Use lcd_writenibble to send 0b0011
    _delay_us(120);             // Delay at least 100usec

    lcd_writenibble(0x30);      // Use lcd_writenibble to send 0b0011, no delay needed

    lcd_writenibble(0x20);      // Use lcd_writenibble to send 0b0010
    _delay_ms(2);               // Delay at least 2ms
    
    lcd_writecommand(0x28);     // Function Set: 4-bit interface, 2 lines

    lcd_writecommand(0x0f);     // Display and cursor on
}

/*
  lcd_moveto - Move the cursor to the row and column given by the arguments.
  Row is 0 or 1, column is 0 - 15.
*/
void lcd_moveto(unsigned char row, unsigned char col)
{
    unsigned char pos;
    if(row == 0) {
        pos = 0x80 + col;       // 1st row locations start at 0x80
    }
    else {
        pos = 0xc0 + col;       // 2nd row locations start at 0xc0
    }
    lcd_writecommand(pos);      // Send command
}

/*
  lcd_stringout - Print the contents of the character string "str"
  at the current cursor position.
*/
void lcd_stringout(char *str)
{
    int i = 0;
    while (str[i] != '\0') {    // Loop until next charater is NULL byte
        lcd_writedata(str[i]);  // Send the character
        i++;
    }
}

/*
  lcd_writecommand - Output a byte to the LCD command register.
*/
void lcd_writecommand(unsigned char cmd)
{
    PORTB &= ~(1 << PB0);       // Clear RS for command write
    lcd_writenibble(cmd >> 2);       // Send upper 4 bits
    lcd_writenibble(cmd << 2);  // Send lower 4 bits
    _delay_ms(2);               // Delay 2ms
}

/*
  lcd_writedata - Output a byte to the LCD data register
*/
void lcd_writedata(unsigned char dat)
{
    PORTB |= (1 << PB0);        // Set RS for data write
    lcd_writenibble(dat >> 2);       // Send upper 4 bits
    lcd_writenibble(dat << 2);  // Send lower 4 bits
    _delay_ms(2);               // Delay 2ms
}

/*
  lcd_writenibble - Output upper four bits of "lcdbits" to the LCD
*/
void lcd_writenibble(unsigned char lcdbits)
{
    /* Send upper four bits of the byte "lcdbits" to the LCD */
    PORTB &= ~DATA_BITS;        // Clear PB2-5;
    PORTB |= (lcdbits & DATA_BITS);   // Put high 4 bits of data in PB2-5
    PORTB |= (1 << PB1);        // Set E to 1
    PORTB |= (1 << PB1);        // Set E to 1
    PORTB &= ~(1 << PB1);       // Set E to 0
}