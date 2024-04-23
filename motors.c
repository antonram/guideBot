#include <avr/io.h>
#include <avr/interrupt.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>

#include <stdio.h> // Include this for sprintf
#include "lcd.h"

#include "serial.h"


void motors_off()
{
    PORTC &= ~(  (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4)  ); //Turn off motors
}


void backward()
{
    PORTC |= (1 << PC1);
    PORTC &= ~(1 << PC2);
    PORTC &= ~(1 << PC3);
    PORTC |= (1 << PC4);   
}

void forward()
{
    PORTC &= ~(1 << PC1);
    PORTC |= (1 << PC2);
    PORTC |= (1 << PC3);
    PORTC &= ~(1 << PC4);
}

void right()
{
    PORTC &= ~(1 << PC1);
    PORTC |= (1 << PC2);
    PORTC &= ~(1 << PC3);
    PORTC &= ~(1 << PC4);
    _delay_ms(950);
    motors_off();
}

void left()
{
    PORTC |= (1 << PC1);
    PORTC &= ~(1 << PC2);
    PORTC &= ~(1 << PC3);
    PORTC &= ~(1 << PC4);
    _delay_ms(950);
    motors_off();
}



