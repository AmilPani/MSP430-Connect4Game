/*--------------------------------------------------------
GEORGE MASON UNIVERSITY
ECE 447 - Functions to scroll text on an LED Matrix

Reguires global array with 8 unsigned char elements

Date:   Fall 2020
Author: Jens-Peter Kaps

Change Log:
20201004 Initial Version, Jens-Peter Kaps
--------------------------------------------------------*/

#include "fonts.h"

// gets the pixel data from the 5x7 font for character 'c'
// and stores it in an array pointed to by '*matrix' address.
void getnextchar(unsigned char *matrix, char c)
{
    unsigned char col=0;
    unsigned char row=0;
    unsigned char bit=0x01;
    unsigned char pc= c - 0x20;
    while (row<8) {
        while (col <5) {
            if (font_5x7[pc][col] & bit)
                matrix[row] |= 0x01;
            matrix[row]=matrix[row] << 1;
            col++;
        }
        col = 0;
        bit = bit << 1;
        row++;
    }
}

// shifts the array '*matrix' one bit to the left and inserts
// into the rightmost column the left column from the array '*nextchar'
void shiftchar(unsigned char *matrix, unsigned char *nextchar)
{
    unsigned row=0;
    while (row<8) {
        matrix[row] = matrix[row] << 1;
        matrix[row] |= (nextchar[row]&0x20) >> 5;
        nextchar[row] = nextchar[row] << 1;
        row++;
    }
}
