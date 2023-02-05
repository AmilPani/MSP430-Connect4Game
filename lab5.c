/*--------------------------------------------------------
GEORGE MASON UNIVERSITY
ECE 447 - Lab 5 

Date:   Spring 2021
Authors: Jens-Peter Kaps (Skeleton code), Amilcar Paniagua

Change Log:
20200928 Initial Version, Jens-Peter Kaps
--------------------------------------------------------*/
#include <msp430.h>

unsigned char rowcnt;               // row counter
unsigned char colcnt;               // column counter
unsigned char g_matrix[8];          // content for LED matrix
unsigned char r_matrix[8];          // content for red LED matrix
unsigned char g_row;                // current row of the green LED matrix
unsigned char r_row;                // current row of the red LED matrix

// Welcome messages
unsigned char g_string[50] = "  Welcome Press any        ";
unsigned char r_string[50] = "  Welcome           BUTTON ";

// Winning messages
unsigned char g_win[50] = "Green Wins!   ";
unsigned char r_win[50] = "Red Wins!   ";

unsigned char g_nextchar[8];
unsigned char r_nextchar[8];

unsigned int cordY; // Stores the "Y coordinate" of the LED.
unsigned int cordX; // Stores the "X coordinate" of the LED.

unsigned int lastCordY; // Stores the cordY value before it changes

unsigned int g_turn; // Stores either 1 or 0 to determine green's turn

unsigned char dropped; // Stores the data for the dropping pixel
unsigned char lastDropped; //Stores the dropped value before it changes
unsigned char temp;

unsigned char flash_matrix[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Content for the flashing winning pixels

// States for the state machine
enum states {
    START,
    MOVE,
    DROP,
    FLASH,
    WIN
};
enum states state = START; // Initial state

unsigned int i = 0;                       // all purpose
unsigned int j = 0; // Used as counters for several functions
unsigned int k = 0;

void InitMove(void) {
    state = MOVE;

    // Clear g_matrix and r_matrix
    for(rowcnt=0; rowcnt<8; rowcnt++){
        g_matrix[rowcnt]=0x00;
    }
    for(rowcnt=0; rowcnt<8; rowcnt++){
        r_matrix[rowcnt]=0x00;
    }
    rowcnt = 0;

    cordY = 0;
    g_matrix[cordY] = 0x08; // Set the pixel to be moved
    g_turn = 1; // Green's turn
}

void InitScroll(void) {
    // Clear g_matrix and r_matrix
    for(rowcnt=0; rowcnt<8; rowcnt++){
        g_matrix[rowcnt]=0x00;
    }
    for(rowcnt=0; rowcnt<8; rowcnt++){
        r_matrix[rowcnt]=0x00;
    }
    rowcnt = 0;
    j = 0;
    k = 0;
}

void InitDrop(void) {
    state = DROP;
    dropped = g_matrix[cordY] | r_matrix[cordY]; // Data for pixel dropped either green or red
}

void StopDrop(void) {
    cordY = 0;

    // Setup red's turn
    if (g_turn == 1) {
        r_matrix[cordY] = 0x08;
        dropped = r_matrix[cordY];
    }

    // Setup green's turn
    else {
        g_matrix[cordY] = 0x08;
        dropped = g_matrix[cordY];
    }

    g_turn ^= 1; // Toggle turn
    state = MOVE;
}

void InitFlash(void) {
    state = FLASH;
    k = 0;
}

/*
 * CheckConnect() will check for all winning conditions on the dropped pixel.
 * If a condition is met, the state will change to FLASH.
 */
void CheckConnect(void) {
    unsigned char scan = 0xF0;
    unsigned int countDiag = 1;
    unsigned int tempRow;
    unsigned int tempBool;

    for(rowcnt=0; rowcnt<8; rowcnt++){ // Clear flash_matrix
        flash_matrix[rowcnt]=0x00;
    }
    rowcnt = 0;

    if (g_turn == 0) { // Check green's pixels
        tempBool = 0;
        // Check vertical
        if ((lastCordY < 5) & ((g_matrix[lastCordY+1] & lastDropped) == lastDropped) & ((g_matrix[lastCordY+2] & lastDropped) == lastDropped) & ((g_matrix[lastCordY+3] & lastDropped) == lastDropped)) {
            flash_matrix[lastCordY] = (g_matrix[lastCordY] & lastDropped);
            flash_matrix[lastCordY+1] = (g_matrix[lastCordY+1] & lastDropped);
            flash_matrix[lastCordY+2] = (g_matrix[lastCordY+2] & lastDropped);
            flash_matrix[lastCordY+3] = (g_matrix[lastCordY+3] & lastDropped);
            InitFlash();
            tempBool = 1;
        }

        // Check horizontal
        for (i = 0; i < 5; i++) { // Scans on the current row for 4 consecutive pixels
            if ((g_matrix[lastCordY] & scan) == scan) {
                flash_matrix[lastCordY] = scan;
                break; // When connection is found
            }
            scan = scan >> 1;
        }
        // When connection is found
        if ((flash_matrix[lastCordY] == 0xF0) | (flash_matrix[lastCordY] == 0x78) | (flash_matrix[lastCordY] == 0x3C) | (flash_matrix[lastCordY] == 0x1E) | (flash_matrix[lastCordY] == 0x0F)) {
            InitFlash();
        }

        // Check diagonal
        else if (tempBool == 0) {
            for(rowcnt=0; rowcnt<8; rowcnt++){ // Clear flash_matrix
                flash_matrix[rowcnt]=0x00;
            }
            rowcnt = 0;

            scan = lastDropped << 1;
            tempRow = lastCordY - 1;
            flash_matrix[lastCordY] = lastDropped;
            while ((tempRow > 0) & (scan != 0x00)) { // Scan top left pixels
                if ((g_matrix[tempRow] & scan ) == scan) {
                    flash_matrix[tempRow] |= scan;
                    countDiag++;
                    scan = scan << 1;
                    tempRow--;
                }
                else {
                    break;
                }
            }
            if (countDiag == 4) { // When connection is found
                InitFlash();
            }
            else {
                scan = lastDropped >> 1;
                tempRow = lastCordY + 1;
                while ((tempRow < 8) & (scan != 0x00)) { // Scan bottom right pixels
                    if ((g_matrix[tempRow] & scan ) == scan) {
                        flash_matrix[tempRow] |= scan;
                        countDiag++;
                        scan = scan >> 1;
                        tempRow++;
                    }
                    else {
                        break;
                    }
                }
            }
            if (countDiag == 4) { // When connection is found
                InitFlash();
            }

            else {
                countDiag = 1;
                for(rowcnt=0; rowcnt<8; rowcnt++){ // Clear flash_matrix
                    flash_matrix[rowcnt]=0x00;
                }
                rowcnt = 0;

                scan = lastDropped >> 1;
                tempRow = lastCordY - 1;
                flash_matrix[lastCordY] = lastDropped;
                while ((tempRow > 0) & (scan != 0x00)) { // Scan top right pixels
                    if ((g_matrix[tempRow] & scan ) == scan) {
                        flash_matrix[tempRow] |= scan;
                        countDiag++;
                        scan = scan  >> 1;
                        tempRow--;
                    }
                    else {
                        break;
                    }
                }
                if (countDiag == 4) { // When connection is found
                    InitFlash();
                }
                else {
                    scan = lastDropped << 1;
                    tempRow = lastCordY + 1;
                    while ((tempRow < 8) & (scan != 0x00)) { // Scan bottom left pixels
                        if ((g_matrix[tempRow] & scan ) == scan) {
                            flash_matrix[tempRow] |= scan;
                            countDiag++;
                            scan = scan << 1;
                            tempRow++;
                        }
                        else {
                            break;
                        }
                    }
                }
                if (countDiag == 4) { // When connection is found
                    InitFlash();
                }
            }
        }
    }
    else { // Check red's pixels
        tempBool = 0;
        // Check vertical
        if ((lastCordY < 5) & ((r_matrix[lastCordY+1] & lastDropped) == lastDropped) & ((r_matrix[lastCordY+2] & lastDropped) == lastDropped) & ((r_matrix[lastCordY+3] & lastDropped) == lastDropped)) {
            flash_matrix[lastCordY] = (r_matrix[lastCordY] & lastDropped);
            flash_matrix[lastCordY+1] = (r_matrix[lastCordY+1] & lastDropped);
            flash_matrix[lastCordY+2] = (r_matrix[lastCordY+2] & lastDropped);
            flash_matrix[lastCordY+3] = (r_matrix[lastCordY+3] & lastDropped);
            InitFlash();
            tempBool = 1;
        }

        // Check horizontal
        for (i = 0; i < 5; i++) { // Scans on the current row for 4 consecutive pixels
            if ((r_matrix[lastCordY] & scan) == scan) {
                flash_matrix[lastCordY] = scan;
                break; // When connection is found
            }
            scan = scan >> 1;
        }
        // When connection is found
        if ((flash_matrix[lastCordY] == 0xF0) | (flash_matrix[lastCordY] == 0x78) | (flash_matrix[lastCordY] == 0x3C) | (flash_matrix[lastCordY] == 0x1E) | (flash_matrix[lastCordY] == 0x0F)) {
            InitFlash();
        }

        // Check diagonal
        else if (tempBool == 0) {
            for(rowcnt=0; rowcnt<8; rowcnt++){ // Clear flash_matrix
                flash_matrix[rowcnt]=0x00;
            }
            rowcnt = 0;

            scan = lastDropped << 1;
            tempRow = lastCordY - 1;
            flash_matrix[lastCordY] = lastDropped;
            while ((tempRow > 0) & (scan != 0x00)) { // Scan top left pixels
                if ((r_matrix[tempRow] & scan ) == scan) {
                    flash_matrix[tempRow] |= scan;
                    countDiag++;
                    scan = scan << 1;
                    tempRow--;
                }
                else {
                    break;
                }
            }
            if (countDiag == 4) { // When connection is found
                InitFlash();
            }
            else {
                scan = lastDropped >> 1;
                tempRow = lastCordY + 1;
                while ((tempRow < 8) & (scan != 0x00)) { // Scan bottom right pixels
                    if ((r_matrix[tempRow] & scan ) == scan) {
                        flash_matrix[tempRow] |= scan;
                        countDiag++;
                        scan = scan >> 1;
                        tempRow++;
                    }
                    else {
                        break;
                    }
                }
            }
            if (countDiag == 4) { // When connection is found
                InitFlash();
            }

            else {
                countDiag = 1;
                for(rowcnt=0; rowcnt<8; rowcnt++){ // Clear flash_matrix
                    flash_matrix[rowcnt]=0x00;
                }
                rowcnt = 0;

                scan = lastDropped >> 1;
                tempRow = lastCordY - 1;
                flash_matrix[lastCordY] = lastDropped;
                while ((tempRow > 0) & (scan != 0x00)) { // Scan top right pixels
                    if ((r_matrix[tempRow] & scan ) == scan) {
                        flash_matrix[tempRow] |= scan;
                        countDiag++;
                        scan = scan  >> 1;
                        tempRow--;
                    }
                    else {
                        break;
                    }
                }
                if (countDiag == 4) { // When connection is found
                    InitFlash();
                }
                else {
                    scan = lastDropped << 1;
                    tempRow = lastCordY + 1;
                    while ((tempRow < 8) & (scan != 0x00)) { // Scan bottom left pixels
                        if ((r_matrix[tempRow] & scan ) == scan) {
                            flash_matrix[tempRow] |= scan;
                            countDiag++;
                            scan = scan << 1;
                            tempRow++;
                        }
                        else {
                            break;
                        }
                    }
                }
                if (countDiag == 4) { // When connection is found
                    InitFlash();
                }
            }
        }
    }
}


/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;             // Stop watchdog timer

    // Add code to:
    // connect the ports P2.7, P3.6, and P3.7 to timer
    // B0 CCR6, CCR2, and CCR3 respectively
    // P3.6: SRCLK (column_clk) CCR2
    // P3.7: RCLK (column_done) CCR3
    P2SEL1 &= ~BIT7; // 01b Primary Module
    P2SEL0 |= BIT7; // Connect to CCR6
    P2DIR |= BIT7; // Make P2.7 an output

    P3SEL1 |= (BIT6 | BIT7); // 10b Secondary Module
    P3SEL0 &= ~(BIT6 | BIT7); // Connected to CCR2 and CCR3
    P3DIR |= BIT6 | BIT7; // Make P3.6 and P3.7 outputs

    // Buttons
    P2DIR &= ~(BIT1 | BIT2 | BIT3 |BIT4); // buttons as inputs
    P2REN |= (BIT1 | BIT2 | BIT3 |BIT4);  // enable resistor
    P2OUT |= (BIT1 | BIT2 | BIT3 |BIT4);  // pull up resistor

    // Interrupt configuration for buttons
    P2IES &= ~(BIT1 | BIT2 | BIT3 |BIT4); // P2.1 to 2.4 rising edge
    P2IFG &= ~(BIT1 | BIT2 | BIT3 |BIT4); // P2.1 to 2.4 clear interrupt flag
    P2IE |= (BIT1 | BIT2 | BIT3 |BIT4); // P2.1 to 2.4 enable interrupt


    // Add code to:
    // make the LED Matrix column serial outputs P9.0 and P9.1 outputs
    // and output a 0 on each.
    // P9.0 SER green (row_init)
    // P9.1 SER red (row_init)
    P9DIR |= BIT0 | BIT1; //Make P9.0 and P9.1 outputs
    P9OUT &= ~(BIT0 | BIT1);

    // P1.0 RED LED on board
    P1DIR |= BIT0; //Make P1.0 an output
    P1OUT &= ~BIT0;

    PM5CTL0 &= ~LOCKLPM5;                 // Unlock ports from power manager


    // CCR2 is connected to P3.6 which is column clock.
    // CCR0 active with interrupt, column clock goes low (set/RESET).
    // When CCR2 triggers (no interrupt) column clock goes high (SET/reset).
    // CCR0 and CCR2 are 4 timer clock cycles apart.

    // Add code to:
    // Initialize TB0CCR0 and TB0CCR2 such that CCR0 has an event 4 clock cycles after CCR2
    // Enable interrupts for CCR0
    // Set the outmod for CCR2
    TB0CCR2 = 0x04;
    TB0CCR0 = 0x08;
    TB0CCTL0 = CCIE; // Enable interrupts for CCR0
    TB0CCTL2 = OUTMOD_3; // Set/Reset

    //TB0CCR4 = 0xCCCC;
    TB0CCTL4 = CCIE; // Enable interrupts for CCR4

    // Add code to:
    // Setup and start timer B in continuous mode, ALCK, clk /1, clear timer
    TB0CTL = MC_2 | TBSSEL_1 | ID_0 | TBCLR;

    // Initialize matrix with test pattern
    //i=2;
    for(rowcnt=0; rowcnt<8; rowcnt++){
        g_matrix[rowcnt]=0x00;
        //i=i<<1;
    }
    //i=255;
    for(rowcnt=0; rowcnt<8; rowcnt++){
        r_matrix[rowcnt]=0x00;
        //i=i>>1;
    }

    //getnextchar(g_nextchar, 'P'); // Gets overwritten in while(1) loop
    //getnextchar(r_nextchar, 'P');

    rowcnt = 0;                 // starting row
    colcnt = 0;                 // starting column


    state = START;

    __enable_interrupt();

    while(1)                    // continuous loop
    {
        __low_power_mode_3();

        switch (state) {
        case START:                                                     // Scrolling start text
            // Initialize *nextchar once after 6 shifts
            if (k == 0) {
                getnextchar(g_nextchar, g_string[j]);
                getnextchar(r_nextchar, r_string[j]);

                j++; // Increment index for both green and red strings
                if (j > (strlen(g_string)-1)) j = 0;
            }

            shiftchar(g_matrix, g_nextchar);
            shiftchar(r_matrix, r_nextchar);

            k++;
            if (k > 5) k = 0;

            break;
        case MOVE:                                                      // Left empty

            break;
        case DROP:                                                      // Drop pixel
            if (g_turn == 1) { // Green's turn
                if ((cordY != 7) & ((g_matrix[cordY+1] & dropped) == 0x00) & ((r_matrix[cordY+1] & dropped) == 0x00)) { // Not on last row and next row down is empty
                    // Move down
                    g_matrix[cordY+1] |= dropped;
                    g_matrix[cordY] &= ~(dropped);
                    cordY++;
                }

                else { // When pixel hit last row or next pixel down is occupied
                    lastCordY = cordY;
                    lastDropped = dropped;
                    StopDrop();
                    CheckConnect();
                }
            }

            else if (g_turn == 0) { // Red's turn
                if ((cordY != 7) & ((g_matrix[cordY+1] & dropped) == 0x00) & ((r_matrix[cordY+1] & dropped) == 0x00)) { // Not on last row and next row down is empty
                    // Move down
                    r_matrix[cordY+1] |= dropped;
                    r_matrix[cordY] &= ~(dropped);
                    cordY++;
                }

                else { // When pixel hit last row or next pixel down is occupied
                    lastCordY = cordY;
                    lastDropped = dropped;
                    StopDrop();
                    CheckConnect();
                }
            }
            break;
        case FLASH:                                                     // Flash winner pixels 4 times
            if (g_turn == 0) { // Green wins
                if (k < 8) {
                    for (j = 0; j < 8; j++) { // Update r_matrix
                        r_matrix[j] ^= flash_matrix[j]; //Toggle winning pixels
                    }
                    k++;
                }
                else {
                    InitScroll();
                    for(rowcnt=0; rowcnt<8; rowcnt++){ // Clear flash_matrix
                        flash_matrix[rowcnt]=0x00;
                    }
                    rowcnt = 0;
                    state = WIN;
                }
            }

            else { // Red wins
                if (k < 8) {
                    for (j = 0; j < 8; j++) { // Update g_matrix
                        g_matrix[j] ^= flash_matrix[j]; // Toggle winning pixels
                    }
                    k++;
                }
                else {
                    InitScroll();
                    for(rowcnt=0; rowcnt<8; rowcnt++){ // Clear flash_matrix
                        flash_matrix[rowcnt]=0x00;
                    }
                    rowcnt = 0;
                    state = WIN;
                }
            }
            break;
        case WIN:                                                      // Scrolling winning text
            // Initialize *nextchar once after 6 shifts
            if (k == 0) {
                if (g_turn == 0) {
                    getnextchar(g_nextchar, g_win[j]);
                    j++; // Increment index for green win string
                    if (j > (strlen(g_win)-1)) {
                        InitScroll();
                        state = START;
                        break;
                    }
                }
                else {
                    getnextchar(r_nextchar, r_win[j]);
                    j++; // Increment index for red win string
                    if (j > (strlen(r_win)-1)) {
                        InitScroll();
                        state = START;
                        break;
                    }
                }
            }

            if (g_turn == 0) {
                shiftchar(g_matrix, g_nextchar);
            }
            else {
                shiftchar(r_matrix, r_nextchar);
            }

            k++;
            if (k > 5) k = 0;
            break;
        }

    }

    return 0;
}

// Interrupt Service Routine for Timer B channel CCR0,
// active on falling edge on column clock
#pragma vector = TIMER0_B0_VECTOR   // associate funct. w/ interrupt vector
__interrupt void T0B0_ISR(void)     // name of ISR (can be anything)
{
    // Add code to:
    // output one bit (column) of the green and of the red row
    // and then shift them to move to the next column

    // Green Row
    if(g_row & 0x01)              //   output one bit (column) of current row
        P9OUT |= BIT0;
    else
        P9OUT &= ~BIT0;
    g_row = g_row>>1;              //   move to next column in green row

    // Red Row
    if(r_row & 0x80)              //   output one bit (column) of current row
        P9OUT |= BIT1;
    else
        P9OUT &= ~BIT1;
    r_row = r_row<<1;              //   move to next column in red row

    // Add code to:
    // create timer events for CCR0 and CCR2.
    // both 8 clock cycles from the last one
    TB0CCR2 += 8;
    TB0CCR0 += 8;

    if(colcnt == 7)                 // When on last column of matrix
    {
      // Add code to:
      // create events for column_done (P3.7: RCLK CCR3) and row_init (P9.0 and P9.1 SER CCR6) based upon the
      // specifications in the lab manual

        if (rowcnt == 7) {
            TB0CCTL6 = OUTMOD_1; // Set CCR6 to high on next event
            TB0CCR6 = TB0CCR2; // Next rising edge
        }

        TB0CCTL3 = OUTMOD_1; // Set CCR3 to high on next event
        TB0CCR3 = TB0CCR0; // Next falling edge

      // Add code to:
      // increment the row counter and set the column counter back to 0
        rowcnt++;               //     increment row counter
        if(rowcnt == 8)
            rowcnt = 0;
        colcnt = 0;             //     go to first column

      // Add code to:
      // update the current row for red and green
        g_row = g_matrix[rowcnt];   //     update current green row
        r_row = r_matrix[rowcnt];   //     update current red row

    } else {
      // Add code to:
      // create events for column_done (P3.7: RCLK CCR3) and row_init (P9.0 and P9.1 SER) based upon the

            TB0CCTL3 = OUTMOD_5; // Set CCR3 to low on next event (column done 0)
            TB0CCR3 = TB0CCR2; // Next rising edge
            TB0CCTL6 = OUTMOD_5; // Set CCR6 to low on next event (row init 0)
            TB0CCR6 = TB0CCR2; // Next rising edge

      // Add code to:
      // increment the column counter
        colcnt++;
    }

}

// Timer ISR
#pragma vector = TIMER0_B1_VECTOR
__interrupt void T0B1_ISR(void)
{
    switch(__even_in_range(TB0IV,14))
    {
        case 0: break;
        case 2: break; // CCR1
        case 4: break; // CCR2
        case 6: break; // CCR3
        case 8: //break; // CCR4
            P1OUT ^= BIT0;
            TB0CCR4 += 0x0FFF;

            __low_power_mode_off_on_exit();
            break;
        case 10: //break; // CCR5                   // Debouncing
            P2IE |= (BIT1 | BIT2 | BIT3 |BIT4); // Enable button interrupts
            TB0CCTL5 &= ~CCIE; // Disable debounce timer
            P2IFG &= ~(BIT1 | BIT2 | BIT3 |BIT4); // P2.1 to 2.4 clear interrupt flag
            break;
        case 12: break; // CCR6
        case 14: break; // TAR Overflow
        default: break;
    }
}

// Port 2 ISR
#pragma vector = PORT2_VECTOR
__interrupt void Port_2(void)
{

    switch(__even_in_range(P2IV,P2IV_P2IFG7))
    {
    case P2IV_NONE:   break;   // Vector  0: no interrupt
    case P2IV_P2IFG0: break;   // Vector  2: P2.0
    case P2IV_P2IFG1:          // Vector  4: P2.1 //LEFT
        if (state == START) {
            InitMove();
        }

        else if (state == MOVE) {
            if (g_turn == 1) {
                if (g_matrix[cordY] != 0x80) {
                    g_matrix[cordY] *= 0x02;
                }
            }
            else {
                if (r_matrix[cordY] != 0x80) {
                    r_matrix[cordY] *= 0x02;
                }
            }
        }
        break;
    case P2IV_P2IFG2:          // Vector  6: P2.2 //UP
        if (state == START) {
            InitMove();
        }
        else if (state == MOVE) {
            InitScroll();
            state = START;
        }
        break;
    case P2IV_P2IFG3:          // Vector  8: P2.3 //DOWN
        if (state == START) {
            InitMove();
        }
        else if (state == MOVE) {
            InitDrop();

            if (((g_matrix[cordY+1] & dropped) != 0x00) | ((r_matrix[cordY+1] & dropped) != 0x00)) { // When the stack limit is reached on that column
                state = MOVE;
            }
        }
        break;
    case P2IV_P2IFG4:          // Vector 10: P2.4 //RIGHT
        if (state == START) {
            InitMove();
        }
        else if (state == MOVE) {
            if (g_turn == 1) {
                if (g_matrix[cordY] != 0x01) {
                    g_matrix[cordY] /= 0x02;
                }
            }

            else {
                if (r_matrix[cordY] != 0x01) {
                    r_matrix[cordY] /= 0x02;
                }
            }
        }
        break;
    case P2IV_P2IFG5: break;   // Vector 12: P2.5
    case P2IV_P2IFG6: break;   // Vector 14: P2.6
    case P2IV_P2IFG7: break;   // Vector 16: P2.7
    default: break;

    }

    // Debouncing
    P2IFG &= ~(BIT1 | BIT2 | BIT3 |BIT4); // P2.1 to 2.4 clear interrupt flag
    P2IE &= ~(BIT1 | BIT2 | BIT3 |BIT4); // Disable button interrupts
    TB0CCR5  = TB0R + 0x10FF; // Debounce delay
    TB0CCTL5 |= CCIE; // Enable CCR5

}


