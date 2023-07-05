/* Copyright (c) 2023, Edwin Freekenhorst and Henk Stegeman

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   HENK STEGEMAN AND EDWIN FREEKENHORST BE LIABLE FOR ANY CLAIM, DAMAGES OR
   OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
   ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
   DEALINGS IN THE SOFTWARE.
   ---------------------------------------------------------------------------

   3705_panel.c: IBM 3705 Interfaced Operator Panel

   This module emulates several founctions of the 3705 front panel.
   To access the panel connect access port 37050 with a TN3270 emulator

   This module includes an interval timer that tiggers every 100msec a L3 interrupt.
*/

#include <stdio.h>
#include <sched.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <ifaddrs.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include "i3705_defs.h"
#include "i3705_Eregs.h"               /* Exernal regs defs */
#include <ncurses.h>

#define RED_BLACK    1
#define GREEN_BLACK  2
#define YELLOW_BLACK 3
#define WHITE_BLACK  4
#define BLUE_BLACK   5
#define BLACK_RED    6
#define BLACK_GREEN  7
#define BLACK_YELLOW 8
#define BLACK_WHITE  9
#define BLACK_BLACK 10

extern int32 msize;
extern int32 PC;
extern int32 saved_PC;
extern int32 opcode;
extern int32 Eregs_Out[];
extern int32 Eregs_Inp[];
extern int8  timer_req_L3;
extern int8  inter_req_L3;

// CCU status flags
extern int8  test_mode;
extern int8  load_state;
extern int8  wait_state;
extern int8  pgm_stop;

void sig_handler (int signo);
void timer_msec (long int msec);

int rc, inp, i;
int32 hex_sw, rot_sw;

#define MAXPORTS 4

int key = KEY_F0;
int rc, len;                           /* Return code from various rtns */
int8 shwpanel = 0;                     /* Dispaly Front Panel, initially no */
uint8_t wrkbyte;                       /* Work byte */
uint8_t mbyte;
uint16_t freebuf;
uint32_t maddr;
uint32_t old_cucr;

char *ipaddr;
uint8_t buf[8192], ibuf[256];
uint8_t buft[80];
uint8_t hexsw[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
int8_t  hexswpos = 0;
int8_t  fsswpos  = 1;
char fssw[10][17] = {"TAR&OP REGISTER", "STATUS",
                     "FUNCTION 6", "STORAGE ADDRESS",
                     "FUNCTION 5", "REGISTER ADDRESS",
                     "FUNCTION 4", "FUNCTION 1",
                     "FUNCTION 3", "FUNCTION 2"};


extern uint8 M[MAXMEMSIZE];
extern pthread_mutex_t r7f_lock;
extern struct IO3705* iobs[MAXCHAN];
extern int Ireg_bit(int reg, int bit_mask);

int row, col;

extern void wait();

// ******************************************************************
// Function to write coloured text on the front panel at row x, col y
// ******************************************************************

void stringAtXY (int x, int y, char *buf, int colour) {

   wmove(stdscr, x, y);
   attron(COLOR_PAIR(colour));
   //attron(A_BOLD);
   printw("%s",buf);
   attroff(COLOR_PAIR(colour));
   return;
}

// ******************************************************************
// Function to write coloured integer  on the front panel at row x, col y
// ******************************************************************

void integerAtXY (int x, int y, int value, int colour) {

   wmove(stdscr,x,y);
   attron(COLOR_PAIR(colour));
   printw("%d", value);
   attroff(COLOR_PAIR(colour));
   return;
}

// ******************************************************************
// Function to write a nibble on the front panel at row x, col y
// ******************************************************************

void nibbleAtXY (int x, int y, int value, int colour) {

   wmove(stdscr, x, y);
   attron(COLOR_PAIR(colour));
   printw("%01X", value);
   attroff(COLOR_PAIR(colour));
   return;
}

// ******************************************************************
// Function to write a byte on the front panel at row x, col y
// ******************************************************************

void byteAtXY (int x, int y, int value, int colour) {

   wmove(stdscr,x,y);
   attron(COLOR_PAIR(colour));
   printw("%02X", value);
   attroff(COLOR_PAIR(colour));
   return;
}

// **********************************************************
// Function to build the front panel.
// **********************************************************
void FrontPanel() {

    stringAtXY(0, 30, "IBM 3705 Front Panel", RED_BLACK);
    stringAtXY(1,  0, "----------------------------------------", GREEN_BLACK);
    stringAtXY(1, 40, "----------------------------------------", GREEN_BLACK);
    stringAtXY(3, 1,  ".----CCU CHECKS-----.", GREEN_BLACK);
    stringAtXY(3, 24, ".--DISPLAY A--.--DISPLAY B--.", GREEN_BLACK);
    stringAtXY(3, 59, "MEMORY SIZE :", BLUE_BLACK);
    stringAtXY(4, 1,  "|", GREEN_BLACK);
    stringAtXY(4, 3,  "ADAPTER CHECK", BLUE_BLACK);
    stringAtXY(4, 21, "|", GREEN_BLACK);
    stringAtXY(4, 24, "|   |    |    |   |    |    |", GREEN_BLACK);
    stringAtXY(4, 26, " ", BLACK_YELLOW);
    stringAtXY(4, 30, "  ", BLACK_YELLOW);
    stringAtXY(4, 35, "  ", BLACK_YELLOW);
    stringAtXY(4, 40, " ", BLACK_YELLOW);
    stringAtXY(4, 44, "  ", BLACK_YELLOW);
    stringAtXY(4, 49, "  ", BLACK_YELLOW);
    stringAtXY(4, 59, "IPL PHASE   :", BLUE_BLACK);
    stringAtXY(5, 1,  "|", GREEN_BLACK);
    stringAtXY(5, 3,  "I/O CHECK", BLUE_BLACK);
    stringAtXY(5, 21, "|", GREEN_BLACK);
    stringAtXY(5, 24, "'-X----0----1-'-X----0----1-'", GREEN_BLACK);
    stringAtXY(5, 59, "FREE BUFFERS:",BLUE_BLACK);
    stringAtXY(6, 1,  "|", GREEN_BLACK);
    stringAtXY(6, 3,  "ADDRESS EXCEPT", BLUE_BLACK);
    stringAtXY(6, 21, "|", GREEN_BLACK);
    stringAtXY(6, 59, "CYCLCE COUNT:", BLUE_BLACK);
    stringAtXY(7, 1,  "|", GREEN_BLACK);
    stringAtXY(7, 3,  "PROTECT CHECK", BLUE_BLACK);
    stringAtXY(7, 21, "|", GREEN_BLACK);
    stringAtXY(8, 1,  "|", GREEN_BLACK);
    stringAtXY(8, 3,  "INVALID OP",BLUE_BLACK);
    stringAtXY(8, 21, "|", GREEN_BLACK);
    stringAtXY(8, 30, "+ A  B  C  D  E +", GREEN_BLACK);
    stringAtXY(9, 1,  "'-------------------'", GREEN_BLACK);
    stringAtXY(12, 28, "DISPLAY FUNCTION SELECT", BLUE_BLACK);
    stringAtXY(14, 1,  ".-Channel Adapter 1-.", GREEN_BLACK);
    stringAtXY(15, 1,  "| A:", GREEN_BLACK);
    stringAtXY(15, 21, "|", GREEN_BLACK);
    stringAtXY(16, 1,  "| B:", GREEN_BLACK);
    stringAtXY(16, 21, "|", GREEN_BLACK);
    stringAtXY(17, 1,  "|-Channel Adapter 2-|", GREEN_BLACK);
    stringAtXY(18, 1,  "| A:", GREEN_BLACK);
    stringAtXY(18, 21, "|", GREEN_BLACK);
    stringAtXY(19, 1,  "| B:", GREEN_BLACK);
    stringAtXY(19, 21, "|", GREEN_BLACK);
    stringAtXY(20, 1,  "'-------------------'", GREEN_BLACK);
    stringAtXY(22, 1,  "PF1=CA1 A/B  PF2=CA2 A/B  PF3=D/F SELECT  PF7=INTERRUPT", GREEN_BLACK);
    stringAtXY(22,70,  "HOME=exit", GREEN_BLACK);
    stringAtXY(23, 1,  " ", GREEN_BLACK);
    return;
}

// **********************************************************
// The panel adaptor handling thread starts here...
// **********************************************************
/* Function to be run as a thread always must have the same
   signature: it has one void* parameter and returns void    */

void *PNL_thread(void *arg) {
   fprintf(stderr, "PNL: Thread %ld started succesfully... \n\r", syscall(SYS_gettid));

   // We can set one or more bits here, each one representing a single CPU
   //cpu_set_t cpuset;

   // Select the CPU core we want to use
   //int cpu = 2;

   //CPU_ZERO(&cpuset);                  // Clears the cpuset
   //CPU_SET( cpu , &cpuset);            // Set CPU on cpuset

   /*
    * cpu affinity for the calling thread
    * first parameter is the pid, 0 = calling thread
    * second parameter is the size of your cpuset
    * third param is the cpuset in which your thread
    * will be placed. Each bit represents a CPU.
    */
   //sched_setaffinity(0, sizeof(cpuset), &cpuset);


   signal (SIGALRM, sig_handler);      // Interval timer //
   timer_msec(100);                    // <=== sets the 3705 interval timer

   int disp_regA, disp_regB;
   int inp_h, inp_l, count, row, col;
   int five_lt = 0x00;
   int fdstdout;
   while(1) {
      if (shwpanel == 1) {
         old_cucr = Eregs_Inp[0x7A];          // save current cycle counter
         while (Eregs_Inp[0x7A] == old_cucr)  // wait until cycle counter changes
            sleep(1);
         // *******************************************************
         // Build the main screen
         // *******************************************************
         FILE *f = fopen("/dev/tty", "r+");
         freopen("/dev/null", "w", stdout);
         SCREEN *pnlwin = newterm(NULL, f, f);
         set_term(pnlwin);
         refresh();
         curs_set(0);

         if (has_colors() == FALSE) {
            endwin();                         // End window
            printf("\nPNL: No colour suipport for your terminal\r");
            exit(-1);
         }  // End if (has_colors)

         start_color();
         init_color(COLOR_YELLOW, 1000, 1000, 0);
         init_color(COLOR_RED, 1000, 0, 0);
         init_color(COLOR_BLUE, 0, 1000, 1000);
         init_color(COLOR_GREEN, 0, 1000, 0);
         init_pair(RED_BLACK, COLOR_RED, COLOR_BLACK);
         init_pair(GREEN_BLACK, COLOR_GREEN, COLOR_BLACK);
         init_pair(YELLOW_BLACK, COLOR_YELLOW, COLOR_BLACK);
         init_pair(WHITE_BLACK, COLOR_WHITE, COLOR_BLACK);
         init_pair(BLUE_BLACK, COLOR_BLUE, COLOR_BLACK);
         init_pair(BLACK_RED, COLOR_BLACK, COLOR_RED);
         init_pair(BLACK_GREEN, COLOR_BLACK, COLOR_GREEN);
         init_pair(BLACK_YELLOW, COLOR_BLACK, COLOR_YELLOW);
         init_pair(BLACK_WHITE, COLOR_BLACK, COLOR_WHITE);
         init_pair(BLACK_BLACK, COLOR_BLACK, COLOR_BLACK);

         noecho();
         keypad(stdscr, TRUE);
         // Display the front panel
         FrontPanel();
         refresh();

         // Init hex switches
         for (int i = 0; i < 5; i++) {
            if (i == hexswpos)
               nibbleAtXY(10, 32+(i*3), hexsw[i], BLACK_WHITE);
            else
               nibbleAtXY(10, 32+(i*3), hexsw[i], WHITE_BLACK);
         }  // End for int i

         // Init Function Select switch
         for (int i = 0; i < 10; i++) {
            if (i%2 == 0) {
                col = 26;
                row = 13 + (i/2);
            } else {
                col = 44;
                row = 13 + ((i-1) / 2);
            } // End if i%2

            if (i == fsswpos)
               stringAtXY(row, col, fssw[i], BLACK_GREEN);
            else
               stringAtXY(row, col, fssw[i], GREEN_BLACK);
         }  // End for int i
         key = KEY_F0;
         while ((key != KEY_HOME) && (key != 0x007E)) {    /* 0x007E needed for putty  */
            /* Show Memory Size */
            integerAtXY(3, 73, msize, BLUE_BLACK);         /* Show memory size         */
            attron(COLOR_PAIR(BLUE_BLACK));
            printw("K");                                   /* Kilobytes...             */
            /* Pick up IPL Phase */
            wrkbyte = ((Eregs_Out[0x72] & 0x3000) >> 12);  /* Get IPL Phase...         */
            integerAtXY(4, 73, wrkbyte, BLUE_BLACK);       /* ...and display it        */
            /* Pick up free buffer count */
            freebuf = (M[0x0754] << 8) + M[0x0755];        /* get free buffer count... */
            integerAtXY(5, 73, freebuf, BLUE_BLACK);       /* ...and display it        */
            /* Pick up cycle counter */
            count = Eregs_Inp[0x7A] & 0x7FFF;              /* get cycle counter...     */
            integerAtXY(6, 73, count, BLUE_BLACK);         /* ...and display it        */

            /********************************/
            /* check channel adapter states  */
            /********************************/
            /* Channel Adapter 1 */
            if (iobs[0]->abswitch == 0) {
                  stringAtXY(16, 6, "DISABLED", RED_BLACK);
               if (iobs[0]->bus_socket[0] > 0) {
                  stringAtXY(15, 6, "ACTIVE  ", BLACK_YELLOW);
               } else {
                  stringAtXY(15, 6, "ENABLED ", GREEN_BLACK);
               }
            } else {
               stringAtXY(15, 6, "DISABLED", RED_BLACK);
               if (iobs[0]->bus_socket[1] > 0) {
                  stringAtXY(16, 6, "ACTIVE  ", YELLOW_BLACK);
               } else {
                  stringAtXY(16, 6, "ENABLED ", GREEN_BLACK);
               }
            }
            /* Channel Adapter 2 */
            if (iobs[1]->abswitch == 0) {
                  stringAtXY(19, 6, "DISABLED", RED_BLACK);
               if (iobs[1]->bus_socket[0] > 0) {
                  stringAtXY(18, 6, "ACTIVE  ", BLACK_YELLOW);
               } else {
                  stringAtXY(18, 6, "ENABLED ", GREEN_BLACK);
               }
            } else {
               stringAtXY(18, 6, "DISABLED", RED_BLACK);
               if (iobs[1]->bus_socket[1] > 0) {
                  stringAtXY(19, 6, "ACTIVE  ", BLACK_YELLOW);
               } else {
                  stringAtXY(19, 6, "ENABLED ", GREEN_BLACK);
               }
            }

            /********************************/
            /* Wait for a key to be pressed */
            /********************************/
            key = getch();
            switch (key) {
               case KEY_F(1):
                  if (iobs[0]->abswitch == 0) iobs[0]->abswitch = 1;
                  else iobs[0]->abswitch = 0;
                  refresh();
                  break;

               case KEY_F(2):
                  if (iobs[1]->abswitch == 0) iobs[1]->abswitch = 1;
                  else iobs[1]->abswitch = 0;
                  refresh();
                  break;

               case KEY_F(3):
                  if (fsswpos%2 == 0) {
                     col = 26;
                     row = 13 + (fsswpos/2);
                  } else {
                     col = 44;
                     row = 13 + ((fsswpos-1) / 2);
                  } // End if fsswpos%2
                  stringAtXY(row, col, fssw[fsswpos], GREEN_BLACK);
                  if (fsswpos%2 == 0) fsswpos=fsswpos - 2;
                  else fsswpos = fsswpos + 2;
                  if (fsswpos > 9) fsswpos = 8;
                  if (fsswpos < 0) fsswpos = 1;
                  if (fsswpos%2 == 0) {
                     col = 26;
                     row = 13 + (fsswpos / 2);
                  } else {
                     col = 44;
                     row = 13 + ((fsswpos-1) / 2);
                  } // End if fsswpos%2
                  stringAtXY(row, col, fssw[fsswpos], BLACK_GREEN);
                  break;

               case KEY_F(7):
                  pthread_mutex_lock(&r7f_lock);
                  Eregs_Inp[0x7F] |= 0x0200;
                  pthread_mutex_unlock(&r7f_lock);
                  inter_req_L3 = ON;         /* Panel L3 request flag */
                  while (Ireg_bit(0x7F, 0x0200) == ON)
                     wait();
                  break;

               case KEY_RIGHT:
                  nibbleAtXY(10, 32+(hexswpos*3), hexsw[hexswpos], WHITE_BLACK);
                  if (fsswpos == 5) {
                     if (hexswpos == 1) hexswpos = 3;
                     else hexswpos = 1;
                  } else {
                     hexswpos++;
                     if (hexswpos > 4) hexswpos = 0;
                  }
                  nibbleAtXY(10, 32+(hexswpos*3), hexsw[hexswpos], BLACK_WHITE);
                  break;

               case KEY_LEFT:
                  nibbleAtXY(10, 32+(hexswpos*3), hexsw[hexswpos], WHITE_BLACK);
                  if (fsswpos == 5) {
                     if (hexswpos == 3) hexswpos = 1;
                     else hexswpos = 3;
                  } else {
                     hexswpos--;
                     if (hexswpos < 0) hexswpos = 4;
                  }
                  nibbleAtXY(10, 32+(hexswpos*3), hexsw[hexswpos], BLACK_WHITE);
                  break;

               case KEY_UP:
                  hexsw[hexswpos] = (hexsw[hexswpos] + 0x01) & 0x0F;
                  if ((fsswpos == 3) && (hexswpos == 0) && (hexsw[hexswpos] > 0x03))
                     hexsw[hexswpos]=0x00;
                  nibbleAtXY(10, 32+(hexswpos*3), hexsw[hexswpos], BLACK_WHITE);
                  break;

               case KEY_DOWN:
                  hexsw[hexswpos] = (hexsw[hexswpos] - 0x01) & 0x0F;
                  if ((fsswpos == 3) && (hexswpos == 0) && (hexsw[hexswpos] > 0x03))
                     hexsw[hexswpos]=0x03;
                  nibbleAtXY(10, 32+(hexswpos*3), hexsw[hexswpos], BLACK_WHITE);
                  break;
            }  // End switch (key)

            //****************************************************
            //*        Execute the switch function
            //****************************************************
            switch (fsswpos) {
               case 0: // TAR & OP Register
                  Eregs_Inp[0x72] &= 0x0002; /* Reset previous switch setting */
                  break;

               case 1: // Status
                  Eregs_Inp[0x72] &= ~0x1877; /* Reset all previous switch setting */
                  /* Display status from Display A en Display B */
                  nibbleAtXY(4, 26, (Eregs_Out[0x71] >> 16), BLACK_YELLOW);
                  byteAtXY(4, 30, ((Eregs_Out[0x71] >> 8) & 0x00FF), BLACK_YELLOW);
                  byteAtXY(4, 35, (Eregs_Out[0x71]        & 0x000FF), BLACK_YELLOW);
                  nibbleAtXY(4, 40, (Eregs_Out[0x72] >> 16), BLACK_YELLOW);
                  byteAtXY(4, 44, ((Eregs_Out[0x72] >> 8) & 0x00FF), BLACK_YELLOW);
                  byteAtXY(4, 49, (Eregs_Out[0x72]        & 0x000FF), BLACK_YELLOW);

                  /* Decode display register 72*/
                  // Adapter Check
                  if (Eregs_Out[0x72] & 0x0800)
                     stringAtXY(4, 19, " ", BLACK_RED);    // Red light !
                  else
                     stringAtXY(4, 19, " ", BLACK_BLACK);  // No light
                  // In/Out Check
                  if (Eregs_Out[0x72] & 0x0400)
                     stringAtXY(5, 19, " ", BLACK_RED);    // Red light !
                  else
                     stringAtXY(5, 19, " ", BLACK_BLACK);  // No light
                  // Address Exception
                  if (Eregs_Out[0x72] & 0x0200)
                     stringAtXY(6, 19, " ", BLACK_RED);    // Red light !
                  else
                     stringAtXY(6, 19, " ", BLACK_BLACK);  // No light
                  // Protect Check
                  if (Eregs_Out[0x72] & 0x0100)
                     stringAtXY(7, 19, " ", BLACK_RED);    // Red light !
                  else
                     stringAtXY(7, 19, " ", BLACK_BLACK);  // No light
                  // Invalid Operation
                  if (Eregs_Out[0x72] & 0x0080)
                     stringAtXY(8, 19, " ", BLACK_RED);    // Red light !
                  else
                     stringAtXY(8, 19, " ", BLACK_BLACK);  // No light
                  break;

               case 2: // Function Select 6
                  Eregs_Inp[0x72] &= ~0x0004;
                  Eregs_Inp[0x72] |= 0x0002;
                  break;

               case 3: // Storage Address
                  /* Set Hex Switch A can be eiter 0,  1,  2 or 3. If not, set to 0  */
                  if (hexsw[0] > 3) {
                     nibbleAtXY(10, 32+(0), hexsw[hexswpos], WHITE_BLACK);
                     hexsw[0] = 0;
                     nibbleAtXY(10, 32+(0), hexsw[hexswpos], BLACK_WHITE);
                  }
                  // Set current function
                  Eregs_Inp[0x72] |= 0x1000;
                  // Display hex switch setting
                  wrkbyte=hexsw[0];                                         // Get switch A
                  nibbleAtXY(4, 26, wrkbyte, BLACK_YELLOW);                 // Show hex switch A
                  wrkbyte=(hexsw[1] << 4) + hexsw[2];                       // Get switch B and C
                  nibbleAtXY(4, 30, wrkbyte, BLACK_YELLOW);                 // Show hex switch B and C
                  wrkbyte=(hexsw[3] << 4) + hexsw[4];                       // Get switch D and E
                  nibbleAtXY(4, 35, wrkbyte, BLACK_YELLOW);                 // Show hex switch D and E
                  // Get and Display memeory content
                  maddr = (hexsw[0] & 0x0003) << 18;
                  maddr = maddr + (((hexsw[1] << 4) + hexsw[2]) << 8);
                  maddr = maddr + ((hexsw[3] << 4) + hexsw[4]);
                  // Check if address is within specified memory size
                  // If not, display all zero's and switch-on address excpetion light
                  // else display memory content
                  if (maddr > (msize * 1024)-1) {
                     stringAtXY(6, 19, " ", BLACK_RED);                     // Address Exception
                     nibbleAtXY(4, 40, 0x00, BLACK_YELLOW);                 // Write 0 in byte X
                     byteAtXY(4, 44, 0x00, BLACK_YELLOW);                   // Write 00 in byte 0
                     byteAtXY(4, 49, 0x00, BLACK_YELLOW);                   // Write 00 in byte 1
                  } else {
                     wrkbyte = M[maddr];
                     stringAtXY(6, 19, " ", BLACK_BLACK);                   // Make sure Address Exception is off
                     nibbleAtXY(4, 40, 0x00, BLACK_YELLOW);                 // Write 0 in byte X
                     byteAtXY(4, 44, 0x00, BLACK_YELLOW);                   // Write 00 in byte 0
                     byteAtXY(4, 49, wrkbyte, BLACK_YELLOW);                // Write byte from memory location in byte 1
                  }
                  break;

               case 4: // Function Select 5
                  Eregs_Inp[0x72] &= ~0x0008;
                  Eregs_Inp[0x72] |= 0x0004;
                  break;

               case 5: // Register Address
                  /* Hilight Hex switch B and D position */
                  stringAtXY(8, 35, "B", BLACK_GREEN);
                  stringAtXY(8, 41, "D", BLACK_GREEN);
                  if ((hexswpos != 1) && (hexswpos != 3)) {
                     nibbleAtXY(10, 32+(hexswpos*3), hexsw[hexswpos], WHITE_BLACK);
                     hexswpos = 1;
                     nibbleAtXY(10, 32+(hexswpos*3), hexsw[hexswpos], BLACK_WHITE);
                  }
                  // Reset previous function select and set current
                  stringAtXY(6, 19, " ", BLACK_BLACK);                         // Reset Address Exception (in case it was on)
                  Eregs_Inp[0x72] &= ~0x1000;
                  Eregs_Inp[0x72] |= 0x0800;

                  // Display hex switch setting
                  stringAtXY(4, 26, " ", BLACK_YELLOW);                        // Clear byte X
                  wrkbyte=hexsw[1] << 4;
                  nibbleAtXY(4, 30, wrkbyte, BLACK_YELLOW);                    // Show hex switch B in bits 0-3
                  stringAtXY(4, 31, " ", BLACK_YELLOW);                        // Clear bits 4-7
                  wrkbyte=hexsw[3] << 4;
                  nibbleAtXY(4, 35, wrkbyte, BLACK_YELLOW);                    // Show hex switch D in bits 0-3
                  stringAtXY(4, 36, " ", BLACK_YELLOW);                        // Clear bits 4-7

                  // Display register content
                  wrkbyte=(hexsw[1] << 4) + hexsw[3];
                  byteAtXY(4, 44, ((Eregs_Inp[wrkbyte] >> 8) & 0x000FF), BLACK_YELLOW);
                  byteAtXY(4, 49, (Eregs_Inp[wrkbyte]        & 0x000FF), BLACK_YELLOW);
                  break;

               case 6: // Function Select 4
                  // First reset highlited B and D switches
                  hexswpos=1;
                  nibbleAtXY(10, 32+(hexswpos*3), hexsw[hexswpos], WHITE_BLACK);
                  hexswpos=3;
                  nibbleAtXY(10, 32+(hexswpos*3), hexsw[hexswpos], WHITE_BLACK);
                  // Reset previous function select and set current
                  Eregs_Inp[0x72] &= ~0x0010;
                  Eregs_Inp[0x72] |= 0x0008;
                  break;

               case 7: // Function Select 1
                  // Reset highlited B and D switches from previous function
                  stringAtXY(8, 35, "B", GREEN_BLACK);
                  stringAtXY(8, 41, "D", GREEN_BLACK);
                  // Reset previous function select and set current
                  Eregs_Inp[0x72] &= ~0x0800;
                  Eregs_Inp[0x72] |= 0x0040;
                  break;

               case 8: // Function Select 3
                  Eregs_Inp[0x72] &= ~0x0020;
                  Eregs_Inp[0x72] |= 0x0010;
                  break;

               case 9: // Function Select 2
                  Eregs_Inp[0x72] &= ~0x0040;
                  Eregs_Inp[0x72] |= 0x0020;
                  break;
            }  // End switch fsspos
         }  // End while key != exit

         endwin();                    /* end window                      */
         shwpanel = 0;                /* do not show front panel anymore */
         freopen("/dev/tty", "w", stdout);
      }  // End if shwpanel == 1
      usleep(1000);                   /* relax a bit                     */
   }  // End while(1)
}

// *****************************************************
// Interval timer definition
// *****************************************************
void timer_msec (long int msec) {
   struct itimerval timer1;

   timer1.it_interval.tv_usec = (1000 * msec);
   timer1.it_interval.tv_sec = 0;
   timer1.it_value.tv_usec = (1000 * msec);
   timer1.it_value.tv_sec = 0;

   setitimer (ITIMER_REAL, &timer1, NULL);
}

// Kick the 3705 100msec timer...
void sig_handler (int signo) {
   if ((test_mode == OFF) && !(Eregs_Inp[0x7F] &= 0x0004)) {
      pthread_mutex_lock(&r7f_lock);
      Eregs_Inp[0x7F] |= 0x0004;
      pthread_mutex_unlock(&r7f_lock);
      timer_req_L3 = ON;
   }
}

