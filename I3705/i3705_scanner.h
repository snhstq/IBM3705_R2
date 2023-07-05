/* Copyright (c) 2021, Henk Stegeman and Edwin Freekenhorst

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

   i3705_scanner.h IBM 3720 scanner definitions
*/

/* PSA offsets */
#define TTC             0                      /* Parameter zone */
#define MOD             1
#define off_set         2
#define off_T           2
#define off_R           3
#define cnt_T_W2        4                      /* Byte count at +0 */
#define cnt_R_W2        4
#define buf_T_W2        4                      /* Buf addr at +1 +2 & +3 */
#define buf_R_W2        4
#define XA1             8
#define XC1             9
#define cnt_R_W4        12                     /* Byte count at +0 */
#define buf_R_W4        12                     /* Buf addr at +1 +2 & +3 */

#define SCF             16                     /* Status zone */
#define CCMD            17
#define SES             18
#define LCS             19
#define res_cnt         20                     /* Residu byte count at +0 */
#define buf_last        20                     /* Last used buf at +1, +2 & +3 */
#define SDF             21
#define V24_IN          22
#define V24_OUT         23
#define RA1             24
#define RC1             25

#define TX              1                      /* Transmit 3270 -> client */
#define RX              2                      /* Receive client -> 3270 */
#define REQ             1                      /* Request Unit */
#define RSP             2                      /* Response Unit */

/* NCP Buffer Header offsets */
#define BH_tag          -4                     /* Buffer tag */
#define BH_C2           -3                     /* Buffer overlay check 'C2' */
#define BH_vvti         -2                     /* Virt route vector table index */
#define BH_len          8                      /* Length buffer header */
#define BH_buf_chn      0                      /* Buffer chain field */
#define BH_dat_off      6                      /* Offset start data in buffer */
#define BH_dat_cnt      7                      /* Count data bytes in buffer */

/* Scanner V24 modem signals. */
#define V24_DSR         0x80                   /* Incoming leads from modem */
#define V24_CTS         0x40
#define V24_RI          0x20
#define V24_DCD         0x10
#define V24_TI          0x08
#define V24_RD          0x04

#define V24_DTR         0x80                   /* Outgoing leads to modem */
#define V24_RTS         0x40
#define V24_NSYNC       0x20
#define V24_RATE_SEL    0x10
#define V24_MDM_TEST    0x08

