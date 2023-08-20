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

   s3_defs.h: IBM 3705 simulator definitions
*/

#include "sim_defs.h"                                   /* simulator defns */

/* General */
#define VERSION         "0.1"

#define ON              1
#define OFF             0
#ifndef TRUE
   #define TRUE         1
#endif
#ifndef FALSE
   #define FALSE        0
#endif
#define FILLED          1
#define EMPTY           0

/* Simulator stop codes */

#define STOP_RSRV       1                               /* must be 1 */
#define STOP_HALT       2                               /* HALT */
#define STOP_IBKPT      3                               /* breakpoint */
#define STOP_INVOP      4                               /* program check - invalid op */
#define STOP_INVQ       5                               /* Prog check - invalid Q */
#define STOP_INVADDR    6                               /* Prog check - invalid addr */
#define STOP_INVDEV     7                               /* Prog check - invalid dev cmd */
#define STOP_NOCD       8                               /* ATTN card reader */
#define RESET_INTERRUPT 77                              /* special return from SIO */

/* Memory */

#define MAXMEMSIZE      262144                          /* max memory size */
#define AMASK           (MAXMEMSIZE - 1)                /* logical addr mask */
#define PAMASK          (MAXMEMSIZE - 1)                /* physical addr mask */
#define MEMSIZE         (cpu_unit.capac)                /* actual memory size */

/* I/O structure

   The I/O structure is tied together by dev_table, indexed by
   the device number.  Each entry in dev_table consists of

        level           Interrupt level for device (0-7)
        priority        Priority for device (1-8)
        routine         IOT action routine
*/

struct ndev {
    int32   level;                                      /* interrupt level */
    int32   pri;                                        /* Device priority */
    int32   (*routine)(int32, int32, int32, int32);     /* dispatch routine */
};

/* Structure to define operation codes */

struct opdef {
    char    mnem[6];                                    /* Mnemonic for op */
    int32   opcode;                                     /* Bits set on in opcode */
    int32   opmask;                                     /* Qbyte */
    int32   form;                                       /* Forms are:
                                                           0 - RR
                                                           1 - RRn
                                                           2 - RI
                                                           3 - RT
                                                           4 - RA
                                                           5 - RS
                                                           6 - RTm
                                                           7 - RSA
                                                           8 - RE
                                                           9 - EXIT */
    int32   group;                                      /* Group Code:
                                                           0 - spare */
};


#define MAXHOSTS 2
#define MAXCHAN  2                                      /* Max channels */

/* IBM 3705 I/O structure   */
struct IO3705 {
   char CA_id;
   int CA_active;
   int CA_socket[2];
   uint16_t CA_mask;
   int32 Eregs_Inp[128];
   int32 Eregs_Out[128];
   int addrlen[MAXCHAN*2];   // Two ports per channel (A/B)
   int bus_socket[2];
   int tag_socket[2];
   int abswitch;             // A/B switch
   int abswhist;             // A/B switch history
   int diag;                 // Diagnostic status
   uint16_t devnum;
   uint8_t buffer[1024];     // Input buffer of 1K
   uint32_t bufferl;         // Received data length
   uint8_t  chainbuf[4096];  // Data chaining buffer of 4K
   uint32_t chainbl;         // Data chain buffer length
   uint8_t carnstat;         // CA return status
   uint8_t IPL_exception;    // IPL exception ON or OFF
   struct sockaddr_in address[2];
   pthread_t CA_tid;
};
