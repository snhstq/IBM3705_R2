/* Copyright (c) 202?, Henk Stegeman and Edwin Freekenhorst

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

   i327x_3274.c - (C) Copyright 2021 by Edwin Freekenhorst and Henk Stegeman

   This module simulates an IBM 3274 cluster controller, including
   SNA/SDLC protocol.

   Depending on the defintion, 1 or more 3274's are 'IML-ed', making
   the defined LU's availble for sign-on. A LU connects to the desired
   3274 by selecting the approriate teLnet port number.
   The ports are defined as 32741 for the first 3274, 32742 for
   the 2nd, etc.
*/

#include <inttypes.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
#include <string.h>
#include <ifaddrs.h>
#include "i327x_327x.h"
#include "../Include/i327x_sdlc.h"
#include <errno.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#if !defined(min)
#define  min(a,b)   (((a) <= (b)) ? (a) : (b))
#endif

#define BUFPD 0x1C

uint16_t Tdbg_flag = OFF;           /* 1 when Ttrace.log open */
FILE *T_trace;                      /* Terminal trace file fd */

struct sockaddr_in servaddr;
struct sockaddr_in sin1, *sin2;
struct epoll_event event, events[MAXSNAPU*2];
struct ifaddrs *nwaddr, *ifa;       /* interface address structure       */
in_addr_t  lineip;                  /* SDLC line listening address       */
uint16_t   lineport;                /* SDLC line listening port          */
int        pusdlc_fd;
int        station;                 /* Station number based on station address */
int        sockopt;                 /* Used for setsocketoption          */
int        pendingrcv;              /* pending data on the socket        */
int        event_count;             /* # events received                 */
char       *ipaddr;
BYTE       bfr[256];

// Host ---> PU request buffer
uint8_t BLU_req_buf[BUFLEN_3274];   // DLC header + TH + RH + RU + DLC trailer
int     BLU_req_ptr;                // Offset pointer to BLU
int     BLU_req_len;                // Length of BLU request
int     LU_req_stat;                // BLU buffer state
// PU ---> Host response buffer
uint8_t BLU_rsp_buf[BUFLEN_3274];   // DLC header + TH + RH + RU + DLC trailer
int     BLU_rsp_ptr;                // Offset pointer to BLU
int     BLU_rsp_len;                // Length of BLU response
int     BLU_rsp_stat;               // BLU buffer state
// Saved RH
uint8_t  saved_FD2_RH_0;
uint8_t  saved_FD2_RH_1;
int THRH_type;                      // TH / RH type processed

uint8_t Rsp_buf = EMPTY;
uint8_t RSP_buf[BUFLEN_3270];       // Status Response buffer
int Plen;                           // Length of PIU response

struct CB327x* pu2[MAXSNAPU];          /* 3274 data structure */
struct IO3270 *ioblk[MAXSNAPU][MAXLU]; /* 3270 data buffer */

uint8_t SDLCrspb[BUFLEN_3274];
uint8_t SDLCreqb[BUFLEN_3274];

void commadpt_read_tty(struct CB327x *i327x, struct IO3270 *ioblk, BYTE * bfr, BYTE lunum, int len);
int send_packet(int csock, BYTE *buf, int len, char *caption);
int connect_client (int *csockp, BYTE i327xnump, BYTE *lunump, BYTE *lunumr);
int SocketReadAct (int fd);

void make_seq (struct CB327x *pu2, BYTE *bufptr, int lunum);

/*-------------------------------------------------------------------*/
/* Supported FMD NS Headers                                          */
/*-------------------------------------------------------------------*/
static unsigned char R010201[3] = {0x01, 0x02, 0x01};  // CONTACT
static unsigned char R010202[3] = {0x01, 0x02, 0x02};
static unsigned char R010203[3] = {0x01, 0x02, 0x03};
static unsigned char R010204[3] = {0x01, 0x02, 0x04};
static unsigned char R010205[3] = {0x01, 0x02, 0x05};
static unsigned char R01020A[3] = {0x01, 0x02, 0x0A};
static unsigned char R01020B[3] = {0x01, 0x02, 0x0B};
static unsigned char R01020F[3] = {0x01, 0x02, 0x0F};
static unsigned char R010211[3] = {0x01, 0x02, 0x11};
static unsigned char R010216[3] = {0x01, 0x02, 0x16};
static unsigned char R010217[3] = {0x01, 0x02, 0x17};
static unsigned char R010219[3] = {0x01, 0x02, 0x19};  // ANA
static unsigned char R01021A[3] = {0x01, 0x02, 0x1A};
static unsigned char R01021B[3] = {0x01, 0x02, 0x1B};
static unsigned char R010280[3] = {0x01, 0x02, 0x80};
static unsigned char R010281[3] = {0x01, 0x02, 0x81};
static unsigned char R010284[3] = {0x01, 0x02, 0x84};  // REQCONT

/*-------------------------------------------------------------------*/
/* Positive/Negative SNA RU responses                                */
/*-------------------------------------------------------------------*/
uint8_t F2_ACTPU_Rsp[] = {
      0x11, 0x11, 0x40, 0x40,  0x40, 0x40, 0x40, 0x40,
      0x40, 0x40, 0x00, 0x00,  0x07, 0x01, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00 };
uint8_t F2_ACTLU_Rsp[] = {
      0x0D, 0x01, 0x01, 0x00,  0x85, 0x00, 0x00, 0x00,   // see PoO 4-186
      0x0C, 0x0E, 0x03, 0x00,  0x01, 0x00, 0x00, 0x00,
      0x40, 0x40, 0x40, 0x40,  0x40, 0x40, 0x40, 0x40 };
uint8_t F2_ACTLU_NegRsp[] = {
      0x0D, 0x01, 0x01 };
uint8_t F2_DACTPU_Rsp[] = {
      0x12 };
uint8_t F2_DACTLU_Rsp[] = {
      0x0E };
uint8_t F2_BIND_Rsp[] = {
      0x31 };
uint8_t F2_UNBIND_Rsp[] = {
      0x32 };
uint8_t F2_UNBIND_Req[] = {
      0x32,0x01 };
uint8_t F2_RSHUTD_Req[] = {
      0xC2 };
uint8_t F2_SDT_Rsp[]  = {
      0xA0 };
uint8_t F2_CLEAR_Rsp[] = {
      0xA1 };
uint8_t F2_SIGNAL_Rsp[] = {
      0xC9 };
uint8_t F2_QEC_Rsp[] = {
      0x80 };
uint8_t F2_QC_Rsp[] = {
      0x81 };
uint8_t F2_INITSELF_Req[] = {
      0x2C, 0x00, 0x00, 0x02, 0x00, 0x01,  0x0B, 0x80, 0x00,
      0x01, 0x06, 0x81, 0x00,  0x40, 0x40, 0x40, 0x40,  // Mode name
      0x40, 0x40, 0x40, 0x40,  0xF3, 0x08, 0x40, 0x40,  // Req id
      0X40, 0x40, 0x40, 0x40,  0x40, 0x40,
      0x00, 0x00, 0x00 };
uint8_t F2_TERMSELF_Req[] = {
      0x81, 0x06, 0x83, 0x11, 0x44, 0x10,  0x00, 0x00, 0x01,
      0xF3, 0x00};
uint8_t F2_NOTIFY_Req[] = {
      0x81, 0x06, 0x20,                                 // Notify
      0x0C, 0x06, 0x03, 0x00,  0x00, 0x00, 0x00, 0x00};
uint8_t F2_NOTIFY_Req_old[] = {
      0x81, 0x06, 0x20,                                 // Notify
      0x0C, 0x0E, 0x03, 0x00,  0x01, 0x00, 0x00, 0x00,
      0x40, 0x40, 0x40, 0x40,  0x40, 0x40, 0x40, 0x40 };
uint8_t F2_LUSTAT_Req[] = {
      0x04, 0x00, 0x01, 0x00, 0x00 };                   // LU now available
uint8_t F2_REQCONT_Req[] = {
      0x01, 0x02, 0x84, 0x18, 0x00, 0x02, 0x00, 0x01, 0x70, 0x00, 0x17 };  // Request Contact

// uint8_t PLU_rsp_buf[BUFLEN_3270]; /* PIU response buffer: TH + RH + RU  */
int double_up_iac (BYTE *buf, int len);

/*-------------------------------------------------------------------*/
/* Process FID2 PIU (TH + RH + RU (Req)                              */
/*-------------------------------------------------------------------*/
//
//      <-------------------------- PIU -------------------------->
//      <------------ TH -----------> <---- RH ----> <---- RU ---->
//     0    1    2    3    4    5    6    7    8    9         n
//  /--|----+----+----+----+----+----|----+----+----|----+---//----|-
//  ~  |FID2|resv|DAF |OAF | seq nr. |RH_0|RH_1|RH_2|RU_0...    ...| ~
//  /--|----+----+----+----+----+----|----+----+----|----+---//----|-
//
int proc_PIU (unsigned char BLU_req_buf[], int BLU_req_len, unsigned char BLU_rsp_buf[]) {
   // BLU_req_buf[FD2_TH_0] must point to byte 0 of the TH.
   // Fcntl: RR / IFRAME / IFRAME + Cpoll
   BYTE  Dbuf[BUFLEN_3270];            // Data buffer
   int   RU_req_len;                   // RU request length
   int   RU_rsp_len;                   // RU response length
   int   RH_req_len;                   // RH request length
   int   i, station;
   int   chainrh;
   uint8_t Fcntl;
   register char *s;

/*-------------------------------------------------------------------*/
/* Get the station address                                           */
/*-------------------------------------------------------------------*/
   // Load Frame Control Field
   Fcntl = BLU_req_buf[FCntl];
   // Set the 3274 to the provided station address.
   // If it is a broadcast (FF) the station address will be set to C1) (has to be improved)
   if (BLU_req_buf[FAddr] == 0xFF) BLU_req_buf[FAddr] = 0xC1;
   station = (BLU_req_buf[FAddr] & 0x0F) - 1;

   if (Tdbg_flag == ON) {  // Trace Terminal Controller ?
      if ((Fcntl & 0x03) == SUPRV) {                     // Supervisory format ?
         fprintf(T_trace, "PIU0: => Supervisory format received. \n");
      } else if ((Fcntl & 0x03) == UNNUM) {              // Unnumbered format ?
         fprintf(T_trace, "PIU0: => Unnumbered format received. \n");
      } else {                                           // Must be a IFRAME with PIU
         fprintf(T_trace, "PIU0: => Iframe received: BLUlen=%2d, \nPIU0: ", BLU_req_len);
         for (s = (char *) &BLU_req_buf[0], i = 0; i < BLU_req_len; ++i, ++s) {
            fprintf(T_trace, "%02X ", (int) *s & 0xFF);
            if ((i + 1) % 16 == 0)
               fprintf(T_trace, " \nPIU0: ");
         }
         fprintf(T_trace, "\n");
         fflush(T_trace);
      }
   }

   //================================================================
   // Unnumberd format received
   // - SNRM: reset send and receive counters.
   //================================================================
   if  ((Fcntl & 0x03) == UNNUM)   {          //  Unnumbered ?
      BLU_rsp_ptr = 0;
      BLU_rsp_buf[BLU_rsp_ptr++] = 0x7E;      //  LH BFlag
      if ((Fcntl & 0xEF) == SNRM) {           //  Normal Response Mode ?
         if (Tdbg_flag == ON)                 // Trace Terminal Controller ?
            fprintf(T_trace, "\rPIU0=>  SNRM received.");
         if (BLU_req_buf[FCntl] & CPoll) {    // Poll command ?
            BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FAddr];
            BLU_rsp_buf[BLU_rsp_ptr++] = UA + CFinal; // Set final
         } else {
            BLU_rsp_len = 0;                  // No response
         } // end (BLU_req_buf[FCntl] & CPoll)
         pu2[station]->seq_Nr = 0;
         pu2[station]->seq_Ns = 0;
      } // End SNRM
      if ((Fcntl & 0xEF) == DISC) {           // Disconnect ?
         if (Tdbg_flag == ON)   // Trace Terminal Controller ?
            fprintf(T_trace, "\rPIU0=>  DISC received.");
         if (BLU_req_buf[FCntl] & CPoll) {    // Poll command ?
            BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FAddr];
            BLU_rsp_buf[BLU_rsp_ptr++] = UA + CFinal;
         } else {
            BLU_rsp_len = 0;                  // No response
         }
      }  // End DISC
      if ((Fcntl & 0xEF) == XID2) {           // Exchange ID ?
         if (Tdbg_flag == ON)                 // Trace Terminal Controller ?
            fprintf(T_trace, "\rPIU0=>  XID received.");
         if (BLU_req_buf[FCntl] & CPoll) {    // Poll command ?
            BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FAddr];
            BLU_rsp_buf[BLU_rsp_ptr++] = XID2 + CFinal;
            BLU_rsp_buf[BLU_rsp_ptr++] = 0x02;   // Fixed format, PU T2
            BLU_rsp_buf[BLU_rsp_ptr++] = 0x00;   // Reserved
            BLU_rsp_buf[BLU_rsp_ptr++] = 0x01;   // IDBLK (First 8 bits)
            BLU_rsp_buf[BLU_rsp_ptr++] = 0x70;   // IDBLK (last 4 bits)"+ IDNUM (First 4 bits)
            BLU_rsp_buf[BLU_rsp_ptr++] = 0x00;   // IDNUM (Middle 8 bits)
            BLU_rsp_buf[BLU_rsp_ptr++] = 0x17;   // IDNUM (Last 8 bits)
         } else {
            BLU_rsp_len = 0;                  // No response
         }
      } // End XID

      if (BLU_rsp_ptr > 1) {
         BLU_rsp_buf[BLU_rsp_ptr++] = 0x47;   // Complete TH
         BLU_rsp_buf[BLU_rsp_ptr++] = 0x0F;
         BLU_rsp_buf[BLU_rsp_ptr++] = 0x7E;
         BLU_rsp_len = BLU_rsp_ptr;           // Update BLU_rsp_len
      }
      return(BLU_rsp_len);
   }  // End UNNUM

   if (BLU_req_buf[FCntl] & 0x0F) {           // SUPERVISORY Format ?
      if ((Fcntl & 0x0F) == RR) {             // Only a RR ?
         //================================================================
         //=== RR format received                                       ===
         //=== Only executes if:                                        ===
         //=== - LU is connected                                        ===
         //=== - LU is ready (initialized)                              ===
         //=== - LU is active for VTAM (ACTLU)                          ===
         //=== - LU has pending input                                   ===
         //================================================================
         if (BLU_rsp_stat == EMPTY) {         // Empty ?
            // RR received and no response pending.
            // SCAN all active LU's for any new input.
            for (int k = pu2[station]->last_lu; k < MAXLU; k++) {
               if ((pu2[station]->lu_fd[k] > 0) && (pu2[station]->readylu[k] == 1)) {
                  if ((pu2[station]->actlu[k] == 1) && (ioblk[pu2[station]->punum][k]->inpbufl > 0)) {

                     // TN3270 input found. Build FID2 & Rsp RU
                     RU_rsp_len = ioblk[pu2[station]->punum][k]->inpbufl;

                     /* Construct 3 byte LH */
                     BLU_rsp_ptr = 0;                    // Reset pointer
                     BLU_rsp_buf[BLU_rsp_ptr++] = 0x7E;  // Bflag
                     BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FAddr]; // Sec Station Addr
                     //BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FCntl]; // Control byte
                     BLU_rsp_buf[BLU_rsp_ptr++] = CFinal;             // Control byte
                     //BLU_rsp_buf[FCntl] = CFinal;                     // Set final bit

                     /* Construct 6 byte FID2 TH */
                     BLU_rsp_buf[FD2_TH_0] = 0x2E;       // FID2
                     BLU_rsp_buf[FD2_TH_1] = 0x00;       // Reserved
                     BLU_rsp_buf[FD2_TH_daf] = pu2[station]->daf_addr1[k]; //  daf
                     BLU_rsp_buf[FD2_TH_oaf] = k+2;      // oaf
                     BLU_rsp_buf[FD2_TH_scf0] = 0x00;    // seq #
                     BLU_rsp_buf[FD2_TH_scf1] = 0x00;
                     make_seq(pu2[station], BLU_rsp_buf, k); // Update sequence number for this LU

                     /* Construct 3 byte FID2 RH */
                     BLU_rsp_buf[FD2_RH_0] = 0x00;
                     BLU_rsp_buf[FD2_RH_0] |= 0x03;      // Indicate this is first and last in chain
                     BLU_rsp_buf[FD2_RH_1] = 0x80;       // We need a response...
                     pu2[station]->dri[k] = ON;          // ...so remember this
                     BLU_rsp_buf[FD2_RH_2] = 0x20;       // Indicate Change Direction
                     BLU_rsp_ptr = BLU_rsp_ptr + 6 + 3;  // Update BLU pointer

                     /* Copy 3270 input buffer as RU (Rsp) after TH and RH */
                     for (int j = 0; j < RU_rsp_len; j++)
                        BLU_rsp_buf[BLU_rsp_ptr++] = ioblk[pu2[station]->punum][k]->inpbuf[j];

                     /* Construct 3 byte LT */
                     BLU_rsp_buf[BLU_rsp_ptr++] = 0x47;  // FCS High
                     BLU_rsp_buf[BLU_rsp_ptr++] = 0x0F;  // FCS Low
                     BLU_rsp_buf[BLU_rsp_ptr++] = 0x7E;  // Eflag
                     BLU_rsp_len = BLU_rsp_ptr;          // Update BLU_rsp_len
                     if (!(BLU_req_buf[FCntl] & CPoll)) {  // No polling? - Unlikely since this is RR, but just in case ....
                        //BLU_rsp_buf[FCntl] = CFinal;   // Set final bit
                        BLU_rsp_stat = FILLED;           // ...Indicate there is data to send.
                     }
                     ioblk[pu2[station]->punum][k]->inpbufl = 0; // 3270 input buffer has been processed, so reset length.

                     if (Tdbg_flag == ON) {              // Trace Terminal Controller ?
                        fprintf(T_trace, "PIU4: <= 3270 Data [%d]: \nPIU4: ", BLU_rsp_len);
                        for (i = 0; i < BLU_rsp_len; i++) {
                           fprintf(T_trace, "%02X ", (int) BLU_rsp_buf[i] & 0xFF);
                           if ((i + 1) % 16 == 0)
                              fprintf(T_trace, " \nPIU4: ");
                        }
                        fprintf(T_trace, "\n");
                     }
                     /* Cycle through all LU's 1 by 1 to check if there is input.
                        Keep a pointer to the last lu that has been scanned.
                        The next RR will start with the one following the last
                        If all LU's have been scanned, start again with the first LU */
                     pu2[station]->last_lu = k + 1;
                     if (pu2[station]->last_lu == MAXLU)
                        pu2[station]->last_lu = 0;
                     /* Send 3270 data response to host */
                     return(BLU_rsp_len);                // Send 3270 response BLU to host
                  } // End if pu2[station]->actlu[k] == 1
               } else if (((pu2[station]->lu_fd[k] > 0) && (pu2[station]->readylu[k] == 2)) ||
                           (pu2[station]->readylu[k] > 2)) { // End if pu2[station]->lu_fd[k] > 0
                  /* This section handles a LU "power on" (i.e. 3270 terminal connect) or           */
                  /*  a LU "power off" (i.e. 3270 terminal disconnect)                              */
                  /* A SNA Nofify command with LU "powered on" is send to VTAM if readylu=2         */
                  /* A SNA Nofify command with LU "powered off" is send to VTAM if readylu=3        */
                  /* readylu=3 indicates TN3270 has disconnected, but LU is still active for VTAM   */
                  if (Tdbg_flag == ON)                   // Trace Terminal Controller ?
                     fprintf(T_trace, "Preparing UNBIND / NOTIFY request\n ");
                  /* Construct 3 byte LH */
                  BLU_rsp_ptr = 0;                       // Reset pointer
                  BLU_rsp_buf[BLU_rsp_ptr++] = 0x7E;     // Bflag
                  BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FAddr];   // Sec Station Addr
                  //BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FCntl]; // Control byte
                  BLU_rsp_buf[BLU_rsp_ptr++] = CFinal;               // Control byte
                  //BLU_rsp_buf[FCntl] = CFinal;         // Set final bit

                  /* Construct 6 byte FID2 TH */
                  BLU_rsp_buf[FD2_TH_0] = 0x2E;          // FID2
                  BLU_rsp_buf[FD2_TH_1] = 0x00;          // Reserved
                  BLU_rsp_buf[FD2_TH_daf] = pu2[station]->daf_addr1[k]; //  daf
                  BLU_rsp_buf[FD2_TH_oaf] = k+2;         // oaf
                  BLU_rsp_buf[FD2_TH_scf0] = 0x00;       // seq #
                  BLU_rsp_buf[FD2_TH_scf1] = 0x00;
                  make_seq(pu2[station], BLU_rsp_buf, k);  // Update sequence number for this LU

                  /* Construct 3 byte FID2 RH */
                  BLU_rsp_buf[FD2_RH_0] = 0x00;          //  FM Data (FMD)
                  BLU_rsp_buf[FD2_RH_0] |= 0x08;         // Field formatted RU
                  BLU_rsp_buf[FD2_RH_0] |= 0x03;         // Indicate this is first and last in chain
                  BLU_rsp_buf[FD2_RH_1] = 0x00;          // We do not need a response...
                  //BLU_rsp_buf[FD2_RH_1] = 0x80;          // We need a response...
                  //pu2[station]->dri[k] = ON;             // ...so remember this
                  BLU_rsp_buf[FD2_RH_2] = 0x20;          // Indicate Change Direction
                  BLU_rsp_ptr = BLU_rsp_ptr + 6 + 3;     // Update BLU pointer

                  /* This section handles LU Power On and LU Power off     */
                  if (pu2[station]->readylu[k] == 4) {  // There is still an activer BIND, so prepare UNBIND
                    //BLU_rsp_buf[FD2_TH_daf] = pu2[station]->bindflag[k];      //  copy DAF of LU at the other end
                    BLU_rsp_buf[FD2_TH_daf] = 0x00;                             //  SSCP
                    memcpy(&BLU_rsp_buf[BLU_rsp_ptr], F2_TERMSELF_Req, sizeof(F2_TERMSELF_Req));
                    pu2[station]->bindflag[k] = 0;     //  reset bindflag
                    BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_TERMSELF_Req);
                  } // End  if (pu2[station]->readylu[k] == 4)
                  if (pu2[station]->readylu[k] == 3)  {  // Power off, no BIND active, so sent NOTIFY for power off
                    memcpy(&BLU_rsp_buf[BLU_rsp_ptr], F2_NOTIFY_Req, sizeof(F2_NOTIFY_Req)); //
                    BLU_rsp_buf[FD2_RU_0 + 5] = 0x01;        // indicate Power off.
                    BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_NOTIFY_Req);
                  } // End if (pu2[station]->readylu[k] == 3)
                  if (pu2[station]->readylu[k] == 2)  {  // Power on after ACTLU, send NOTIFY for power on
                    memcpy(&BLU_rsp_buf[BLU_rsp_ptr], F2_NOTIFY_Req, sizeof(F2_NOTIFY_Req)); //
                    BLU_rsp_buf[FD2_RU_0 + 5] = 0x03;        // indicate Power on.
                    BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_NOTIFY_Req);
                  } // End if (pu2[station]->readylu[k] == 2)
                  /*                                      */
                  /* Construct 3 byte LT */
                  BLU_rsp_buf[BLU_rsp_ptr++] = 0x47;     // FCS High
                  BLU_rsp_buf[BLU_rsp_ptr++] = 0x0F;     // FCS Low
                  BLU_rsp_buf[BLU_rsp_ptr++] = 0x7E;     // Eflag
                  BLU_rsp_len = BLU_rsp_ptr;             // Update BLU_rsp_len
                  if (!(BLU_req_buf[FCntl] & CPoll)) {   // No polling? - Unlikely since this is RR, but just in case...
                     BLU_rsp_stat = FILLED;              // ...Indicate there is data to send.
                  }
                  if (pu2[station]->readylu[k] > 1)
                     pu2[station]->readylu[k]--;          // Indicate next phase (1 = active, 2 powering on, 3 = powering off, 4 = unbind)
                  /* Cycle through all LU's 1 by 1 to check if there is input. */
                  /* Keep a pointer to the last lu that has been scanned. */
                  /* The next RR will start with the one following the last */
                  /* If all LU's have been scanned, start again with the first LU */
                  pu2[station]->last_lu = k + 1;
                  if (pu2[station]->last_lu == MAXLU) pu2[station]->last_lu = 0;
                  return(BLU_rsp_len);                   // Send 3270 response BLU to host
               }  // End if ((pu2[station]->lu_fd[k] > 0)
            }  // End for int k=0

            // No pending TN3270 input found, just send a RR + CFinal.
            /* Construct a RR response */
            BLU_rsp_buf[BFlag] = 0x7E;                   // Bflag
            BLU_rsp_buf[FAddr] = BLU_req_buf[FAddr];     // Sec Station Addr
            BLU_rsp_buf[FCntl] = BLU_req_buf[FCntl];     // Control byte

            // Send RR with final bit on. (No response PIU)
            //BLU_rsp_buf[FCntl] = RR + CFinal;
            BLU_rsp_buf[FCntl] = RR;

            BLU_rsp_buf[Hfcs] = 0x47;                    // FCS High
            BLU_rsp_buf[Lfcs] = 0x0F;                    // FCS Low
            BLU_rsp_buf[EFlag] = 0x7E;                   // Eflag
            BLU_rsp_len = 6;                             // BLU_rsp_len

            pu2[station]->last_lu = 0;
            return(BLU_rsp_len);                         // Send RR BLU to host

         } // End if (BLU_rsp_stat == EMPTY)

         if (BLU_rsp_stat == FILLED) {                   // RR & Rsp buffer is filled with a SNA cmd resp.
            /* Send response to host */
            if (Tdbg_flag == ON)                         // Trace Terminal Controller ?
               fprintf(T_trace, "RR: Buffer filled. Will be send, reset to empty\n ");
            BLU_rsp_stat = EMPTY;                        // Indicate response buffer is empty
            return 0;                                    // Send Response BLU to host
         }  // End if BLU_rsp_stat == FILLED
      }  // End if RR format

      if ((BLU_req_buf[FCntl] & 0x0F) == RNR)  {
         // Send RNR with final bit on. (No response PIU)
         // Construct a RNR response
         BLU_rsp_buf[BFlag] = 0x7E;                      // Bflag
         BLU_rsp_buf[FAddr] = BLU_req_buf[FAddr];        // Sec Station Addr
         BLU_rsp_buf[FCntl] = RNR;

         BLU_rsp_buf[Hfcs] = 0x47;                       // FCS High
         BLU_rsp_buf[Lfcs] = 0x0F;                       // FCS Low
         BLU_rsp_buf[EFlag] = 0x7E;                      // Eflag
         BLU_rsp_len = 6;                                // BLU_rsp_len

         return(BLU_rsp_len);                            // Send Response BLU to host
      }  // End if RNR format
   }  // End Supervisory format

   //================================================================
   //=== Iframe received with an PIU                              ===
   //================================================================
   pu2[station]->lu_addr0 = 0x00;
   pu2[station]->lu_addr1 = BLU_req_buf[FD2_TH_daf];

   if ((Fcntl & 0x01) == IFRAME) {
      // Determine THRH type
      if ((BLU_req_buf[FD2_TH_0] & 0x01) == 0x00) {      // Normal data flow ?
         // Determine which segment (only, first, middle or last)
         if ((BLU_req_buf[FD2_TH_0] & 0x0C) == 0x0C)     // Only segment ?
            THRH_type = DATA_ONLY;
         if ((BLU_req_buf[FD2_TH_0] & 0x0C) == 0x00)     // Middle segment ?
            THRH_type = DATA_MIDDLE;
         if ((BLU_req_buf[FD2_TH_0] & 0x0C) == 0x04)     // Last segment ?
            THRH_type = DATA_LAST;
         if ((BLU_req_buf[FD2_TH_0] & 0x0C) == 0x08)     // First segment ?
            THRH_type = DATA_FIRST;
      } else {
         // Expedited flow
         // SNA: command or sense code ?
         if (BLU_req_buf[FD2_RH_0] & 0x04)               // Sense Bytes included ?
            THRH_type = SNA_SENSE;                       // SENSE code included
         else
            THRH_type = SNA_CMD;                         // SNA command
      }  // end if else (BLU_req_buf[FD2_TH_0] & 0x01)
      if (Tdbg_flag == ON)                               // Trace Terminal Controller ?
         fprintf(T_trace, "PIU0: => THRH type = %d\n", THRH_type);

      /**********************************************************/
      /*** PROCESS IFRAME as SNA cmd, Resp or as TN3270 DATA STREAM ***/
      /**********************************************************/
      if (Tdbg_flag == ON)                                          // Trace Terminal Controller ?
         fprintf(T_trace, "DRI %d  \n", pu2[station]->dri[pu2[station]->lu_addr1 - 2]);
      if ((THRH_type == DATA_ONLY) &&
          (pu2[station]->dri[pu2[station]->lu_addr1 - 2] == ON)) {  // Response?
         if (((BLU_req_buf[FD2_RH_0] & 0x80) == 0x80) &&            // Should be a Response PIU ...
            ((BLU_req_buf[FD2_RH_1] & 0x80) == 0x80)) {             // ...with DRI on
            // Reset the pending response.
            pu2[station]->dri[pu2[station]->lu_addr1 - 2] = OFF;    // Indicate Response received
            return 0;
         }
      }

      // *****************************************************
      // *****************************************************
      // *** Check if we received SENSE code               ***
      // *****************************************************
      // *****************************************************
      if (THRH_type == SNA_SENSE) {                      // Sense Bytes included ?
         /* Save daf as our own net addr */
         pu2[station]->pu_addr0 = 0x00;
         pu2[station]->pu_addr1 = BLU_req_buf[FD2_TH_daf];
         if (Tdbg_flag == ON) {                          // Trace Terminal Controller ?
            fprintf(T_trace, "\rSense data: ");
            for (int i = 0; i < 4; i++)
               fprintf(T_trace, "\n%02X ", BLU_req_buf[FD2_RU_0 + i]);
            fprintf(T_trace, "\n\r");
            fflush(T_trace);
         }  // End if ((Tdbg_flag == ON)
      }  // End if (THRH_type == SNA_SENSE)

      // *****************************************************
      // *** Check for use of chaining instead of segments ***
      // *****************************************************
      chainrh = 0;                                       // Reset RH length
      if ((THRH_type == DATA_ONLY) && ((BLU_req_buf[FD2_RH_0] & 0x03) != 0x03)) {   // If not a Begin as well as End Chain
         chainrh = 3;                                    // Chaining includes a RH for middle and last chains (segments do not).
         if (BLU_req_buf[FD2_RH_0] & 0x02) {
            THRH_type = DATA_FIRST;
            pu2[station]->chaining[pu2[station]->lu_addr1 - 2] = ON;  // Remember we are in a chain
            if (Tdbg_flag == ON)                         // Trace Terminal Controller ?
               fprintf(T_trace, "PIU0: => THRH type changed to %d because of chaining. \n", THRH_type);
         }
         if (BLU_req_buf[FD2_RH_0] & 0x01) {
            THRH_type = DATA_LAST;
            pu2[station]->chaining[pu2[station]->lu_addr1 - 2] = OFF;    // No longer in a chain
            if (Tdbg_flag == ON)                         // Trace Terminal Controller ?
               fprintf(T_trace, "PIU0: => THRH type changed to %d because of chaining. \n", THRH_type);
         }
         if (((BLU_req_buf[FD2_RH_0] & 03) == 0x00) &&
              (pu2[station]->chaining[pu2[station]->lu_addr1 - 2] == ON)) {
            THRH_type = DATA_MIDDLE;
            if (Tdbg_flag == ON)                         // Trace Terminal Controller ?
               fprintf(T_trace, "PIU0: => THRH type changed to %d because of chaining. \n", THRH_type);
         }
      } // End if (THRH_type == DATA_ONLY)

      // ****************************************************
      // ****************************************************
      // *** Check if we received 3270 DATA               ***
      // ****************************************************
      // ****************************************************
      // RU is type DATA.
      // Get RU_req_len by searching for x'470F7E'. (CRC + EFlag)
      // and copy data to the Dbuf.
      if ((THRH_type == DATA_ONLY) || (THRH_type == DATA_FIRST) ||
          (THRH_type == DATA_MIDDLE) || (THRH_type == DATA_LAST)) {
         if (Tdbg_flag == ON)                            // Trace Terminal Controller ?
            fprintf(T_trace, "PIU0: => IFRAME data type received. \n");
         if ((THRH_type == DATA_ONLY) || (THRH_type == DATA_FIRST)) {  // only or first segment ?
            i = 0;
            // TH & RH when first or only segment
            while (!((BLU_req_buf[PIU + FD2_TH_len + FD2_RH_len + i+0] == 0x47) &&
                     (BLU_req_buf[PIU + FD2_TH_len + FD2_RH_len + i+1] == 0x0F) &&
                     (BLU_req_buf[PIU + FD2_TH_len + FD2_RH_len + i+2] == 0x7E))) {
               Dbuf[i] = BLU_req_buf[PIU + FD2_TH_len + FD2_RH_len + i];
               i++;
               // Save RH for building a response RH later
               saved_FD2_RH_0 = BLU_req_buf[FD2_RH_0];
               saved_FD2_RH_1 = BLU_req_buf[FD2_RH_1];
            }
            RU_req_len = i;
         }  // End if ((THRH_type == DATA_ONLY)

         if ((THRH_type == DATA_MIDDLE) || (THRH_type == DATA_LAST)) {  // middle or last segment ?
            i = 0;
            // Only a TH when middle or last segment, but if chaining: There will also be a RH.
            while (!((BLU_req_buf[PIU + FD2_TH_len + chainrh + i+0] == 0x47) &&
                     (BLU_req_buf[PIU + FD2_TH_len + chainrh + i+1] == 0x0F) &&
                     (BLU_req_buf[PIU + FD2_TH_len + chainrh + i+2] == 0x7E))) {
               Dbuf[i] = BLU_req_buf[PIU + FD2_TH_len + chainrh + i];
               i++;
            }
            RU_req_len = i;
         }  // End if THRH type = DATA_MIDDLE || THRH_type = DATA_LAST

         if ((THRH_type == DATA_ONLY) || (THRH_type == DATA_LAST)) {   // only or last seg ?
            Dbuf[RU_req_len++] = IAC;
            Dbuf[RU_req_len++] = EOR_MARK;
         }

         if (Tdbg_flag == ON) {                          // Trace Terminal Controller ?
            fprintf(T_trace, "PIU5: 3270 Data => [%d]: \nPIU5: ", RU_req_len);
            for ( i = 0; i < RU_req_len; i++) {
               fprintf(T_trace, "%02X ", (int) Dbuf[i] & 0xFF);
               if ((i + 1) % 16 == 0)
                  fprintf(T_trace, " \nPIU5: ");
            }
            fprintf(T_trace, "\n");
            fflush(T_trace);
         }
         //************************************************************
         if   (pu2[station]->lu_fd[pu2[station]->lu_addr1 - 2] > 0)
            send_packet (pu2[station]->lu_fd[pu2[station]->lu_addr1 - 2], (BYTE *) Dbuf, RU_req_len, "3270 Data");
         //************************************************************

         //*******************************************************************************************************
         //* The section below creates a response if the response flag (DRI) in the RH is on
         //* The response is created upon one of the following conditions:
         //* - When there is no Segmentation or Chaining (DATA_ONLY): use the sequence number in the TH of the BTU
         //* - When there is Segmentation: based on DATA_FIRST using the TH sequence number of the first segment
         //* - When there is chaining: based on DATA_LAST using the TH sequence number of the last chain
         //*******************************************************************************************************

         if (((chainrh == 3) && (THRH_type == DATA_LAST)) || (THRH_type == DATA_ONLY)) {  // If last or only segment check for DR1
            if ((BLU_req_buf[FD2_RH_1] & 0x80) != 0x80) {   // Disregard if not DR1 requested
               return 0;
            }
         }
         if (((chainrh == 3) && (THRH_type == DATA_FIRST)) || (THRH_type == DATA_MIDDLE))
            return 0;                                       // Disregard if 1st in chain or middle segment/chain

         if ((chainrh != 3) && (THRH_type == DATA_LAST)) {  // A last segment will flag that there is a response
            BLU_rsp_stat = FILLED;                          // Update BLU buf status
            return(BLU_rsp_len);
         }

         /* Send a +Rsp back to the host */
         /* Construct 3 byte SDLC LH */
         BLU_rsp_ptr = 0;                                // Reset pointer
         BLU_rsp_buf[BLU_rsp_ptr++] = 0x7E;              // Bflag
         BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FAddr];   // Sec Station Addr
         BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FCntl];   // Control byte
         //BLU_rsp_buf[FCntl] = CFinal;                  // Set final bit

         /* Construct 6 byte FID2 TH */
         BLU_rsp_buf[FD2_TH_0]    = BLU_req_buf[FD2_TH_0] | 0x0C;  // FID2 & only segment
         BLU_rsp_buf[FD2_TH_1]    = BLU_req_buf[FD2_TH_1];     // Reserved
         BLU_rsp_buf[FD2_TH_daf]  = BLU_req_buf[FD2_TH_oaf];   // oaf -> daf
         BLU_rsp_buf[FD2_TH_oaf]  = BLU_req_buf[FD2_TH_daf];   // daf -> oaf
         BLU_rsp_buf[FD2_TH_scf0] = BLU_req_buf[FD2_TH_scf0];  // seq #
         BLU_rsp_buf[FD2_TH_scf1] = BLU_req_buf[FD2_TH_scf1];

         /* Construct 3 byte FID2 RH */
         BLU_rsp_buf[FD2_RH_0]  = saved_FD2_RH_0;        // RU_cat & FI
         BLU_rsp_buf[FD2_RH_0] |= 0x83;                  // Indicate this is a Response
         BLU_rsp_buf[FD2_RH_0] &= 0xFB;                  // Reset SDI
         BLU_rsp_buf[FD2_RH_1]  = saved_FD2_RH_1 & 0xEF; // +Rsp
         BLU_rsp_buf[FD2_RH_2]  = 0x00;

         BLU_rsp_ptr = BLU_rsp_ptr + 6 + 3;              // Update pointer
      }

      // ****************************************************
      // ****************************************************
      // *** Check if we received an SNA command          ***
      // ****************************************************
      // ****************************************************
      if (THRH_type == SNA_CMD) {
         // RU contains a SNA command.
         if (Tdbg_flag == ON)                            // Trace Terminal Controller ?
            fprintf(T_trace, "PIU0: => IFRAME SNA command 0x%02X received.\n",
                    BLU_req_buf[FD2_RU_0]);

         /* Prepare FID2 SNA response
         /* Construct 3 byte SDLC LH */
         BLU_rsp_ptr = 0;                                // Reset pointer
         BLU_rsp_buf[BLU_rsp_ptr++] = 0x7E;              // Bflag
         BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FAddr];   // Sec Station Addr
         BLU_rsp_buf[BLU_rsp_ptr++] = BLU_req_buf[FCntl];   // Control byte

         /* Construct 6 byte FID2 TH */
         BLU_rsp_buf[FD2_TH_0]    = BLU_req_buf[FD2_TH_0];     // FID2
         BLU_rsp_buf[FD2_TH_1]    = BLU_req_buf[FD2_TH_1];     // Reserved
         BLU_rsp_buf[FD2_TH_daf]  = BLU_req_buf[FD2_TH_oaf];   // oaf -> daf
         BLU_rsp_buf[FD2_TH_oaf]  = BLU_req_buf[FD2_TH_daf];   // daf -> oaf
         BLU_rsp_buf[FD2_TH_scf0] = BLU_req_buf[FD2_TH_scf0];  // seq #
         BLU_rsp_buf[FD2_TH_scf1] = BLU_req_buf[FD2_TH_scf1];

         /* Construct 3 byte FID2 RH */
         BLU_rsp_buf[FD2_RH_0]  = BLU_req_buf[FD2_RH_0];       // <--- NEEDS UPDATE
         BLU_rsp_buf[FD2_RH_0] |= 0x83;                        // Indicate this is a Response
         BLU_rsp_buf[FD2_RH_0] &= 0xFB;                        // Reset SDI
         BLU_rsp_buf[FD2_RH_1]  = BLU_req_buf[FD2_RH_1] & 0xEF;  // +Rsp
         BLU_rsp_buf[FD2_RH_2]  = 0x00;

         BLU_rsp_ptr = BLU_rsp_ptr + 6 + 3;                    // Update pointer

         /***********************/
         /*** ACTPU (PU)      ***/
         /***********************/
         if (BLU_req_buf[FD2_RU_0] == 0x11) {
            /* Save daf as our own net addr */
            pu2[station]->pu_addr0 = 0x00;
            pu2[station]->pu_addr1 = BLU_req_buf[FD2_TH_daf];

            // Copy +ACTPU to RU.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_ACTPU_Rsp, sizeof(F2_ACTPU_Rsp));
            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_ACTPU_Rsp);  // Update pointer
         }  // End if BLU_buf (ACTPU)

         /***********************/
         /*** ACTLU           ***/
         /***********************/
         if (BLU_req_buf[FD2_RU_0] == 0x0D) {
            /* Save daf as our own net addr */
            //pu2[station]->lu_addr0 = 0x00;
            //pu2[station]->lu_addr1 = BLU_req_buf[FD2_TH_daf];
            pu2[station]->daf_addr1[pu2[station]->lu_addr1 - 2] = BLU_req_buf[FD2_TH_oaf];
            /* Save oaf as our sscp net addr */
            pu2[station]->sscp_addr0 = 0x00;
            pu2[station]->sscp_addr1 = BLU_req_buf[FD2_TH_oaf];
            /*            */
            // pu2[station]->lu_sscp_seqn = 0;
            pu2[station]->bindflag[pu2[station]->lu_addr1 - 2] = 0;
            pu2[station]->initselfflag[pu2[station]->lu_addr1 - 2] = 0;
            // Send +Rsp.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_ACTLU_Rsp, sizeof(F2_ACTLU_Rsp));
            if (pu2[station]->lu_fd[pu2[station]->lu_addr1 - 2] > 0)     // If LU connected (i.e. x3270 telnet)
               BLU_rsp_buf[FD2_RU_0 + 10] = 0x03;        // indicate Power on
            else                                         // else
               BLU_rsp_buf[FD2_RU_0 + 10] = 0x01;        // indicate Power off
            pu2[station]->actlu[pu2[station]->lu_addr1 - 2] = 1;
            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_ACTLU_Rsp);    // Update pointer
         }  // End if BLU_buf (ACTLU)

         /***********************/
         /*** BIND            ***/
         /***********************/
         if (BLU_req_buf[FD2_RU_0] == 0x31) {
            pu2[station]->daf_addr1[pu2[station]->lu_addr1 - 2] = BLU_req_buf[FD2_TH_oaf];
            pu2[station]->lu_lu_seqn[pu2[station]->lu_addr1 - 2] = 0;
            pu2[station]->bindflag[pu2[station]->lu_addr1 - 2] = 1;
            // If not FM3 profile or cols < 24 or rows < 80, respond with -BIND
            if ((BLU_req_buf[FD2_RU_0 + 2] != 0x03) ||
                (BLU_req_buf[FD2_RU_0 + 20] < 0x18) ||
                (BLU_req_buf[FD2_RU_0 + 21] < 0x50 )) {
                   BLU_rsp_buf[FD2_RH_1] = BLU_req_buf[FD2_RH_1] | 0x10;  // -Rsp
                   pu2[station]->bindflag[pu2[station]->lu_addr1 - 2] = 0;
               }
            // Copy BIND to RU.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_BIND_Rsp, sizeof(F2_BIND_Rsp));

            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_BIND_Rsp);   // Update pointer
         }

         /********************************/
         /*** SDT (Start Data Traffic) ***/
         /********************************/
         if (BLU_req_buf[FD2_RU_0] == 0xA0) {
            /* Save oaf from SDT request */
            pu2[station]->daf_addr1[pu2[station]->lu_addr1 - 2] = BLU_req_buf[FD2_TH_oaf];
            pu2[station]->lu_lu_seqn[pu2[station]->lu_addr1 - 2] = 0;
            // Copy +SDT to RU.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_SDT_Rsp, sizeof(F2_SDT_Rsp));

            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_SDT_Rsp);    // Update pointer
         }

         /*******************************/
         /*** CLEAR                   ***/
         /*******************************/
         if (BLU_req_buf[FD2_RU_0] == 0xA1) {
            /* Save oaf from request */
            pu2[station]->daf_addr1[pu2[station]->lu_addr1 - 2] = BLU_req_buf[FD2_TH_oaf];
            pu2[station]->lu_lu_seqn[pu2[station]->lu_addr1 - 2] = 0;
            // Copy +CLEAR to RU.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_CLEAR_Rsp, sizeof(F2_CLEAR_Rsp));

            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_CLEAR_Rsp);   // Update pointer
         }

         /*******************************/
         /*** SIGNAL                  ***/
         /*******************************/
         if (BLU_req_buf[FD2_RU_0] == 0xC9) {
            /* Save oaf from request */
            pu2[station]->daf_addr1[pu2[station]->lu_addr1 - 2] = BLU_req_buf[FD2_TH_oaf];
            // Copy +SIGNAL to RU.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_SIGNAL_Rsp, sizeof(F2_SIGNAL_Rsp));

            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_SIGNAL_Rsp);   // Update pointer
         }

         /*******************************/
         /*** QEC                     ***/
         /*******************************/
         if (BLU_req_buf[FD2_RU_0] == 0x80) {
            /* Save oaf from request */
            pu2[station]->daf_addr1[pu2[station]->lu_addr1 - 2] = BLU_req_buf[FD2_TH_oaf];
            // Copy +QEC to RU.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_QEC_Rsp, sizeof(F2_QEC_Rsp));
            BLU_rsp_buf[FD2_RH_0] &= 0xFB;                   // Reset SDI bit
            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_QEC_Rsp);  // Update pointer
            //usleep(50000);
            //BLU_rsp_len = 6;
         }

         /*******************************/
         /*** QC                      ***/
         /*******************************/
         if (BLU_req_buf[FD2_RU_0] == 0x81) {
            /* Save oaf from request */
            pu2[station]->daf_addr1[pu2[station]->lu_addr1 - 2] = BLU_req_buf[FD2_TH_oaf];
            // Copy +QC to RU.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_QC_Rsp, sizeof(F2_QC_Rsp));
            BLU_rsp_buf[FD2_RH_0] &= 0xFB;                   // Reset SDI bit
            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_QC_Rsp);   // Update pointer
         }


         /*******************************/
         /*** DACTPU                  ***/
         /*******************************/
         if (BLU_req_buf[FD2_RU_0] == 0x12) {
            // Copy +DACTPU to RU.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_DACTPU_Rsp, sizeof(F2_DACTPU_Rsp));

            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_DACTPU_Rsp);   // Update pointer
         }

         /*******************************/
         /*** DACTLU                  ***/
         /*******************************/
         if (BLU_req_buf[FD2_RU_0] == 0x0E) {
            /* Save oaf from request */
            pu2[station]->daf_addr1[pu2[station]->lu_addr1 - 2] = BLU_req_buf[FD2_TH_oaf];
            pu2[station]->lu_lu_seqn[pu2[station]->lu_addr1 - 2] = 0;
            // Copy +DACTLU to RU.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_DACTLU_Rsp, sizeof(F2_DACTLU_Rsp));

            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_DACTLU_Rsp);   // Update pointer
            pu2[station]->actlu[pu2[station]->lu_addr1 - 2] = 0;
         }

         /**************************************/
         /*** UNBIND & Normal end of session ***/
         /**************************************/
         //if (BLU_req_buf[FD2_RU_0] == 0x32 && BLU_req_buf[FD2_RU_1] != 0x02) {
         if (BLU_req_buf[FD2_RU_0] == 0x32) {
            pu2[station]->bindflag[pu2[station]->lu_addr1 - 2 ] = 0;
            /* Save oaf from UNBIND request */
            pu2[station]->daf_addr1[pu2[station]->lu_addr1 - 2] = BLU_req_buf[FD2_TH_oaf];
            pu2[station]->lu_lu_seqn[pu2[station]->lu_addr1 - 2] = 0;
            // Copy +UNBIND to RU.
            memcpy(&BLU_rsp_buf[FD2_RU_0], F2_UNBIND_Rsp, sizeof(F2_UNBIND_Rsp));

            BLU_rsp_ptr = BLU_rsp_ptr + sizeof(F2_UNBIND_Rsp);   // Update pointer
            //pu2[station]->readylu[pu2[station]->lunum] = 2;      // Set LU in power off state to force a NOTIFY command.
         }
      }  // End if ((BLU_req_buf[FD2_RH_0] & (unsigned char)0xFC) != 0x00)

      // *************************************************************************
      // *** End assembly: update seqnr's and add LT to PIU in BLU resp buffer ***
      // *************************************************************************
      if (Tdbg_flag == ON)    // Trace Terminal Controller ?
         fprintf(T_trace, "PIU3: <= Fcntl=0x%02X \n", Fcntl );

      /* Construct 3 byte SDLC LT */
      BLU_rsp_buf[BLU_rsp_ptr++] = 0x47;                 // FCS High
      BLU_rsp_buf[BLU_rsp_ptr++] = 0x0F;                 // FCS Low
      BLU_rsp_buf[BLU_rsp_ptr++] = 0x7E;                 // Eflag
      BLU_rsp_len = BLU_rsp_ptr;                         // BLU_rsp_len

      if (THRH_type != DATA_FIRST) {                     // Do not send a resp if 1st segment (keep until last segment)
         BLU_rsp_stat = FILLED;                          // Update BLU buf status
      } else {                                           // If a first segment...
         return 0;

      }

      if (Tdbg_flag == ON) {  // Trace Terminal Controller ?
         fprintf(T_trace, "PIU3: <= SNA rsp: \nPIU3: ");
         for (s = (char *) &BLU_rsp_buf[0], i = 0; i < (BLU_rsp_len); ++i, ++s) {
            fprintf(T_trace, "%02X ", (int) *s & 0xFF);
            if ((i + 1) % 16 == 0)
               fprintf(T_trace, " \nPIU3: ");
         }
         fprintf(T_trace, "\n");
         fflush(T_trace);
      }
      return(BLU_rsp_len);
   }
}
//#####################################################################


/*-------------------------------------------------------------------*/
/* Subroutine to create unique PIU sequence numbers.                 */
/*-------------------------------------------------------------------*/
void make_seq (struct CB327x * pu2, BYTE * bufptr, int lunum) {
   bufptr[FD2_TH_scf0] = (unsigned char)(++pu2->lu_lu_seqn[lunum] >> 8) & 0xff;
   bufptr[FD2_TH_scf1] = (unsigned char)(  pu2->lu_lu_seqn[lunum]     ) & 0xff;
}

/********************************************************************/
/* Procedure to 'iml' the 3274                                      */
/********************************************************************/
int proc_PU2iml() {
   for (BYTE j = 0; j < MAXSNAPU; j++) {
      pu2[j] =  malloc(sizeof(struct CB327x));
      //Init sockets for LU's
      for (BYTE i = 0; i < MAXLU; i++) {
         pu2[j]->lu_fd[i] = 0;
         pu2[j]->readylu[i] = 0;
      } // End for i = 0
      pu2[j]->lunum = 0;
      pu2[j]->last_lu = 0;
      pu2[j]->punum = j;
      pu2[j]->seq_Nr = 0;    /* Intitialize sequence receive number */
      pu2[j]->seq_Ns = 0;    /* Intitialize sequence send number    */
   } // End for j = 0

   getifaddrs(&nwaddr);      /* get network address */
   for (ifa = nwaddr; ifa != NULL; ifa = ifa->ifa_next) {
       if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name, "lo")) {
          sin2 = (struct sockaddr_in *) ifa->ifa_addr;
          ipaddr = inet_ntoa((struct in_addr) sin2->sin_addr);
          if (strcmp(ifa->ifa_name, "eth")) break;
       }
   } // End   for ifa = nwaddr
   printf("\nPU2: Using network Address %s on %s for 3270 connections\n", ipaddr, ifa->ifa_name);

   for (BYTE j = 0; j < MAXSNAPU; j++) {
      if ((pu2[j]->pu_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1)
         printf("\nPU2: Endpoint creation for 3274 failed with error %s ", strerror(errno));
      /* Reuse the address regardless of any */
      /* spurious connection on that port    */
      sockopt=1;
      setsockopt(pu2[j]->pu_fd, SOL_SOCKET, SO_REUSEADDR, (void*)&sockopt, sizeof(sockopt));
      /* Bind the socket */
      sin1.sin_family=AF_INET;
      sin1.sin_addr.s_addr = inet_addr(ipaddr);
      sin1.sin_port=htons(32741+j);
      if (bind(pu2[j]->pu_fd, (struct sockaddr *)&sin1, sizeof(sin1)) < 0) {
          printf("\nPU2: Bind 3274-%01X socket failed\n\r", j);
          free(pu2[j]);
          return -1;
      }
      /* Listen and verify */
      if ((listen(pu2[j]->pu_fd, 10)) != 0) {
         printf("\nPU2: 3174-%01X Socket listen failed %s\n\r", j, strerror(errno));
          free(pu2[j]);
          return -2;
      }
      // Add polling events for the port
      pu2[j]->epoll_fd = epoll_create(1);
      if (pu2[j]->epoll_fd == -1) {
         printf("\nPU2: failed to created the 3274-%01X epoll file descriptor\n\r", j);
         free(pu2[j]);
         return -3;
      }
      event.events = EPOLLIN;
      event.data.fd = pu2[j]->pu_fd;
      if (epoll_ctl(pu2[j]->epoll_fd, EPOLL_CTL_ADD, pu2[j]->pu_fd, &event) == -1) {
         printf("\nPU2: Add polling event failed for 3274-%01X with error %s \n\r", j, strerror(errno));
         close(pu2[j]->epoll_fd);
         free(pu2[j]);
         return -4;
      }
      printf("\rPU2: 3274-%01X IML ready. TN3270 can connect to port %d \n\r", j,32741+j);
   }  // End for j=0
   return 0;
 }
/********************************************************************/
/* Procedure to handle 3270 connections and data requests           */
/********************************************************************/
int proc_3270 () {

   int    rc;
   //
   // Poll briefly for connect requests. If a connect request is received,
   // proceed with connect/accept the request.
   // Next, check all active connection for input data.
   //
      for (BYTE k = 0; k < MAXSNAPU; k++) {
         event_count = epoll_wait(pu2[k]->epoll_fd, events, MAXLU, 50);
         for (int i = 0; i < event_count; i++) {
            if (pu2[k]->lunum != 0xFF) {                                   /* if available LU pool not exhausted      */
               //pu2[k]->readylu[pu2[k]->lunum] = 0;                         /* Indicate LU is not yet ready for action */
               pu2[k]->lu_fd[pu2[k]->lunum]=accept(pu2[k]->pu_fd, NULL, 0);  /* accept connection request               */
               if (pu2[k]->lu_fd[pu2[k]->lunum] < 1) {
                  printf("\rPU2: accept failed for 3174-%01X %s\n", k, strerror(errno));
               } else {
                  if (connect_client(&pu2[k]->lu_fd[pu2[k]->lunum], pu2[k]->punum, &pu2[k]->lunum, &pu2[k]->lunumr))  {
                     pu2[k]->is_3270[pu2[k]->lunum] = 1;
                  } else {
                     pu2[k]->is_3270[pu2[k]->lunum] = 0;
                  }  // End if connect_client

                  if (pu2[k]->lunumr != pu2[k]->lunum) {                                  /* Requested LU number is not the proposed lu number   */
                     if (pu2[k]->lu_fd[pu2[k]->lunumr] < 1) {
                        pu2[k]->lu_fd[pu2[k]->lunumr]  = pu2[k]->lu_fd[pu2[k]->lunum];    /* Copy fd to request lu                               */
                        pu2[k]->is_3270[pu2[k]->lunumr] = pu2[k]->is_3270[pu2[k]->lunum]; /* copy 3270 indicator                                 */
                        pu2[k]->lu_fd[pu2[k]->lunum] = 0;                                 /* clear fd in proposed lu number                      */
                        pu2[k]->is_3270[pu2[k]->lunum] = 0;                               /* clear 3270 indicator for proposed lu number         */
                        pu2[k]->lunum = pu2[k]->lunumr;                                   /* replace proposed lu number with requested lu number */
                     } else {
                        printf("\rPU2: requested lu port %02X is not available, request denied\n", pu2[k]->lunumr);
                     }  // End  if (pu2[k]->lu_fd[pu2[k]->lunumr]
                  }  // End if pu[k]->lunumr
                  ioblk[k][pu2[k]->lunum] =  malloc(sizeof(struct IO3270));
                  ioblk[k][pu2[k]->lunum]->inpbufl = 0;                   /* make sure the initial length is 0  */
                  pu2[k]->daf_addr1[pu2[k]->lunum] = 0;                   /* make sure the initial value is 0   */
                  pu2[k]->bindflag[pu2[k]->lunum] = 0;                    /* make sure the initial value is 0   */
                  pu2[k]->reqcont[pu2[k]->lunum] = 0;                     /* make sure the initial value is 0   */
                  pu2[k]->initselfflag[pu2[k]->lunum] = 0;                /* make sure the initial value is 0   */
                  pu2[k]->dri[pu2[k]->lunum] = OFF;                       /* make sure the initial value is OFF */
                  pu2[k]->chaining[pu2[k]->lunum] = OFF;                  /* make sure the initial value is OFF */
                  if (pu2[k]->actlu[pu2[k]->lunum] == 1)                  /* Is actlu already done?             */
                     pu2[k]->readylu[pu2[k]->lunum] = 2;                  /* Indicate LU is in power off state  */
                  else
                     pu2[k]->readylu[pu2[k]->lunum] = 1;                  /* Indicate LU is ready to go         */
                  if (Tdbg_flag == ON)    // Trace Terminal Controller ?
                     fprintf(T_trace, "3274: LU %02X connected, readylu=%d \n", pu2[k]->lunum, pu2[k]->readylu[pu2[k]->lunum]);
                  printf("\rPU2: LU %02X connected to 3274-%01X\n", pu2[k]->lunum, k);
                  //  Find first available LU
                  pu2[k]->lunum = 0xFF;                                   /* preset to no LU's availble         */
                  for (BYTE j = 0; j < MAXLU; j++) {
                     if (pu2[k]->lu_fd[j] < 1) pu2[k]->lunum = j;
                  }  // end for BYTE j
                  if (pu2[k]->lunum == 0xFF) printf("\rPU2: No more LU ports available. New connections rejected until a LU port is released;\n");
               }  // End if pu2[j]->lu_fd
            }  // End if (pu2[k] != 0xFF)
         }  // End for int i
         for (BYTE j = 0; j < MAXLU; j++) {
            if (pu2[k]->lu_fd[j] > 0) {
               rc = ioctl(pu2[k]->lu_fd[j], FIONREAD, &pendingrcv);
               if ((pendingrcv < 1) && (SocketReadAct(pu2[k]->lu_fd[j]))) rc = -1;
               if (rc < 0) {
                  if (pu2[k]->actlu[j] == 1)  {                           /* Is actlu already done?                                 */
                      if (pu2[k]->bindflag[j] == 0)                       /* LU has no active BIND                                  */
                        pu2[k]->readylu[j] = 3;                           /* Indicate LU is in power off state (triggers a NOTIFY)  */
                     else                                                 /* LU has an active BIND                                  */
                        pu2[k]->readylu[j] =  4;                          /* Indicate LU is in power off state (triggers an UNBIND) */
                     }
                  else {
                     pu2[k]->readylu[j] = 0;                              /* Indicate LU is not ready for action anymore            */
                     pu2[k]->actlu[j] = 0;                                /* Indicate ACTLU has not been sent                       */
                  }
                  if (Tdbg_flag == ON)    // Trace Terminal Controller ?
                     fprintf(T_trace, "3274: LU %02X disconnected, readylu=%d \n", j, pu2[k]->readylu[j]);
                  pu2[k]->reqcont[j] = 0;                                 /* Indicate LU has not requested contact                  */
                  free(ioblk[k][j]);
                  close (pu2[k]->lu_fd[j]);
                  pu2[k]->lu_fd[j] = 0;
                  printf("\rPU2: LU %02X disconnected from 3174-%01X\n\r", j, k);
                  if ((pu2[k]->lunum > j) || (pu2[k]->lunum == 0xFF))     /* If next available lu greater or no LU's availble...    */
                     pu2[k]->lunum = j;                                   /* ...replace with the just released LU number            */
               } else {
                  if (pendingrcv > 0) {
                     rc = read(pu2[k]->lu_fd[j], bfr, 256-BUFPD);
                     //******
                     if (Tdbg_flag == ON) {              // Trace
                        fprintf(T_trace, "\n3270 Read Buffer: ");
                        for (int i=0; i < rc; i ++) {
                           fprintf(T_trace, "%02X ", bfr[i]);
                        }
                        fprintf(T_trace, "\n\r");
                     }  // End if Tdbg_flag == ON
                     //******
                     commadpt_read_tty(pu2[k], ioblk[k][j], bfr, j, rc);
                  }  // End if pendingrcv
               }  // End if rc < 0
            }  // End if pu2-lu_fd
         }  // End for int j
      }  // End for int k
   return 0;
}

void main(int argc, char *argv[]) {
   unsigned long inaddr;
   struct hostent *lineent;
   uint16_t SDLCrspl;               /* Size of response frame        */
   uint16_t SDLCreql;               /* Size of request fram          */
   int SDLCrsptl = 0;               /* Total size of response frames */
   int FptrI = 0;
   int    pendingrcv;               /* pending data on the socket    */
   int i, rc, Fptr;
   int Fptr2[16] = {0};
   char ipv4addr[sizeof(struct in_addr)];

   //pthread_t thread;
   /* Read command line arguments */
   if (argc == 1) {
      printf("PU2: Error - Arguments missing\n\r");
      printf("\r   Valid arguments are:\n");
      printf("\r   -cchn {hostname}  : hostname of host running the 3705\n");
      printf("\r   -ccip {ipaddress} : ipaddress of host running the 3705 \n");
      printf("\r   -d : switch debug on  \n");
   return;
   }
   Tdbg_flag = OFF;
   i = 1;
   while (i < argc) {
      if (strcmp(argv[i], "-d") == 0) {
         Tdbg_flag = ON;
         printf("\rPU2: Debug on. Trace file is trace_3274.log\n");
         i++;
         continue;
      } else if (strcmp(argv[i], "-cchn") == 0) {
         if ( (lineent = gethostbyname(argv[i+1]) ) == NULL ) {
            printf("\rPU2: Cannot resolve hostname %s\n", argv[i+1]);
            return; /* error */
         }  // End if lineent
         printf("\rPU2: Connection to be established with 3705 SDLC line at host %s\n", argv[i+1]);
         i = i + 2;
         continue;
      } else if (strcmp(argv[i], "-ccip") == 0) {
         inet_pton(AF_INET, argv[i+1], ipv4addr);
         if ( (lineent = gethostbyaddr(&ipv4addr,sizeof(ipv4addr),AF_INET) ) == NULL ) {
            printf("\rPU2: Cannot resolve ip address %s\n", argv[i+1]);
            return; /* error */
         }  // End if lineent
         printf("\rPU2: Connection to be established with 3705 SDLC line at ip address %s\n", argv[i+1]);
         i = i + 2;
         continue;
      } else {
         printf("\rPU2: invalid argument %s\n",argv[i]);
         printf("\r     Valid arguments are:\n");
         printf("\r      -cchn {hostname}  : hostname of host running the 3705\n");
         printf("\r      -ccip {ipaddress} : ipaddress of host running the 3705 \n");
         printf("\r      -d : switch debug on  \n");
         return;
      }  // End else
   }  // End while

   // ********************************************************************
   //  Terminal controller debug trace facility
   // ********************************************************************
   if (Tdbg_flag == ON) {
      T_trace = fopen("trace_3274.log", "w");
      fprintf(T_trace, "     ****** 3274 Terminal Controller log file ****** \n\n"
                       "     i327x_3274 -d : trace all 3274 activities\n"
                       );
   }
   // SDLC line socket creation
   pusdlc_fd = socket(AF_INET, SOCK_STREAM, 0);
   if (pusdlc_fd <= 0) {
      printf("\rPU2: Cannot create line socket\n");
      return;
   }
   // Assign IP addr and PORT number
   servaddr.sin_family = AF_INET;
   memcpy(&servaddr.sin_addr, lineent->h_addr_list[0], lineent->h_length);
   // servaddr.sin_addr.s_addr = lineip;
   servaddr.sin_port = htons(SDLCLBASE);
   // Connect to the SDLC line socket
   printf("\rPU2: Waiting for SDLC line connection to be established\n");
   while (connect(pusdlc_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) != 0) {
      sleep(1);
   }
   printf("\rPU2: SDLC line connection has been established\n");
   // Now 'IML' the 3274
   rc = proc_PU2iml();
   FptrI = 0;
   while (1) {
      rc = proc_3270();
      rc = ioctl(pusdlc_fd, FIONREAD, &pendingrcv);
      if ((pendingrcv < 1) && (SocketReadAct(pusdlc_fd))) rc = -1;
      if (rc < 0) {                                      // Retry once to account for timing delays in TCP.
         if ((pendingrcv < 1) && (SocketReadAct(pusdlc_fd))) rc = -1;
      }
      if (rc < 0) {
         printf("\rPU2: SDLC line dropped, trying to re-establish connection\n");
         // SDLC line socket recreation
         close(pusdlc_fd);
         pusdlc_fd = socket(AF_INET, SOCK_STREAM, 0);
         if (pusdlc_fd <= 0) {
            printf("\rPU2: Cannot create line socket\n");
            return;
         }
         while (connect(pusdlc_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) != 0) {
            sleep(1);
         }  // End while
         printf("\rPU2: SDLC line connection has been re-established\n");
      } else {
         if (pendingrcv > 0) {
            SDLCreql = read(pusdlc_fd, SDLCreqb, pendingrcv);
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\r3274 Request Buffer (%d): ", SDLCreql);
               for (int i=0; i < SDLCreql; i ++) {
                  fprintf(T_trace, "%02X ", SDLCreqb[i]);
               }
               fprintf(T_trace, "\n");
               fflush(T_trace);
            }  // End if debug
            if ((SDLCreqb[FCntl] & 0x01) == IFRAME) {
               pu2[station]->seq_Nr++;                   // Update receive sequence number
               if (pu2[station]->seq_Nr == 8) pu2[station]->seq_Nr = 0;
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "\r3274 LH receive sequence count=%d, Fcntl=%02X\n", pu2[station]->seq_Nr, SDLCreqb[FCntl]);
            }  //End if SDLCreqb[FCntl]
            SDLCrspl = proc_PIU(SDLCreqb, SDLCreql, &SDLCrspb[SDLCrsptl]);
            if (SDLCrspl > 0) {
               Fptr2[FptrI] =  SDLCrsptl;
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "\r3274 Frame pointer index %d contains %d", FptrI, Fptr2[FptrI]);
               FptrI++;
               Fptr2[FptrI] =  0;
            }  // End if SDLCrspl
            SDLCrsptl = SDLCrsptl + SDLCrspl;
            if (Tdbg_flag == ON)
               fprintf(T_trace, "\r3274 Total response length: %d\n", SDLCrsptl);
            if (SDLCreqb[FCntl] & CPoll) {               // Poll command ?
               // Make sure the receive count is up-to-date before sending the repsonse.
               // First get the station address and replace the receive count in the Link Header
               FptrI = 0;
               Fptr = Fptr2[FptrI];                                //  First frame located at offset 0.
               do {
                  station =  (SDLCrspb[Fptr+FAddr] & 0x0F) - 1;
                  if ((SDLCrspb[Fptr+FCntl] & 0x03) == SUPRV) {      // Supervisory format ?
                     SDLCrspb[Fptr+FCntl] = (SDLCrspb[Fptr+FCntl] & 0x1F) | (pu2[station]->seq_Nr << 5);  // Insert receive sequence
                  }
                  if  ((SDLCrspb[Fptr+FCntl] & 0x01) == IFRAME) {
                     // Insert receive and send sequence numbers into the Frame Control byte of the response
                     SDLCrspb[Fptr+FCntl] = (SDLCrspb[Fptr+FCntl] & 0x1F) | (pu2[station]->seq_Nr << 5);  // Insert receive sequence
                     SDLCrspb[Fptr+FCntl] = (SDLCrspb[Fptr+FCntl] & 0xF1) | (pu2[station]->seq_Ns << 1);  // Insert send sequence
                     pu2[station]->seq_Ns++;             // Update send sequence number
                     if (pu2[station]->seq_Ns == 8) pu2[station]->seq_Ns = 0;
                  }  // End if (SDLCrspb[Fptr+FCntl] & 0x01)
                  if (Tdbg_flag == ON)
                     fprintf(T_trace, "\r3274 LH  Receive sequence=%d, Next send Sequence=%d, Fcntl=%02X\n",
                     pu2[station]->seq_Nr, pu2[station]->seq_Ns, SDLCrspb[Fptr+FCntl]);
                     FptrI++;                            // Move to next Frame pointer in the array
                     Fptr = Fptr2[FptrI];                // Get it
                  }  // End do
               while (Fptr != 0);                        // If the frame pointer is zero, there are no more frames
               // Now set the final bit in the last Frame.
               Fptr = Fptr2[FptrI-1];                    // Get the pointer to the last frame
               SDLCrspb[Fptr+FCntl] |= CFinal;           // Set the final bit;
               rc = send(pusdlc_fd, SDLCrspb, SDLCrsptl, 0);
               if (Tdbg_flag == ON) {
                  fprintf(T_trace, "\r3274 Response Buffer (%d): ", SDLCrsptl);
                  for (int i=0; i < SDLCrsptl; i ++) {
                     fprintf(T_trace, "%02X ", SDLCrspb[i]);
                  }
                  fprintf(T_trace, "\n");
                  fflush(T_trace);
               }  // End if debug
               SDLCrsptl = 0;                            // Reset response total length
               FptrI = 0;
            } else {
               // This part is required to avoid multiple frames being stacked into the tcp buffer,
               // which might happen because the sdlc code delivers frames faster than 3274 can process.
               rc = send(pusdlc_fd, SDLCrspb, 1, 0);     // Just send a single byte
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "\r3274 No poll bit, sending one byte for sync purposes");
            }  // End SDLCreqb[FCntl] & CPoll
         }  // End if (pendingrcv > 0)
      }  // End if (rc < 0)
   }  // End while (1)
   return;
}
