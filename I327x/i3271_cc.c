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

   i3271.C   (c) Copyright  Edwin Freekenhost & Henk Stegeman

   This module appears to the 3705/3720 as a 3271 cluster controller
   Depending on the defintion, 1 or more 3271's are 'IML-ed', making
   the defined LU's availble for sign-on. A terminal may connect to
   the desired 3271 by selecting the approriate telnet port number.
   The TCP ports are defined as 32711 for the first IBM3271, 32712 for
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
#include "i327x.h"
#include "ebcdic.h"
#include <errno.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>


#if !defined(min)
#define min(a,b)  (((a) <= (b)) ? (a) : (b))
#endif

#define BUFPD 0x1C
#define GPOLL 1
#define SPOLL 2
#define SELECT 3

uint16_t Tdbg_flag = OFF;          /* 1 when Ttrace.log open            */
FILE *T_trace;

struct sockaddr_in servaddr;
struct sockaddr_in    sin1, *sin2; /* bind socket address structure     */
struct epoll_event event, events[MAXCLSTR];
struct ifaddrs *nwaddr, *ifa;      /* interface address structure       */
in_addr_t  lineip;                 /* BSC line listening address        */
uint16_t   lineport;               /* BSC line listening port           */
int        clubsc_fd;
int        sockopt;                /* Used for setsocketoption          */
int        pendingrcv;             /* pending data on the socket        */
int        event_count;            /* # events received                 */
char       *ipaddr;
uint8_t     bfr[256];

struct CB327x *clu[MAXCLSTR];          /* 3271 control block */
struct IO3270 *ioblk[MAXCLSTR][MAXLU]; /* 3270 data buffer   */

uint8_t CLSTR_config[MAXCLSTR][4] = {{0x20,0x40, 0x40,0x40}};

uint8_t BSC_rbuf[65536];
uint8_t BSC_tbuf[65536];

// void *PU2_thread(void *arg);
void commadpt_read_tty(struct CB327x *i327x, struct IO3270 *ioblk, uint8_t * bfr, int lunum, int len);
int send_packet(int csock, uint8_t *buf, int len, char *caption);
int connect_client (int *csockp, BYTE i327xnump, BYTE *lunump, BYTE *lunumr);
int SocketReadAct (int fd);

uint8_t  ACKreq = 0;
uint8_t  NAKreq = 0;
uint8_t  EOTreq = 0;
uint8_t  lastACK = 1;                      // Value of last ACK
uint8_t  ENQ_type;                         // Type of ENQ.
                                           // 0 = Init xmit, 1 = GPOLL, 2 = Special POLL, 3 = Select
//uint8_t GPOLL[] = {
//      SYN, SYN, 0x40,0x40, 0x7F, 0x7F, ENQ, PAD}; // General POLL
//uint8_t SPOLL[] = {
//      SYN, SYN, 0x40,0x40, 0x40, 0x40, ENQ, PAD}; // Specific Poll
//uint8_t SELECT[] = {
//      SYN, SYN, 0x60,0x60, 0x40, 0x40, ENQ, PAD}; // Selection
uint8_t NAK_dlc[] = {
      SYN, SYN, NAK, PAD};                  // NAK
uint8_t EOT_dlc[] = {
      SYN, SYN, EOT, PAD};                  // EOT
uint8_t ENQ_dlc[] = {
      SYN, SYN, ENQ, PAD};                  // ENQ
uint8_t RVI_dlc[] = {
      SYN, SYN, DLE, 0x7C, PAD};            // RVI
uint8_t ACK0_dlc[] = {
      SYN, SYN, DLE, 0x70, PAD};            // ACK 0
uint8_t ACK1_dlc[] = {
      SYN, SYN, DLE, 0x61, PAD};            // ACK 1
uint8_t STX_dlc[] = {
      SYN, SYN, STX};                       // STX
uint8_t STX_addr[] = {
      SYN, SYN, STX, 0x40, 0x40};           // STX + CU addr
uint8_t SOH_stat[] = {
      SYN, SYN, SOH, 0x6C, 0xD9, STX, 0x40, 0x40}; // SOH (status) + STX
// Sense and Status uint8_ts
uint8_t SS_IR[] = { 0x40, 0x50 };        // Sense/Status : Intervention Required (Not Ready)
uint8_t SS_DE[] = { 0xC2, 0x40 };        // Sense/Status : Device End (Ready)
//int write_socket( int fd, const void *_ptr, int nuint8_ts );
//int send_packet (int csock, uint8_t *buf, int len, char *caption);

int double_up_iac (uint8_t *buf, int len);

/*-------------------------------------------------------------------*/
/* Function to calculate The Cyclic Redundancy Check                 */
/*-------------------------------------------------------------------*/
int crc16(unsigned char *ptr, int count)
{
   uint16_t crc;
   char i;

   crc = 0x0000;                         /* start value               */
   while (--count >= 0) {
      crc = crc ^ (unsigned int) *ptr++;
      i = 8;
      do {
         if (crc & 0x0001)
            crc = (crc >> 1) ^ CRC_POLY_16;
         else
            crc = (crc >> 1);
      } while(--i);
   }
   crc = (crc << 8) + (crc >> 8);        /* swap high and low bytes */
   return (crc);
}

/*-------------------------------------------------------------------*/
/* Process BSC message                                               */
/*-------------------------------------------------------------------*/
/*
   BSC text layout:
   +----------+-----+-----+-------//-------+-----+---+---+-----+
   | AA | SYN | SYN.| SOT |     Text       | EOT |  CRC  | PAD |
   +----------+-----+-----+-------//-------+-----+---+---+-----+

   BSC long text layout:
   +----------+-----+-----+-------//-----+-----+-----+-------//------+-----+----+---+-----+
   | AA | SYN | SYN.| SOT |     Text     | SYN | SYN |     Text      | EOT |  CRC   | PAD |
   +----------+-----+-----+-------//-----+-----+-----+-------//------+-----+----+---+-----+

   3271 General Poll:
   CU0:  40 40 7F 7F
   CU1:  C1 C1 7F 7F
   CU2:  C2 C2 7F 7F
   CU3:  C3 C3 7F 7F
   3271 Specific Poll:
   CU0 Terminal0: 40 40 40 40
   CU0 Terminal1: 40 40 C1 C1
   CU0 Terminal2: 40 40 C2 C2
   CU0 Terminal3: 40 40 C3 C3
   3271 Device Select:
   CU0 Terminal0: 60 60 40 40
   CU1 Terminal1: 61 61 C1 C1
   CU2 Terminal2: E2 E2 C2 C2
   CU3 Terminal3: E3 E3 C3 C3
*/
int proc_BSC(char* BSC_tbuf, uint16_t BSCtlen) {
   uint8_t  Dbuf[BUFLEN_3270];            // Data buffer
   uint16_t BSCrlen, Dlen;
   int  i, rc, ckesc;
   uint16_t CRCds, CRCck;


   if ((BSC_tbuf[0] == SYN) && (BSC_tbuf[1] == SYN)) {
      if (strncmp(BSC_tbuf, EOT_dlc, sizeof(EOT_dlc)) == 0)  {     // if EOT...
         BSCrlen = 0;
         if (Tdbg_flag == ON)
            fprintf(T_trace, "\r===> Received EOT...\n\r");
      }
      if (strncmp(BSC_tbuf, NAK_dlc, sizeof(NAK_dlc)) == 0)  {     // if NAK...
         BSCrlen = 0;
         if (Tdbg_flag == ON)
            fprintf(T_trace, "\r===> Received NAK...\n\r");
      }
      if ((strncmp(BSC_tbuf, ACK0_dlc, sizeof(ACK0_dlc)) == 0) ||  // if ACK0 or...
         (strncmp(BSC_tbuf, ACK1_dlc, sizeof(ACK1_dlc)) == 0)) {   // if ACK1...
      if (Tdbg_flag == ON)
         fprintf(T_trace, "\r===> Received ACK0/ACK1...\n\r");
      if (EOTreq == 1) {
         memcpy(&BSC_rbuf, EOT_dlc, sizeof(EOT_dlc));              // ...send EOT (nothing to send)
         BSCrlen = sizeof(EOT_dlc);
         EOTreq = 0;
         if (Tdbg_flag == ON)
            fprintf(T_trace, "\r===> Sending EOT...\n\r");         // Nothing more to send
         }
      }
      //***********************************************************
      // Check for init transmission ENQ
      //***********************************************************
      if (strncmp(BSC_tbuf, ENQ_dlc, sizeof(ENQ_dlc)) == 0) {      // if ENQ...
         if (Tdbg_flag == ON)
            fprintf(T_trace, "\r===> Received Init transmission ENQ...\n\r");
         lastACK = 1;                                              // Respond with ACK...
         ACKreq = 1;                                               // ...send ACK
      }
      //***********************************************************
      // Check for  POLL ENQ and determine type
      //***********************************************************
      if ((BSCtlen == 8) && (BSC_tbuf[6] == ENQ)) {                // If POLL ENQ received or
         if  (BSC_tbuf[2]  & 0x20) ENQ_type = SELECT;
         else ENQ_type = SPOLL;
         if ((BSC_tbuf[4] == 0x7F) && (BSC_tbuf[5] == 0x7F)) ENQ_type = GPOLL;

         switch (ENQ_type) {
            //***********************************************************
            // Select type ENQ
            //***********************************************************
            case SELECT:
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "\r===> Received Select...\n\r");
               if (clu[0]->lu_fd[0] > 0) {                         // ...and if terminal connected
                  lastACK = 1;                                     // A select should always responds with ACK0
                  ACKreq = 1;                                      // ...send ACK
               } else {                                            // If terminal not connected...
                  memcpy(&BSC_rbuf, RVI_dlc, sizeof(RVI_dlc));     // ...send RVI (want to send status/sense)
                  BSCrlen = sizeof(RVI_dlc);
               }
               break;

            //***********************************************************
            // Specific poll type ENQ
            //***********************************************************
            case SPOLL:
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "\r===> Received SPOLL...\n\r");
               if (clu[0]->lu_fd[0] > 0) {                         // ...and if terminal connected
                  ACKreq = 1;                                      // ...send ACK
               } else {                                            // If terminal not connected...
                  memcpy(BSC_rbuf, SOH_stat, sizeof(SOH_stat));    // ...then begin with SYN SYN SOH
                  BSCrlen = sizeof(SOH_stat);
                  memcpy(&BSC_rbuf[BSCrlen], SS_IR, sizeof(SS_IR ));  // ...and add IR sense
                  BSCrlen = BSCrlen + sizeof(SS_IR);
                  BSC_rbuf[BSCrlen++] = ETX;                       // add ETX
                  //***********************************************************
                  //* Calculate CRC
                  //************************************************************
                  CRCck = crc16(&BSC_rbuf[3], BSCrlen-3);          // Calculate CRC, Exclude SYN SYN SOH
                  BSC_rbuf[BSCrlen++] = CRCck >> 8;                // First CRC 16 byte
                  BSC_rbuf[BSCrlen++] = CRCck & 0x00FF;            // Second CRC 16 byte
                  BSC_rbuf[BSCrlen++] = PAD;                       // Line turnaround
                  EOTreq = 1;                                      // Send EOT after receiving an ACK,
                  clu[0]->not_ready[0] = 1;
                  if (Tdbg_flag == ON)
                    fprintf(T_trace, "\r===> Sending Sense data %02X %02X...\n\r", BSC_rbuf[8], BSC_rbuf[9] );
               }  // End if (clu[0]->lu_fd[0] > 0)
               break;

            //***********************************************************
            // General poll type ENQ
            //***********************************************************
            case GPOLL:
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "\r===> Received GPOLL...\n\r");
               if (clu[0]->lu_fd[0] > 0) {                         // If terminal connected
                  if ((ioblk[0][0]->inpbufl > 0) && !(clu[0]->not_ready[0])) {  // Do we have data to transmit and terminal is ready... ?
                     memcpy(BSC_rbuf, STX_addr, sizeof(STX_addr)); // ...then begin with SYN SYN STX
                     BSCrlen = sizeof(STX_addr);
                     memcpy(&BSC_rbuf[BSCrlen], ioblk[0][0]->inpbuf, ioblk[0][0]->inpbufl);  // ...add the 3270 buffer content
                     BSCrlen = BSCrlen + ioblk[0][0]->inpbufl;
                     BSC_rbuf[BSCrlen++] = ETX;                    // add ETX
                     //***********************************************************
                     //* Calculate CRC
                     //************************************************************
                     CRCck = crc16(&BSC_rbuf[3], BSCrlen-3);       // Calculate CRC, Exclude SYN SYN STX
                     BSC_rbuf[BSCrlen++] = CRCck >> 8;             // First CRC 16 byte
                     BSC_rbuf[BSCrlen++] = CRCck & 0x00FF;         // Second CRC 6 byte
                     BSC_rbuf[BSCrlen++] = PAD;                    // Line turnaround
                     if (Tdbg_flag == ON) {
                        fprintf(T_trace, "\r3270 Output Buffer: ");
                        for (int i = 0; i < ioblk[0][0]->inpbufl; i++) {
                           fprintf(T_trace, "%02X ", ioblk[0][0]->inpbuf[i]);
                        }
                        fprintf(T_trace, "\n\r");
                     }  // End if Debug
                     ioblk[0][0]->inpbufl = 0;
                     EOTreq = 1;                                   // Send EOT after receiving an ACK,
                  } else {
                     memcpy(&BSC_rbuf, EOT_dlc, sizeof(EOT_dlc));  // ... send EOT (nothing to send)
                     BSCrlen = sizeof(EOT_dlc);
                     if (Tdbg_flag == ON)
                        fprintf(T_trace, "\r===> Returning EOT (a)...\n\r");
                  }  // End if ioblk
                  if (clu[0]->not_ready[0]) {                      // If previous Not Ready state....
                     memcpy(BSC_rbuf, SOH_stat, sizeof(SOH_stat));   //...then begin with SYN SYN SOH
                     BSCrlen = sizeof(SOH_stat);
                     memcpy(&BSC_rbuf[BSCrlen], SS_DE, sizeof(SS_DE ));  //...and add DE sense
                     BSCrlen = BSCrlen + sizeof(SS_DE);
                     BSC_rbuf[BSCrlen++] = ETX;                    //  add ETX
                     //************************************************************
                     //* Calculate CRC
                     //************************************************************
                     CRCck = crc16(&BSC_rbuf[3], BSCrlen-3);       // Calculate CRC, Exclude SYN SYN STX
                     BSC_rbuf[BSCrlen++] = CRCck >> 8;             // First CRC 16 byte
                     BSC_rbuf[BSCrlen++] = CRCck & 0x00FF;         // Second CRC 16 byte
                     BSC_rbuf[BSCrlen++] = PAD;                    // Line turnaround
                     EOTreq = 1;                                   // Send EOT after receiving an ACK,
                     clu[0]->not_ready[0] = 0;                     // Reset not_ready state
                     if (Tdbg_flag == ON)
                        fprintf(T_trace, "\r===> Sending Sense data %02X %02X...\n\r", BSC_rbuf[8], BSC_rbuf[9] );
                  }  // End if (clu[0]->not_ready[0])
               } else {
                  memcpy(&BSC_rbuf, EOT_dlc, sizeof(EOT_dlc));     // ... send EOT (nothing to send)
                  BSCrlen = sizeof(EOT_dlc);
                  if (Tdbg_flag == ON)
                     fprintf(T_trace, "\r===> Returning EOT (b)...\n\r");
               }  // End if clu[0]->lu_fd[0]
            break;
         }  // End switch ENQ_type
      }  // End if BSCtlen == 8

      //***********************************************************
      //* ???
      //***********************************************************
      if (strncmp(BSC_tbuf, STX_dlc, sizeof(STX_dlc)) == 0) {       // if STX...
         if (Tdbg_flag == ON)
            fprintf(T_trace, "\r ===> Receiving %d bytes of data...\n\r", BSCtlen);
         // Get Dlen by searching for x'FF'. (PAD)
         // and copy data to the Dbuf.
         if ((BSC_tbuf[BSCtlen - 1] == PAD) && (BSC_tbuf[BSCtlen - 4] == ETX)) {  // If we have a complete record
            CRCds = (BSC_tbuf[BSCtlen - 3] << 8) + BSC_tbuf[BSCtlen - 2];
            if (Tdbg_flag == ON)
               fprintf(T_trace, "\r===> CRC = %04X\n\r", CRCds);
            // memcpy(&Dbuf, BSC_tbuf+3, BSCtlen-6);               // copy all data between STX and ETX
            // Dlen = BSCtlen-7;
            if (Tdbg_flag == ON)
              fprintf(T_trace, "\r===> Dlen=%d Dbuf[0]=%02X, Dbuf[Dlen-1]=%02X, BSCtlen=%d\n\r", Dlen, Dbuf[0], Dbuf[Dlen-1], BSCtlen );
            //***********************************************************
            //* Check CRC
            //***********************************************************
            // CRCck = crc16(Dbuf, Dlen);                          // Calculate CRC, Exclude IAC
            CRCck = crc16(BSC_tbuf+3, BSCtlen-6);                  // Calculate CRC, Exclude SOT header
            // Dbuf[Dlen++] = IAC;
            // Dbuf[Dlen++] = 0xEF;
            BSC_tbuf[BSCtlen-4] = IAC;
            BSC_tbuf[BSCtlen-3] = 0xEF;

            ckesc = 0;                                             // preset to not preceeded by ESC
            if (BSC_tbuf[3+ckesc] == 0x27) ckesc=1;                // If data begins with ESC exclude it
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\r3270 Input Buffer: ");
               for (int i = 0; i < BSCtlen-5-ckesc; i ++) {
                  fprintf(T_trace, "%02X ", BSC_tbuf[i+3+ckesc]);
               }
               fprintf(T_trace, "\n\r");
            }
            if ((CRCds ^ CRCck) == 0x0000) {
               //************************************************************
               rc = send_packet (clu[0]->lu_fd[0], (uint8_t *) BSC_tbuf+3+ckesc, BSCtlen-5-ckesc, "3270 BSC Data");
               //************************************************************
               if (rc == 0) ACKreq = 1;
                  else NAKreq = 1;
            } else {
               if (Tdbg_flag == ON)
                  fprintf(T_trace, "\rCLU: CRC error: actual: %04X Received: %04X\n\r", CRCds, CRCck);
               printf("\rCLU: CRC error: Received: %04X Calculated: %04X\n\r", CRCds, CRCck);
               NAKreq = 1;
            }  // End if CRCds
         } else {                                                  // Somethings wrong with the input record
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\r3270 Input Buffer: ");
               for (int i = 0; i < BSCtlen; i ++) {
                  fprintf(T_trace, "%02X ", BSC_tbuf[i]);
               }
               fprintf(T_trace, "\n\r");
            }
            NAKreq = 1;
         }  // End BSC_tbuf
      }  // End strncmp STX

      if (ACKreq == 1) {
         if (lastACK == 1) {
            memcpy(&BSC_rbuf, ACK0_dlc, sizeof(ACK0_dlc));         // send ACK
            BSCrlen = sizeof(ACK0_dlc);
            lastACK = 0;
            if (Tdbg_flag == ON)
               fprintf(T_trace, "\r===> Returning ACK0...\n\r");
         } else {
            memcpy(&BSC_rbuf, ACK1_dlc, sizeof(ACK1_dlc));         // send ACK
            BSCrlen = sizeof(ACK1_dlc);
            lastACK = 1;
            if (Tdbg_flag == ON)
               fprintf(T_trace, "\r===> Returning ACK1...\n\r");
         }  // End if lastACK
         ACKreq = 0;
      }  // End if ACKreq

      if (NAKreq == 1) {
         memcpy(&BSC_rbuf, NAK_dlc, sizeof(NAK_dlc));              // send NACK
         BSCrlen = sizeof(NAK_dlc);
         NAKreq = 0;
         if (Tdbg_flag == ON)
            fprintf(T_trace, "\r===> Returning NAK...\n\r");
      }  // End if NAKreq
   }  // End if BSC_buf SYN
   return BSCrlen;
}
// #####################################################################


/**********************************************************************/
/* Procedure to 'iml' the 3271                                        */
/**********************************************************************/
int proc_CLUiml() {
   for (BYTE j = 0; j < MAXCLSTR; j++) {
      clu[j] =  malloc(sizeof(struct CB327x));
      // Init sockets for LU's
      for (BYTE i = 0; i < MAXLU; i++) {
         clu[j]->lu_fd[i] = 0;
         clu[j]->not_ready[i] = 1;
      }
      clu[j]->lunum = 0;
      clu[j]->last_lu = 0;
      clu[j]->punum = j;
   }  // End for j = 0

   getifaddrs(&nwaddr);      /* get network address */
   for (ifa = nwaddr; ifa != NULL; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name, "lo")) {
         sin2 = (struct sockaddr_in *) ifa->ifa_addr;
         ipaddr = inet_ntoa((struct in_addr) sin2->sin_addr);
         if (strcmp(ifa->ifa_name, "eth")) break;
      }
   }
   printf("\rCLU: Using network Address %s on %s for 3270 connections\n", ipaddr, ifa->ifa_name);

   for (BYTE j = 0; j < MAXCLSTR; j++) {
      if ((clu[j]->pu_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1)
         printf("\rCLU: Endpoint creation for 3271 failed with error %s ", strerror(errno));
      /* Reuse the address regardless of any */
      /* spurious connection on that port    */
      sockopt=1;
      setsockopt(clu[j]->pu_fd, SOL_SOCKET, SO_REUSEADDR, (void*)&sockopt, sizeof(sockopt));
      /* Bind the socket */
      sin1.sin_family=AF_INET;
      sin1.sin_addr.s_addr = inet_addr(ipaddr);
      sin1.sin_port=htons(32711+j);
      if (bind(clu[j]->pu_fd, (struct sockaddr *)&sin1, sizeof(sin1)) < 0) {
          printf("\rCLU: Bind 3271-%01X socket failed\n\r", j);
          free(clu[j]);
          return -1;
      }
      /* Listen and verify */
      if ((listen(clu[j]->pu_fd, 10)) != 0) {
         printf("\rCLU: 3174-%01X socket listen failed %s\n\r", j, strerror(errno));
          free(clu[j]);
          return -1;
      }
      // Add polling events for the port
      clu[j]->epoll_fd = epoll_create(1);
      if (clu[j]->epoll_fd == -1) {
         printf("\rCLU: failed to created the 3271-%01X epoll file descriptor\n\r", j);
         free(clu[j]);
         return -3;
      }
      event.events = EPOLLIN;
      event.data.fd = clu[j]->pu_fd;
      if (epoll_ctl(clu[j]->epoll_fd, EPOLL_CTL_ADD, clu[j]->pu_fd, &event) == -1) {
         printf("\rCLU: Add polling event failed for 3271-%01X with error %s \n\r", j,  strerror(errno));
         close(clu[j]->epoll_fd);
         free(clu[j]);
         return -4;
      }
      printf("\rCLU: 3271-%01X IML ready. TN3270 can connect to port %d \n\r)", j,32711+j);
   }  // End for j=0
   return 0;
}

/********************************************************************/
/* Procedure to check for 3270 connection or data requests          */
/********************************************************************/
int proc_3270 () {
   int    rc;
   //
   // Poll briefly for connect requests. If a connect request is received,
   // proceed with connect/accept the request.
   // Next, check all active connection for input data.
   //
   for (BYTE k = 0; k < MAXCLSTR; k++) {
      event_count = epoll_wait(clu[k]->epoll_fd, events, MAXLU, 50);

      for (int i = 0; i < event_count; i++) {
         if (clu[k]->lunum != 0xFF) {                                            /* if avail LU pool not exhausted      */
            clu[k]->lu_fd[clu[k]->lunum]=accept(clu[k]->pu_fd, NULL, 0);
            if (clu[k]->lu_fd[clu[k]->lunum] < 1) {
               printf("\rCLU: accept failed for 3171-%01X %s\n", k, strerror(errno));
            } else {
               if (connect_client(&clu[k]->lu_fd[clu[k]->lunum], clu[k]->punum, &clu[k]->lunum, &clu[k]->lunumr))  {
                  clu[k]->is_3270[clu[k]->lunum] = 1;
               } else {
                  clu[k]->is_3270[clu[k]->lunum] = 0;
               }  // End if connect_client

               if (clu[k]->lunumr != clu[k]->lunum)  {                                 /* Requested terminal number is not the proposed terminal number   */
                  if (clu[k]->lu_fd[clu[k]->lunumr] < 1) {
                     clu[k]->lu_fd[clu[k]->lunumr] = clu[k]->lu_fd[clu[k]->lunum];     /* Copy fd to request lu                               */
                     clu[k]->is_3270[clu[k]->lunumr] = clu[k]->is_3270[clu[k]->lunum]; /* copy 3270 indicator                                 */
                     clu[k]->lu_fd[clu[k]->lunum] = 0;                                 /* clear fd in proposed lu number                      */
                     clu[k]->is_3270[clu[k]->lunum] = 0;                               /* clear 3270 indicator for proposed lu number         */
                     clu[k]->lunum = clu[k]->lunumr;                                   /* replace proposed lu number with requested lu number */
                  } else {
                     printf("\rCLU: requested terminal port %02X is not available, request denied\n", clu[k]->lunumr);
                  }  // End  if (pu2[k]->lu_fd[pu2[k]->lunumr]
               }  // End if pu[k]->lunumr

               ioblk[k][clu[k]->lunum] =  malloc(sizeof(struct IO3270));
               ioblk[k][clu[k]->lunum]->inpbufl = 0;                             /* make sure the initial length is 0 */
               clu[k]->daf_addr1[clu[k]->lunum] = 0;                             /* make sure the initial value is 0 */
               clu[k]->bindflag[clu[k]->lunum] = 0;                              /* make sure the initial value is 0 */
               clu[k]->initselfflag[clu[k]->lunum] = 0;                          /* make sure the initial value is 0 */
               clu[k]->not_ready[clu[k]->lunum] = 0;                             /* Reset not-ready state            */
               printf("\rCLU: terminal %d connected to 3271-%01X\n", clu[k]->lunum, k);
               // Next available terminal
               clu[k]->lunum = clu[k]->lunum + 1;
               // Find first available LU
               clu[k]->lunum = 0xFF;                                              /* preset to no LU's availble         */
               for (BYTE j = 0; j < MAXLU; j++) {
                  if (clu[k]->lu_fd[j] < 1) clu[k]->lunum = j;
               } // end for BYTE j
               if (clu[k]->lunum == 0xFF) printf("\rCLU: No more terminal ports available. New connections rejected until a terminal port is released;\n");
            }  // End if clu[j]->lu_fd
         }  // End if (pu2[k] != 0xFF)
      }  // End for int i

      for (BYTE j = 0; j < MAXLU; j++) {
         if (clu[k]->lu_fd[j] > 0) {
            rc = ioctl(clu[k]->lu_fd[j], FIONREAD, &pendingrcv);
            if ((pendingrcv < 1) && (SocketReadAct(clu[k]->lu_fd[j]))) rc = -1;

            if (rc < 0) {
               clu[k]->not_ready[j] = 1;
               free(ioblk[k][j]);
               clu[k]->actlu[j] = 0;
               close (clu[k]->lu_fd[j]);
               clu[k]->lu_fd[j] = 0;
               printf("\rCLU: terminal %d disconnected from 3271-%01X\n", j, k);
               if ((clu[k]->lunum > j) || (clu[k]->lunum == 0xFF))  /* If next available terminal greater or no terminals availble ... */
                   clu[k]->lunum = j;                               /* ...replace with the just released terminal number          */
            } else {
               if (pendingrcv > 0) {
                  rc=read(clu[k]->lu_fd[j], bfr, 256-BUFPD);
                  //******
                  if (Tdbg_flag == ON) {
                     fprintf(T_trace, "\n3270 Read Buffer: ");
                     for (int i = 0; i < rc; i ++) {
                        fprintf(T_trace, "%02X ", bfr[i]);
                     }
                     fprintf(T_trace, "\n\r");
                  } // End if debug
                  //******
                  commadpt_read_tty(clu[k], ioblk[k][j], bfr, j, rc);
               }  // End if pendingrcv
            }  // End if rc < 0
         }  // End if bsc-lu_fd
      }  // End for int j
   }  // End for int k
   return 0;

}

void main(int argc, char *argv[]) {
   unsigned long inaddr;
   struct hostent *lineent;
   uint16_t BSCrlen;               /* Buffer size of received data      */
   uint16_t BSCtlen;               /* Buffer size of transmitted data   */
   int    pendingrcv;              /* pending data on the socket        */
   int    i, rc;
   char ipv4addr[sizeof(struct in_addr)];

   /* Read command line arguments */
   if (argc == 1) {
      printf("\rCLU: Error - arguments missing(s)!\n");
      printf("\r     Usage: i3271 [-cchn hostname | -ccip ipaddr]\n");
      printf("\r                  [-d]\n\n");
      return;
   }
   Tdbg_flag = OFF;
   i = 1;
   while (i < argc) {
      if (strcmp(argv[i], "-d") == 0) {
         Tdbg_flag = ON;
         printf("\rCLU: Debug on. Trace file is trace_3271.log\n");
         i++;
         continue;
      } else if (strcmp(argv[i], "-cchn") == 0) {
         if ( (lineent = gethostbyname(argv[i+1]) ) == NULL ) {
            printf("\rCLU: Cannot resolve hostname %s\n", argv[i+1]);
            return;                /* error */
         }  // End if lineent
         printf("\rCLU: Connection to be established with 3705 BSC line at host %s\n", argv[i+1]);
         i = i + 2;
         continue;
      } else if (strcmp(argv[i], "-ccip") == 0) {
         inet_pton(AF_INET, argv[i+1], ipv4addr);
         if ( (lineent = gethostbyaddr(&ipv4addr,sizeof(ipv4addr),AF_INET) ) == NULL ) {
            printf("\rCLU: Cannot resolve ip address %s\n", argv[i+1]);
            return; /* error */
         }  // End if lineent
         printf("\rCLU: Connection to be established with 3705 BSC line at ip address %s\n", argv[i+1]);
         i = i + 2;
         continue;
      } else {
         printf("\rCLU: Error - invalid argument %s!\n", argv[i]);
         printf("\r     Usage: i3271 [-cchn hostname | -ccip ipaddr]\n");
         printf("\r                  [-d]\n\n");
         return;
      }  // End else
   }  // End while

   // ********************************************************************
   // Terminal controller debug trace facility
   // ********************************************************************
   if (Tdbg_flag == ON) {
      T_trace = fopen("trace_3271.log", "w");
      fprintf(T_trace, "     ****** 3271 Terminal Controller log file ****** \n\n"
                       "     i327x_3271 -d : trace all 3271 activities\n"
                       );
   }
   // BSC line socket creation
   clubsc_fd = socket(AF_INET, SOCK_STREAM, 0);
   if (clubsc_fd <= 0) {
      printf("\rCLU: Cannot create line socket\n");
      return;
   }
   // Assign IP addr and PORT number
   servaddr.sin_family = AF_INET;
   memcpy(&servaddr.sin_addr, lineent->h_addr_list[0], lineent->h_length);
   servaddr.sin_port = htons(BSCLBASE);
   // Connect to the BSC line socket
   printf("\rCLU: Waiting for BSC line connection to be established\n");
   while (connect(clubsc_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) != 0) {
      sleep(1);
   }
   printf("\rCLU: BSC line connection has been established\n");
   // Now 'start' the 3271
   rc = proc_CLUiml();
   while (1) {
      rc = proc_3270();
      rc = ioctl(clubsc_fd, FIONREAD, &pendingrcv);
      if ((pendingrcv < 1) && (SocketReadAct(clubsc_fd))) rc = -1;
      if (rc < 0) {            // Retry once to account for timing delays in TCP.
         if ((pendingrcv < 1) && (SocketReadAct(clubsc_fd))) rc = -1;
      }
      if (rc < 0) {
         printf("\rCLU: BSC line dropped, trying to re-establish connection\n");
         // BSC line socket recreation
         close(clubsc_fd);
         clubsc_fd = socket(AF_INET, SOCK_STREAM, 0);
         if (clubsc_fd <= 0) {
            printf("\rCLU: Cannot create line socket\n");
            return;
         }
         while (connect(clubsc_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) != 0) {
            sleep(1);
         }  // End while
      } else {
         if (pendingrcv > 0) {
            BSCtlen = read(clubsc_fd, BSC_tbuf, sizeof(BSC_tbuf));
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\r3271 Receive Buffer: ");
               for (int i = 0; i < BSCtlen; i ++) {
                  fprintf(T_trace, "%02X ", BSC_tbuf[i]);
               }
               fprintf(T_trace, "\n");
            } // End if debug
            BSCrlen = proc_BSC(BSC_tbuf, BSCtlen);
            if (BSCrlen == 0) BSCrlen = 1;     // If no response needed, send 1 byte
            rc = send(clubsc_fd, BSC_rbuf, BSCrlen, 0);
            if (Tdbg_flag == ON) {
               fprintf(T_trace, "\r3271 Send Buffer: ");
               for (int i = 0; i < BSCrlen; i ++) {
                  fprintf(T_trace, "%02X ", BSC_rbuf[i]);
               }
               fprintf(T_trace, "\n\r");
               fflush(T_trace);
            }  // End if debug
         }  // End if (pendingrcv > 0)
      }  // End if (rc < 0)
   }  // End while (1)
   return;
}
