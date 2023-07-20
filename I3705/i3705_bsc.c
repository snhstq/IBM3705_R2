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

   i3705_bsc.c: This module contains the IBM 3705 BSC line function

   BSC text layout:
   +----------+-----+-----+-------//-------+-----+---+---+-----+
   | AA | SYN | SYN.| SOT |     Text       | EOT |  CRC  | PAD |
   +----------+-----+-----+-------//-------+-----+---+---+-----+

   BSC long text layout:
   +----------+-----+-----+-------//-----+-----+-----+-------//------+-----+----+---+-----+
   | AA | SYN | SYN.| SOT |     Text     | SYN | SYN |     Text      | EOT |  CRC   | PAD |
   +----------+-----+-----+-------//-----+-----+-----+-------//------+-----+----+---+-----+
*/

#include <stdbool.h>
#include "sim_defs.h"
#include "i3705_defs.h"
#include <ifaddrs.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MAXBSCLINES     2              /* Maximum of lines          */
#define BUFLEN_3271     65536          /* 3271 Send/Receive buffer  */
#define LINEBASE        30             /* BSC lines start at 30     */

#define SYN 0x32

struct BSCLine {
   int      line_fd;
   int      linenum;
   int      d3271_fd;
   int      epoll_fd;
   uint8_t  BSC_rbuf[BUFLEN_3271];     // Received data buffer
   uint8_t  BSC_tbuf[BUFLEN_3271];     // Transmit data buffer
   uint16_t BSCrlen;                   // Size of received data in buffer
   uint16_t BSCtlen;                   // Size of transmit data in buffer
} *bscline[MAXBSCLINES];

int proc_BSC(char* BSC_tbuf, uint16_t BSCtlen);
void proc_BSCtdata (unsigned char BSCchar, uint8_t state);    // BSC character handler

extern FILE *trace;
extern int8 debug_reg;

int8 BSCsync;                          // Track receive progress
int8 station;                          // Station #

//*********************************************************************
//  Shift all chacters in an array yo the left.                       *
//*********************************************************************
int ShiftLeft(char *buffer, int len)  {
   int i;
   for (i = 1; i < len; i++) {
      buffer[i - 1] = buffer[i];
   }
   len--;
   return len;
}

//*********************************************************************
// Check if there is read activiy on the socket                       *
// This is used by the caller to detect a connection break            *
//*********************************************************************
int SocketReadAct(int fd) {
   int     rc;
   fd_set  fdset;
   struct  timeval timeout;
   timeout.tv_sec = 0;
   timeout.tv_usec = 0;
   FD_ZERO(&fdset);
   FD_SET(fd, &fdset);
   return select(fd + 1, &fdset, NULL, NULL,  &timeout);
}

//*********************************************************************
// Function to check if socket is (still) connected                   *
//*********************************************************************
static bool IsSocketConnected(int sockfd) {
   int rc;
   struct sockaddr_in peer_addr;
   int addrlen = sizeof(peer_addr);
   if (sockfd < 1) {
      return false;
   }
   rc = getpeername(sockfd, (struct sockaddr*)&peer_addr, &addrlen);
   if (rc != 0) {
      return false;
   }
   return true;
}

//*********************************************************************
// Receive data from the 3271                                         *
// If an error occurs, the connection will be closed                  *
//*********************************************************************
int  ReadBSC(int k) {
   int rc;
   if (IsSocketConnected(bscline[k]->d3271_fd)) {
      bscline[k]->BSCrlen = read(bscline[k]->d3271_fd, bscline[k]->BSC_rbuf, BUFLEN_3271);
      if (bscline[k]->BSCrlen == 1)
         bscline[k]->BSCrlen = 0;    // Received 1 byte means Reset received data length.
      if ((debug_reg & 0x40) && (bscline[k]->BSCrlen > 0)) {
         fprintf(trace, "\n3271 Read Buffer: ");
         for (int i = 0; i < bscline[k]->BSCrlen; i ++) {
            fprintf(trace, "%02X ", bscline[k]->BSC_rbuf[i]);
         }
         fprintf(trace, "\n\r");
      }  // End if debug_reg
      return 0;
   } else {
      close (bscline[k]->d3271_fd);
      bscline[k]->d3271_fd = 0;
     printf("\rBSC: 3271 disconnected from line-%d\n", k);
   }  // End if IsSocketConnected
   return -1;
}

//*********************************************************************
// Receive data from the 3271                                         *
// If an error occurs, the connection will be closed                  *
//*********************************************************************
int ReadBSC2(int k) {
   int rc;
   int pendingrcv;              /* pending data on the socket        */
   if (bscline[k]->d3271_fd > 0) {
      while(1) {
         rc = ioctl(bscline[k]->d3271_fd, FIONREAD, &pendingrcv);
         if ((pendingrcv < 1) && (SocketReadAct(bscline[k]->d3271_fd))) rc = -1;
         if (rc < 0) {
            close (bscline[k]->d3271_fd);
            bscline[k]->d3271_fd = 0;
            printf("\rBSC: 3271 disconnected from line-%d\n", k);
            return -1;
         } else {
            if (pendingrcv > 0) {
               bscline[k]->BSCrlen = read(bscline[k]->d3271_fd, bscline[k]->BSC_rbuf, BUFLEN_3271);
               if ( bscline[k]->BSCrlen == 1)
                  bscline[k]->BSCrlen = 0;    // Received 1 byte meansReset received data length.
               if ((debug_reg & 0x40) && (bscline[k]->BSCrlen > 0)) {
                  fprintf(trace, "\n3271 Read Buffer: ");
                  for (int i = 0; i < bscline[k]->BSCrlen; i ++) {
                     fprintf(trace, "%02X ", bscline[k]->BSC_rbuf[i]);
                  }
               fprintf(trace, "\n\r");
               } // End if debug_reg
               return 0;
            }  // End if pendingrcv
         }  // End if rc < 0
      }  // End while(1)
   }  // End if bsc-d3271_fd
   return -1;
}

//*********************************************************************
//   Transmitted Character from scanner                               *
//*********************************************************************
void proc_BSCtdata (unsigned char BSCtchar, uint8_t state) {
uint8 rc;

   // State C means end of transmission, send buffer to cluster controller.
   if ((BSCsync == 1) && (state == 0xC)) {
      BSCsync = 0;                             // Reset SYNC.
      rc = send(bscline[0]->d3271_fd, bscline[0]->BSC_tbuf, bscline[0]->BSCtlen, 0);
      bscline[0]->BSCtlen = 0;                 // Reset transmitted data length.
      rc = ReadBSC(0);
   }  // End if state

   // If we are in receive mode, append the character to the buffer
   if (BSCsync == 1) {
      bscline[0]->BSC_tbuf[bscline[0]->BSCtlen] = BSCtchar;            // Add character to buffer
      bscline[0]->BSCtlen++;                               // Increment length
   }  // End if BSCsync

   // Check if we received a synchronization character. This indicates the start of a transmission
   if ((BSCtchar == 0xAA) && (state == 0x8)) {
      BSCsync = 1;                             // Indicate we are in receive mode
      bscline[0]->BSCtlen = 0;                 // Ensure length is set to zero
   }  // End if BSCtchar == 0xAA

   // Check if we received two consequtive SYN characters in the text;...
   // ...these are time-fill sync and must be removed
   if ((bscline[0]->BSCtlen > 3) && (BSCtchar == SYN) && (bscline[0]->BSC_tbuf[bscline[0]->BSCtlen-2] == SYN))  {
      bscline[0]->BSCtlen = bscline[0]->BSCtlen - 2;
   }  // End if BSCtlen
   return;                                     // back to schanner
}

//*********************************************************************
//   Received Character for scanner                                   *
//*********************************************************************
int  proc_BSCrdata (unsigned char *BSCrchar) {
uint8 rc;
   rc = 0;                                     // preset to n o characters to transmit
   if (bscline[0]->BSCrlen > 0) {
      *BSCrchar = bscline[0]->BSC_rbuf[0];
      bscline[0]->BSCrlen = ShiftLeft(bscline[0]->BSC_rbuf, bscline[0]->BSCrlen);
      rc = 1;                                  // One character to transmit
   }
   return rc;                                  // back to schanner
}

//*********************************************************************
//   Thread to handle connections from the 3271 cluster emulator      *
//*********************************************************************
void *BSC_thread(void *arg)
{
   int    devnum;                  /* device nr copy for convenience */
   int    sockopt;                 /* Used for setsocketoption       */
   int    pendingrcv;              /* pending data on the socket     */
   int    event_count;             /* # events received              */
   int    rc, rc1;                 /* return code from various rtns  */
   struct sockaddr_in sin, *sin2;  /* bind socket address structure  */
   struct ifaddrs *nwaddr, *ifa;   /* interface address structure    */
   char   *ipaddr;
   struct epoll_event event, events[MAXBSCLINES];

   printf("\rBSC: Thread %ld started succesfully...\n", syscall(SYS_gettid));

   for (int j = 0; j < MAXBSCLINES; j++) {
      bscline[j] =  malloc(sizeof(struct BSCLine));
      bscline[j]->linenum = j;
      bscline[j]->BSCrlen = 0;
      bscline[j]->BSCtlen = 0;
   }  // End for j = 0
   getifaddrs(&nwaddr);      /* get network address */
   for (ifa = nwaddr; ifa != NULL; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name, "lo")) {
         sin2 = (struct sockaddr_in *) ifa->ifa_addr;
         ipaddr = inet_ntoa((struct in_addr) sin2->sin_addr);
         if (strcmp(ifa->ifa_name, "eth")) break;
      }
   }
   printf("\nBSC: Using network Address %s on %s for 3271 connections\n", ipaddr, ifa->ifa_name);

   for (int j = 0; j < MAXBSCLINES; j++) {
      if ((bscline[j]->line_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1)
         printf("\nBSC: Endpoint creation for 3271 failed with error %s ", strerror(errno));
      /* Reuse the address regardless of any */
      /* spurious connection on that port    */
      sockopt = 1;
      setsockopt(bscline[j]->line_fd, SOL_SOCKET, SO_REUSEADDR, (void*)&sockopt, sizeof(sockopt));
      /* Bind the socket */
      sin.sin_family = AF_INET;
      sin.sin_addr.s_addr = inet_addr(ipaddr);
      sin.sin_port = htons(37500 + LINEBASE + j);        // <=== port related to line number
      if (bind(bscline[j]->line_fd, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
          printf("\nBSC: Bind line-%d socket failed\n\r", j);
          free(bscline[j]);
          exit(EXIT_FAILURE);
      }
      /* Listen and verify */
      if ((listen(bscline[j]->line_fd, 10)) != 0) {
         printf("\nBSC: Line-%d Socket listen failed %s\n\r", j, strerror(errno));
          free(bscline[j]);
          exit(-1);
      }
      // Add polling events for the port
      bscline[j]->epoll_fd = epoll_create(1);
      if (bscline[j]->epoll_fd == -1) {
         printf("\nBSC: failed to created the line-%d epoll file descriptor\n\r", j);
         free(bscline[j]);
         exit(-2);
      }
      event.events = EPOLLIN;
      event.data.fd = bscline[j]->line_fd;
      if (epoll_ctl(bscline[j]->epoll_fd, EPOLL_CTL_ADD, bscline[j]->line_fd, &event) == -1) {
         printf("\nBSC: Add polling event failed for line-%d with error %s \n\r", j, strerror(errno));
         close(bscline[j]->epoll_fd);
         free(bscline[j]);
         exit(-3);
      }
      printf("\rBSC: line-%d ready, waiting for connection on TCP port %d\n\r", j, 37500 + LINEBASE + j );
   }
   //
   // Poll briefly for connect requests. If a connect request is received, proceed with connect/accept the request.
   // Next, check all active connection for input data.
   //
   while (1) {
      for (int k = 0; k < MAXBSCLINES; k++) {
         event_count = epoll_wait(bscline[k]->epoll_fd, events, 1, 50);
         for (int i = 0; i < event_count; i++) {
            bscline[k]->d3271_fd = accept(bscline[k]->line_fd, NULL, 0);
            if (bscline[k]->d3271_fd < 1) {
               printf("\nBSC: accept failed for line-%d %s\n", k, strerror(errno));
            } else {
               printf("\rBSC: 3271 connected to line-%d\n", k);
            } // End if bscline[k]->d3271_fd
         }  // End for int i
      }  // End for int k
   }  // End while(1)

    return NULL;
}

