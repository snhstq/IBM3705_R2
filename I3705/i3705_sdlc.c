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

   i3705_sdlc.c: IBM 3720 SDLC Primary Station simulator

   SLDC frame
    <-------------------------------- BLU ----------------------------->
   layout:         |   FCntl   |
   +-------+-------+-----------+-------//-------+-------+-------+-------+
   | BFlag | FAddr |Nr|PF|Ns|Ft| ... Iframe ... | Hfcs  | Lfcs  | EFlag |
   +-------+-------+-----------+-------//-------+-------+-------+-------+
*/

#include <stdbool.h>
#include "sim_defs.h"
#include "i3705_defs.h"
#include "../Include/i327x_sdlc.h"
#include <ifaddrs.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define MAXSDLCLINES     2             /* Maximum of lines          */
#define LINEBASE        20             /* SDLC lines start at 20    */


struct SDLCLine {
   int      line_fd;
   int      linenum;
   int      d3274_fd;
   int      epoll_fd;
   uint8_t  SDLC_rbuf[BUFLEN_3274];    // Received data buffer
   uint8_t  SDLC_tbuf[BUFLEN_3274];    // Transmit data buffer
   uint16_t SDLCrlen;                  // Size of received data in buffer
   uint16_t SDLCtlen;                  // Size of transmit data in buffer
} *sdlcline[MAXSDLCLINES];

extern FILE *S_trace;
extern uint16_t Sdbg_reg;
extern uint16_t Sdbg_flag;

// Host ---> PU request buffer
extern uint8 BLU_req_buf[BUFLEN_3274]; // DLC header + TH + RH + RU + DLC trailer
extern int   BLU_req_ptr;              // Offset pointer to BLU
extern int   BLU_req_len;              // Length of BLU request
extern int   BLU_req_stat;             // BLU buffer state
// PU ---> Host response buffer
extern uint8 BLU_rsp_buf[BUFLEN_3274]; // DLC header + TH + RH + RU + DLC trailer
extern int   BLU_rsp_ptr;              // Offset pointer to BLU
extern int   BLU_rsp_len;              // Length of BLU response
extern int   BLU_rsp_stat;             // BLU buffer state

int8 stat_mode = NDM;
int8 rxtx_dir = RX;                    // Rx or Tx flag
int8 station;                          // Station #

int proc_BLU(unsigned char BLU_req_buf[], int Blen);   // SDLC frame handler
int proc_frame(unsigned char BLU_req_buf[], int Blen); // Process frame header
int proc_PIU(unsigned char PIU_buf[], int Blen, int Ftype);   // PIU handler
void trace_Fbuf(unsigned char BLU_buf[], int Blen, int rxtx_dir);   // Print trace records
void prt_BLU_buf(int reqorrsp);

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
// Receive data from the 3274                                         *
// If an error occurs, the connection will be closed                  *
//*********************************************************************
int  ReadSDLC(int k, unsigned char *BLU_buf) {
   int rc;
   if (sdlcline[k]->d3274_fd > 0) {                      // Should we have a connection?
      if (IsSocketConnected(sdlcline[k]->d3274_fd)) {
         sdlcline[k]->SDLCrlen = read(sdlcline[k]->d3274_fd, BLU_buf, BUFLEN_3274);
         //******
         if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04) && (sdlcline[k]->SDLCrlen > 0)) {
            fprintf(S_trace, "\nSDLC: PU response Read Buffer: ");
            for (int i = 0; i < sdlcline[k]->SDLCrlen; i++) {
               fprintf(S_trace, "%02X ", BLU_buf[i]);
            }
            fprintf(S_trace, "\n\r");
         }  // End if debug_reg
         return sdlcline[k]->SDLCrlen;
      } else {
         close (sdlcline[k]->d3274_fd);
         sdlcline[k]->d3274_fd = 0;
         printf("\rSDLC: PU disconnected from line-%d\n", k);
      }  // End if IsSocketConnected
   }  // End  if (sdlcline[k]->d3274_fd > 0)
   return -1;
}

//*********************************************************************
//   Incomming SDLC frame (BLU) handler
//*********************************************************************
//int proc_BLU (unsigned char BLU_req_buf[], int BLU_req_len, int8 CS_line_addr, int8 CS_line_LCD) {
int proc_BLU (unsigned char BLU_req_buf[], int BLU_req_len) {
   register char *s;
   int Pflag = FALSE;                  // SDLC Poll bit flag
   int direction = TX;                 // TX or RX
   int Fptr, frame_len;
   int i, rc;

   if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {         // Trace BLU activities ?
      fprintf(S_trace, "\nSDLC: Received %d bytes from scanner.\nSDLC: Request Buffer: ", BLU_req_len);
      for (i = 0; i < BLU_req_len; i++)
         fprintf(S_trace, "%02X ", (int) BLU_req_buf[i] & 0xFF);
      fprintf(S_trace, "\n");
   }
   // Search for SDLC frames, process it and when Poll bit found: return.
   Pflag = FALSE;
   Fptr = 0;
   if ((BLU_req_buf[Fptr] == 0x00) || (BLU_req_buf[Fptr] == 0xAA)) Fptr = 1;  // If modem clocking is used skip first char
   if ((BLU_req_buf[Fptr] == 0x7E) && (BLU_req_buf[Fptr+1] == 0x7E) && (BLU_req_buf[Fptr+2] == 0x7E)) return 0; // Consequtive 7E's. Ignore.
   do {                                // Do till Poll bit found...
      frame_len = 0;                   //
      // Find end of SDLC frame...
      while (!((BLU_req_buf[Fptr + frame_len + 0] == 0x47) &&
               (BLU_req_buf[Fptr + frame_len + 1] == 0x0F) &&
               (BLU_req_buf[Fptr + frame_len + 2] == 0x7E))) {
         frame_len++;
      } // End while
      frame_len = frame_len + 3;              // Correction length LT
      if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04)) {  // Trace BLU activities ?
         fprintf(S_trace, "\nSDLC: sending request to PU: Frame Length=%d \nSDLC: Request Buffer:  ", frame_len);
         for (i = 0; i < frame_len; i++)
            fprintf(S_trace, "%02X ", (int) BLU_req_buf[Fptr + i] & 0xFF);
         fprintf(S_trace, "\n");
      }
      // ******************************************************************
      if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04))        // Trace BLU activities ?
         trace_Fbuf(BLU_req_buf + Fptr, frame_len, TX);  // Print trace records
      rc = send(sdlcline[0]->d3274_fd, &BLU_req_buf[Fptr], frame_len, 0);
      if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04))        // Trace BLU activities ?
         fprintf(S_trace, "\nSDLC: Sent %d bytes to PU, rc=%d\n ", frame_len, rc);
      BLU_rsp_len = ReadSDLC(0, BLU_rsp_buf);
      if (BLU_req_buf[Fptr + 2] & 0x10) {                // if Poll bit on we should have an answer
         if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04))     // Trace BLU activities ?
            trace_Fbuf(BLU_rsp_buf, BLU_rsp_len, RX);    // Print trace records
      }  else {
         BLU_rsp_len = 0;                                // If poll bit not on, we received a single sync byte which can be ignored
      }
      // ******************************************************************
      Fptr = Fptr + frame_len;
      if ((BLU_req_buf[Fptr] == 0x00) || (BLU_req_buf[Fptr] == 0xAA)) Fptr++; // If modem clocking is used skip first char
   }  // End Do
   while (Fptr < BLU_req_len);

   if ((Sdbg_flag == ON) && (Sdbg_reg & 0x04))           // Trace BLU activities ?
      fprintf(S_trace, "\nSDLC: Returning %d bytes to scanner ", BLU_rsp_len);
   if (BLU_rsp_len > 0) BLU_rsp_stat = FILLED;           // Indicate there is data in the response buffer
   return BLU_rsp_len;                                   // With response in BLU
}

//*********************************************************************
//   Print trace records of frame buffer (Fbuf)                       *
//*********************************************************************
void trace_Fbuf(uint8_t BLU_req_buf[], int Flen, int rxtx_dir) {
   register char *s;
   int i;

   if (rxtx_dir == TX)
      fprintf(S_trace, "SDLC: ==> ");   // TX 3705 -> client
   else
      fprintf(S_trace, "SDLC: <== ");   // RX 3705 <- client

   switch (BLU_req_buf[FCntl] & 0x03) {
      case UNNUM:
         // *** UNNUMBERED FORMAT ***
         switch (BLU_req_buf[FCntl] & 0xEF) {
            case SNRM:
               fprintf(S_trace, "SNRM - PF=%d \n", (BLU_req_buf[FCntl] >> 4) & 0x1);
               break;

            case DISC:
               fprintf(S_trace, "DISC - PF=%d \n", (BLU_req_buf[FCntl] >> 4) & 0x1);
               break;

            case UA:
               fprintf(S_trace, "UA   - PF=%d \n", (BLU_req_buf[FCntl] >> 4) & 0x1);
               break;

            case DM:
               fprintf(S_trace, "DM   - PF=%d \n", (BLU_req_buf[FCntl] >> 4) & 0x1);
               break;

            case FRMR:
               fprintf(S_trace, "FRMR - PF=%d \n", (BLU_req_buf[FCntl] >> 4) & 0x1);
               break;

            case TEST:
               fprintf(S_trace, "TEST - PF=%d \n", (BLU_req_buf[FCntl] >> 4) & 0x1);
               break;

            case XID:
               fprintf(S_trace, "XID  - PF=%d \n", (BLU_req_buf[FCntl] >> 4) & 0x1);
               break;

            default:
               fprintf(S_trace, "ILLEGAL - ");
               for (s = (char *) BLU_req_buf, i = 0; i < 6; ++i, ++s)
                  fprintf(S_trace, "%02X ", (int) *s & 0xFF);
               fprintf(S_trace, "\n");
               break;
         }  // End of BLU_req_buf[FCntl] & 0x03
         break;

      case SUPRV:
         // *** SUPERVISORY FORMAT ***
         switch (BLU_req_buf[FCntl] & 0x0F) {
            case RR:
               fprintf(S_trace, "SDLC: RR   - N(r)=%d, PF=%d \n",
                      (BLU_req_buf[FCntl] >> 5) & 0x7,
                      (BLU_req_buf[FCntl] >> 4) & 0x1 );
               break;

            case RNR:
               fprintf(S_trace, "SDLC: RNR  - N(r)=%d, PF=%d \n",
                      (BLU_req_buf[FCntl] >> 5) & 0x7,
                      (BLU_req_buf[FCntl] >> 4) & 0x1 );
               break;
         }  // End of switch (BLU_req_buf[FCntl] & 0x0F)

         break;

      case IFRAME:                             // ...00 & ...10 are both I-frames
      case IFRAME + 0x02:                      // due to switch BLU_req_buf[FCntl] & 0x03!
         // *** INFORMATIONAL FRAME ***
         fprintf(S_trace, "SDLC: IFRAME - N(r)=%d, PF=%d, N(s)=%d - BLU_req_buf=",
                 (BLU_req_buf[FCntl] >> 5) & 0x7,
                 (BLU_req_buf[FCntl] >> 4) & 0x1,
                 (BLU_req_buf[FCntl] >> 1) & 0x7 );
         prt_BLU_buf(1);
         for (s = (char *) BLU_req_buf, i = 0; i < Flen; ++i, ++s)
            fprintf(S_trace, "%02X ", (int) *s & 0xFF);
         fprintf(S_trace, "\n");
         break;

   }  // End of switch (BLU_req_buf[FCntl] & 0x03)
}

//*********************************************************************
//   Thread to handle connections from the 3274 cluster emulator      *
//*********************************************************************
void *SDLC_thread(void *arg) {
   int    devnum;                  /* device nr copy for convenience    */
   int    sockopt;                 /* Used for setsocketoption          */
   int    pendingrcv;              /* pending data on the socket        */
   int    event_count;             /* # events received                 */
   int    rc, rc1;                 /* return code from various rtns     */
   struct sockaddr_in  sin, *sin2; /* bind socket address structure     */
   struct ifaddrs *nwaddr, *ifa;   /* interface address structure       */
   char   *ipaddr;
   struct epoll_event event, events[MAXSDLCLINES];

   printf("\rSDLC: Thread %ld started succesfully... \n", syscall(SYS_gettid));

   for (int j = 0; j < MAXSDLCLINES; j++) {
      sdlcline[j] = malloc(sizeof(struct SDLCLine));
      sdlcline[j]->linenum = j;
      sdlcline[j]->SDLCrlen = 0;
      sdlcline[j]->SDLCtlen = 0;
   }  // End for j = 0
   getifaddrs(&nwaddr);      /* get network address */
   for (ifa = nwaddr; ifa != NULL; ifa = ifa->ifa_next) {
      if (ifa->ifa_addr->sa_family == AF_INET && strcmp(ifa->ifa_name, "lo")) {
         sin2 = (struct sockaddr_in *) ifa->ifa_addr;
         ipaddr = inet_ntoa((struct in_addr) sin2->sin_addr);
         if (strcmp(ifa->ifa_name, "eth")) break;
      }
   }
   printf("\rSDLC: Using network Address %s on %s for PU connections\n", ipaddr, ifa->ifa_name);

   for (int j = 0; j < MAXSDLCLINES; j++) {
      if ((sdlcline[j]->line_fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1)
         printf("\nSDLC: Endpoint creation for 3274 failed with error %s ", strerror(errno));
      /* Reuse the address regardless of any */
      /* spurious connection on that port    */
      sockopt = 1;
      setsockopt(sdlcline[j]->line_fd, SOL_SOCKET, SO_REUSEADDR, (void*)&sockopt, sizeof(sockopt));
      /* Bind the socket */
      sin.sin_family=AF_INET;
      sin.sin_addr.s_addr = inet_addr(ipaddr);
      sin.sin_port = htons(37500 + LINEBASE + j);        // <=== port related to line number
      if (bind(sdlcline[j]->line_fd, (struct sockaddr *)&sin, sizeof(sin)) < 0) {
         printf("\nSDLC: Bind line-%d socket failed\n\r", j);
         free(sdlcline[j]);
         exit(EXIT_FAILURE);
      }
      /* Listen and verify */
      if ((listen(sdlcline[j]->line_fd, 10)) != 0) {
         printf("\nSDLC: Line-%d Socket listen failed %s\n\r", j, strerror(errno));
         free(sdlcline[j]);
         exit(-1);
      }
      // Add polling events for the port
      sdlcline[j]->epoll_fd = epoll_create(1);
      if (sdlcline[j]->epoll_fd == -1) {
         printf("\nSDLC: Failed to created the line-%d epoll file descriptor\n\r", j);
         free(sdlcline[j]);
         exit(-2);
      }
      event.events = EPOLLIN;
      event.data.fd = sdlcline[j]->line_fd;
      if (epoll_ctl(sdlcline[j]->epoll_fd, EPOLL_CTL_ADD, sdlcline[j]->line_fd, &event) == -1) {
         printf("\nSDLC: Add polling event failed for line-%d with error %s \n\r", j, strerror(errno));
         close(sdlcline[j]->epoll_fd);
         free(sdlcline[j]);
         exit(-3);
      }
      printf("\rSDLC: line-%d ready, waiting for connection on TCP port %d\n\r", j, 37500 + LINEBASE + j );
   }
   //
   //  Poll briefly for connect requests. If a connect request is received, proceed with connect/accept the request.
   //  Next, check all active connection for input data.
   //
   while (1) {
      for (int k = 0; k < MAXSDLCLINES; k++) {
         event_count = epoll_wait(sdlcline[k]->epoll_fd, events, 1, 50);
         for (int i = 0; i < event_count; i++) {
            sdlcline[k]->d3274_fd = accept(sdlcline[k]->line_fd, NULL, 0);
            if (sdlcline[k]->d3274_fd < 1) {
               printf("\nSDLC: accept failed for line-%d %s\n", k, strerror(errno));
             } else {
               printf("\rSDLC: PU connected to line-%d\n", k);
            }  // End if sdlcline[k]->d3274_fd
         }  // End for int i
      }  // End for int k
   }  // End while(1)

   return NULL;
}
