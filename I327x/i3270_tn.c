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

   i327x_3270.c - (C) Copyright 2021 by Henk Stegeman & Edwin Freekenhorst

   Taken from the comm3705.c module by Max H. Parke

   This module contains the functions that handle the read/write
   to/from the 3270 telnet sessions
*/

#include <inttypes.h>
#include <stdbool.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <ctype.h>
#include <pthread.h>
#include <string.h>
#include <ifaddrs.h>
#include "i327x.h"
#include "codepage.c"
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <bsd/string.h>

#if !defined(min)
#define  min(a,b)   (((a) <= (b)) ? (a) : (b))
#endif

#define BUFPD 0x1C

extern uint16_t Tdbg_flag;                 /* 1 when Ttrace.log open */
extern FILE *T_trace;                      /* Terminal trace file fd */

int write_socket( int fd, const void *_ptr, int nbytes );
int send_packet (int csock, BYTE *buf, int len, char *caption);

int double_up_iac (BYTE *buf, int len);

/*-------------------------------------------------------------------*/
/* Check if there is read activiy on the socket                      */
/* This is used by the caller to detect a connection break           */
/*-------------------------------------------------------------------*/
int  SocketReadAct(int fd) {
   int     rc;
   fd_set  fdset;
   struct  timeval timeout;
   timeout.tv_sec = 0;
   timeout.tv_usec = 0;
   FD_ZERO(&fdset);
   FD_SET(fd, &fdset);
   return select(fd + 1, &fdset, NULL, NULL,  &timeout);
}

/*-------------------------------------------------------------------*/
/* Print VTAM connected or disconnected message.                     */
/*-------------------------------------------------------------------*/
void connect_message(int sfd, int na, int flag) {
   int rc;
   struct sockaddr_in client;
   socklen_t namelen;
   char *ipaddr;
   char msgtext[256];
   if (!sfd)
      return;

   namelen = sizeof(client);
   rc = getpeername (sfd, (struct sockaddr *)&client, &namelen);
   ipaddr = inet_ntoa(client.sin_addr);
   if (flag == 0)
      sprintf(msgtext, "%s:%d VTAM CONNECTION ACCEPTED - NETWORK NODE= %4.4X",
              ipaddr, (int)ntohs(client.sin_port), na);
   else
      sprintf(msgtext, "%s:%d VTAM CONNECTION TERMINATED.",
              ipaddr, (int)ntohs(client.sin_port));
// logmsg( _("%s\n"), msgtext);

   write(sfd, msgtext, strlen(msgtext));
   write(sfd, "\r\n", 2);
}

/*-------------------------------------------------------------------*/
/* Subroutine to double up any IAC bytes in the data stream.         */
/* Returns the new length after inserting extra IAC bytes.           */
/*-------------------------------------------------------------------*/
int double_up_iac (BYTE *Dbuf, int len) {
   int m, n, x, newlen;

   /* Count the number of IAC bytes in the data */
   for (x = 0, n = 0; n < len; n++)
      if (Dbuf[n] == IAC) x++;

   /* Exit if nothing to do */
   if (x == 0) return len;

   /* Insert extra IAC bytes backwards from the end of the buffer */
   newlen = len + x;
   if (Tdbg_flag == ON) {
      fprintf(T_trace, "CC1: %d IAC bytes added, newlen = %d\n",
                      x, newlen);
   }
   for (n = newlen, m = len; n > m; ) {
      Dbuf[--n] = Dbuf[--m];
      if (Dbuf[n] == IAC) Dbuf[--n] = IAC;
   }
   return newlen;
}  /* End of function double_up_iac */

unsigned char host_to_guest (unsigned char byte)
{
    return (unsigned char)codepage_conv->h2g[(unsigned int)byte];
}

uint8_t * prt_host_to_guest( const uint8_t *psinbuf, uint8_t *psoutbuf,
                             const u_int ilength ) {
   u_int count;
   int pad = FALSE;

   for (count = 0; count < ilength; count++ ) {
      if ( !pad && psinbuf[count] == '\0' )
          pad = TRUE;
      if ( !pad ) {
          psoutbuf[count] = isprint      (psinbuf[count]) ?
                            host_to_guest(psinbuf[count]) :
                            host_to_guest('.');
      } else {
          psoutbuf[count] = host_to_guest(' ');
      }
   }
   return psoutbuf;
}

/*-------------------------------------------------------------------*/
/* Write "n" bytes to a descriptor.                                  */
/* Use in place of write() when fd is a stream socket.               */
/*-------------------------------------------------------------------*/
int write_socket( int fd, const void *_ptr, int nbytes ) {
const char *ptr;
int  nleft, nwritten;

   ptr   = _ptr;               /* point to data to be written       */
   nleft = nbytes;             /* number of bytes to be written     */

   while (nleft > 0) {         /* while bytes remain to be written  */
      nwritten = write( fd, ptr, nleft );

      if (nwritten <= 0)
         return nwritten;    /* error, return <= 0 */

      ptr   += nwritten;      /* bump to next o/p buffer location  */
      nleft -= nwritten;      /* fix remaining bytes to be written */

   } // end of do while

   return (nbytes - nleft);    /* return number of bytes written */

}  // end of write_socket

/*-------------------------------------------------------------------*/
/* SUBROUTINE TO SEND A DATA PACKET TO THE CLIENT                    */
/*-------------------------------------------------------------------*/
int send_packet(int csock, BYTE *buf, int len, char *caption) {
int  rc;                                /* Return code               */

   rc = send (csock, buf, len, 0);

   if (rc < 0) {
      printf("\nsend to client failed");
      return -1;
   } // end if(rc)

   return 0;

}  // end function send_packet */

/*-------------------------------------------------------------------*/
/* SUBROUTINE TO RECEIVE A DATA PACKET FROM THE CLIENT               */
/* This subroutine receives bytes from the client.  It stops when    */
/* the receive buffer is full, or when the last two bytes received   */
/* consist of the IAC character followed by a specified delimiter.   */
/* If zero bytes are received, this means the client has closed the  */
/* connection, and this is treated as an error.                      */
/* Input:                                                            */
/*      csock is the socket number                                   */
/*      buf points to area to receive data                           */
/*      reqlen is the number of bytes requested                      */
/*      delim is the delimiter character (0=no delimiter)            */
/* Output:                                                           */
/*      buf is updated with data received                            */
/*      The return value is the number of bytes received, or         */
/*      -1 if an error occurred.                                     */
/*-------------------------------------------------------------------*/
static int
recv_packet (int csock, BYTE *buf, int reqlen, BYTE delim) {
int   rc = 0;                           /* Return code               */
int   rcvlen = 0;                       /* Length of data received   */

   while (rcvlen < reqlen) {

      rc = recv (csock, buf + rcvlen, reqlen - rcvlen, 0);

      if (rc < 0) {
         // WRMSG(HHC01034, "E", "recv()", strerror(HSO_errno));
         return -1;
      }

      if (rc == 0) {
         // TNSDEBUG1("console: DBG004: Connection closed by client\n");
         return -1;
      }

      rcvlen += rc;

      if (delim != '\0' && rcvlen >= 2
         && buf[rcvlen-2] == IAC && buf[rcvlen-1] == delim)
      break;
   }

   // TNSDEBUG2("console: DBG005: Packet received length=%d\n", rcvlen);
   // packet_trace (buf, rcvlen);

   return rcvlen;

}  // end function recv_packet


/*-------------------------------------------------------------------*/
/* SUBROUTINE TO RECEIVE A PACKET AND COMPARE WITH EXPECTED VALUE    */
/*-------------------------------------------------------------------*/
static int
expect (int csock, BYTE *expected, int len, char *caption) {
int     rc;                             /* Return code               */
BYTE    buf[512];                       /* Receive buffer            */

#if defined( OPTION_MVS_TELNET_WORKAROUND )

   /* TCP/IP for MVS returns the server sequence rather then the
      client sequence during bin negotiation.   Jan Jaeger, 19/06/00  */

   static BYTE do_bin[] = { IAC, DO, BINARY, IAC, WILL, BINARY };
   static BYTE will_bin[] = { IAC, WILL, BINARY, IAC, DO, BINARY };

#endif // defined( OPTION_MVS_TELNET_WORKAROUND )

   // UNREFERENCED(caption);

   rc = recv_packet (csock, buf, len, 0);
   if (rc < 0)
      return -1;

#if defined( OPTION_MVS_TELNET_WORKAROUND )
   /* BYPASS TCP/IP FOR MVS WHICH DOES NOT COMPLY TO RFC1576 */
   if (1
      && memcmp(buf, expected, len) != 0
      && !(len == sizeof(will_bin)
      && memcmp(expected, will_bin, len) == 0
      && memcmp(buf, do_bin, len) == 0)
   )
#else
   if (memcmp(buf, expected, len) != 0)
#endif // defined( OPTION_MVS_TELNET_WORKAROUND )
   {
      // TNSDEBUG2("console: DBG006: Expected %s\n", caption);
      return -1;
   }
   // TNSDEBUG2("console: DBG007: Received %s\n", caption);

   return 0;

}  // end function expect

/*-------------------------------------------------------------------*/
/* SUBROUTINE TO NEGOTIATE TELNET PARAMETERS                         */
/* This subroutine negotiates the terminal type with the client      */
/* and uses the terminal type to determine whether the client        */
/* is to be supported as a 3270 display console or as a 1052/3215    */
/* printer-keyboard console.                                         */
/*                                                                   */
/* Valid display terminal types are "IBM-NNNN", "IBM-NNNN-M", and    */
/* "IBM-NNNN-M-E", where NNNN is 3270, 3277, 3278, 3279, 3178, 3179, */
/* or 3180, M indicates the screen size (2=25x80, 3=32x80, 4=43x80,  */
/* 5=27x132, X=determined by Read Partition Query command), and      */
/* -E is an optional suffix indicating that the terminal supports    */
/* extended attributes. Displays are negotiated into tn3270 mode.    */
/* An optional device number suffix (example: IBM-3270@01F) may      */
/* be specified to request allocation to a specific device number.   */
/* Valid 3270 printer type is "IBM-3287-1"                           */
/*                                                                   */
/* Terminal types whose first four characters are not "IBM-" are     */
/* handled as printer-keyboard consoles using telnet line mode.      */
/*                                                                   */
/* Input:                                                            */
/*      csock   Socket number for client connection                  */
/* Output:                                                           */
/*      class   D=3270 display console, K=printer-keyboard console   */
/*              P=3270 printer                                       */
/*      model   3270 model indicator (2,3,4,5,X)                     */
/*      extatr  3270 extended attributes (Y,N)                       */
/*      devn    Requested device number, or FF=any device number     */
/* Return value:                                                     */
/*      0=negotiation successful, -1=negotiation error               */
/*-------------------------------------------------------------------*/
int negotiate(int csock, BYTE *class, BYTE *model, BYTE *extatr, BYTE *devn)
{
int    rc;                              /* Return code               */
char  *termtype;                        /* Pointer to terminal type  */
char  *s;                               /* String pointer            */
BYTE   c;                               /* Trailing character        */
BYTE   devnum;                          /* Requested device number   */
BYTE   buf[512];                        /* Telnet negotiation buffer */
static BYTE do_term[] = { IAC, DO, TERMINAL_TYPE };
static BYTE will_term[] = { IAC, WILL, TERMINAL_TYPE };
static BYTE req_type[] = { IAC, SB, TERMINAL_TYPE, SEND, IAC, SE };
static BYTE type_is[] = { IAC, SB, TERMINAL_TYPE, IS };
static BYTE do_eor[] = { IAC, DO, EOR, IAC, WILL, EOR };
static BYTE will_eor[] = { IAC, WILL, EOR, IAC, DO, EOR };
static BYTE do_bin[] = { IAC, DO, BINARY, IAC, WILL, BINARY };
static BYTE will_bin[] = { IAC, WILL, BINARY, IAC, DO, BINARY };
#if 0
static BYTE do_tmark[] = { IAC, DO, TIMING_MARK };
static BYTE will_tmark[] = { IAC, WILL, TIMING_MARK };
static BYTE wont_sga[] = { IAC, WONT, SUPPRESS_GA };
static BYTE dont_sga[] = { IAC, DONT, SUPPRESS_GA };
#endif
static BYTE wont_echo[] = { IAC, WONT, ECHO_OPTION };
static BYTE dont_echo[] = { IAC, DONT, ECHO_OPTION };
static BYTE will_naws[] = { IAC, WILL, NAWS };

   /* Perform terminal-type negotiation */
   rc = send_packet (csock, do_term, sizeof(do_term),
                       "IAC DO TERMINAL_TYPE");
   if (rc < 0) return -1;

   rc = expect (csock, will_term, sizeof(will_term),
                       "IAC WILL TERMINAL_TYPE");
   if (rc < 0) return -1;

   /* Request terminal type */
   rc = send_packet (csock, req_type, sizeof(req_type),
                       "IAC SB TERMINAL_TYPE SEND IAC SE");
   if (rc < 0) return -1;

   rc = recv_packet (csock, buf, sizeof(buf)-2, SE);
   if (rc < 0) return -1;

   /* Ignore Negotiate About Window Size */
   if (rc >= (int)sizeof(will_naws) &&
      memcmp (buf, will_naws, sizeof(will_naws)) == 0)
   {
      memmove(buf, &buf[sizeof(will_naws)], (rc - sizeof(will_naws)));
      rc -= sizeof(will_naws);
   }

   if (rc < (int)(sizeof(type_is) + 2)
         || memcmp(buf, type_is, sizeof(type_is)) != 0
         || buf[rc-2] != IAC || buf[rc-1] != SE) {
//    TNSDEBUG2("console: DBG008: Expected IAC SB TERMINAL_TYPE IS\n");
      return -1;
   }
   buf[rc-2] = '\0';
   termtype = (char *)(buf + sizeof(type_is));
// TNSDEBUG2("console: DBG009: Received IAC SB TERMINAL_TYPE IS %s IAC SE\n",
// termtype);

   /* Check terminal type string for device name suffix */
   s = strchr (termtype, '@');

   if (s != NULL && sscanf (s, "@%02x", &devnum) == 1) {
      *devn = devnum;
   }
   else {
      *devn = 0xFF;
   }

   // Test for non-display terminal type
   if (memcmp(termtype, "IBM-", 4) != 0) {
#if 0
      /* Perform line mode negotiation */
      rc = send_packet (csock, do_tmark, sizeof(do_tmark),
                          "IAC DO TIMING_MARK");
      if (rc < 0) return -1;

      rc = expect (csock, will_tmark, sizeof(will_tmark),
                          "IAC WILL TIMING_MARK");
      if (rc < 0) return 0;

      rc = send_packet (csock, wont_sga, sizeof(wont_sga),
                          "IAC WONT SUPPRESS_GA");
      if (rc < 0) return -1;

      rc = expect (csock, dont_sga, sizeof(dont_sga),
                          "IAC DONT SUPPRESS_GA");
      if (rc < 0) return -1;
#endif

      if (memcmp(termtype, "ANSI", 4) == 0) {
         rc = send_packet (csock, wont_echo, sizeof(wont_echo),
                           "IAC WONT ECHO");
         if (rc < 0) return -1;

         rc = expect (csock, dont_echo, sizeof(dont_echo),
                           "IAC DONT ECHO");
         if (rc < 0) return -1;
      }

      /* Return printer-keyboard terminal class */
      *class = 'K';
      *model = '-';
      *extatr = '-';
      return 0;
   }

   /* Determine display terminal model */
   if (memcmp(termtype+4,"DYNAMIC",7) == 0) {
       *model = 'X';
       *extatr = 'Y';
   } else {
      if (!(memcmp(termtype+4, "3277", 4) == 0
         || memcmp(termtype+4, "3270", 4) == 0
         || memcmp(termtype+4, "3178", 4) == 0
         || memcmp(termtype+4, "3278", 4) == 0
         || memcmp(termtype+4, "3179", 4) == 0
         || memcmp(termtype+4, "3180", 4) == 0
         || memcmp(termtype+4, "3287", 4) == 0
         || memcmp(termtype+4, "3279", 4) == 0))
         return -1;

      *model = '2';
      *extatr = 'N';

      if (termtype[8]=='-') {
         if (termtype[9] < '1' || termtype[9] > '5')
             return -1;
         *model = termtype[9];
         if (memcmp(termtype+4, "328",3) == 0)
            *model = '2';
         if (memcmp(termtype+10, "-E", 2) == 0)
            *extatr = 'Y';
      }
   }

   /* Perform end-of-record negotiation */
   rc = send_packet (csock, do_eor, sizeof(do_eor),
                       "IAC DO EOR IAC WILL EOR");
   if (rc < 0) return -1;

   rc = expect (csock, will_eor, sizeof(will_eor),
                       "IAC WILL EOR IAC DO EOR");
   if (rc < 0) return -1;

   /* Perform binary negotiation */
   rc = send_packet (csock, do_bin, sizeof(do_bin),
                       "IAC DO BINARY IAC WILL BINARY");
   if (rc < 0) return -1;

   rc = expect (csock, will_bin, sizeof(will_bin),
                       "IAC WILL BINARY IAC DO BINARY");
   if (rc < 0) return -1;

   /* Return display terminal class */
   if (memcmp(termtype+4,"3287",4)==0) *class='P';
   else *class = 'D';
   return 0;

}  // End function negotiate


/*-------------------------------------------------------------------*/
/* NEW CLIENT CONNECTION                                             */
/*-------------------------------------------------------------------*/
int connect_client (int *csockp, BYTE i327xnump, BYTE *portnump, BYTE *portnumr)
/* returns 1 if 3270, else 0 */
{
int                     rc;             /* Return code               */
size_t                  len;            /* Data length               */
int                     csock;          /* Socket for conversation   */
BYTE                    portnum;        /* Proposed port number      */
struct sockaddr_in      client;         /* Client address structure  */
socklen_t               namelen;        /* Length of client structure*/
char                   *clientip;       /* Addr of client ip address */
BYTE                    devnum;         /* Requested device number   */
BYTE                    class;          /* D=3270, P=3287, K=3215/1052 */
BYTE                    model;          /* 3270 model (2,3,4,5,X)    */
BYTE                    extended;       /* Extended attributes (Y,N) */
char                    buf[256];       /* Message buffer            */
char                    conmsg[256];    /* Connection message        */
char                    devmsg[40];     /* Device message            */
char                    hostmsg[256];   /* Host ID message           */
char                    num_procs[16];  /* #of processors string     */

   /* Load the socket address from the thread parameter */
   csock = *csockp;

   /* Obtain the client's IP address */
   namelen = sizeof(client);
   rc = getpeername (csock, (struct sockaddr *)&client, &namelen);

   /* Log the client's IP address and hostname */
   clientip = strdup(inet_ntoa(client.sin_addr));


   /* Negotiate telnet parameters */
   rc = negotiate (csock, &class, &model, &extended, portnumr);
   if (*portnumr == 0xFF) *portnumr = *portnump;

   if (rc != 0) {
      close (csock);
      if (clientip) free(clientip);
      return 0;
   }

   /* Build connection message for client */
       snprintf (devmsg, sizeof(devmsg)-1, "Connecting to 327x-%01X port %02X  ", i327xnump, *portnumr);

   /* Send connection message to client */
   if (class != 'K') {
      len = snprintf (buf, sizeof(buf)-1,
               "\xF5\x40\x11\x40\x40\x1D\x60%s",
               prt_host_to_guest( (BYTE*) devmsg,  (BYTE*) devmsg,  strlen( devmsg  )));

      if (len < sizeof(buf)) {
         buf[len++] = IAC;
      }
      else {
      // ASSERT(FALSE);
      }

      if (len < sizeof(buf)) {
         buf[len++] = EOR_MARK;
      }
      else {
      // ASSERT(FALSE);
      }
   }
   else {
      len = snprintf (buf, sizeof(buf)-1, "%s\r\n%s\r\n%s\r\n",
                      conmsg, hostmsg, devmsg);
   }

   if (class != 'P') {  /* do not write connection resp on 3287 */
      rc = send_packet (csock, (BYTE *)buf, (int)len, "CONNECTION RESPONSE");
   }
   return (class == 'D') ? 1 : 0;   /* return 1 if 3270 */
}  // End function connect_client */

/********************************************************************/
/* function to read the data from the 3270 terminal                 */
/********************************************************************/

void commadpt_read_tty(struct CB327x *i327x, struct IO3270 *ioblk, BYTE * bfr, BYTE lunum, int len)
// everything has been tortured to now do 3270 also
{
   BYTE        bfr3[3];
   BYTE        c;
   int i1;
   int eor=0;
// logdump("RECV",i327x->dev, bfr,len);
   /* If there is a complete data record already in the buffer
      then discard it before reading more data
      For TTY, allow data to accumulate until CR is received */

   if (i327x->is_3270[lunum]) {
      if (ioblk->inpbufl) {
         i327x->rlen3270[lunum] = 0;
         ioblk->inpbufl = 0;
      }
   }


   for (i1 = 0; i1 < len; i1++) {
      c = (unsigned char) bfr[i1];

      if (i327x->telnet_opt[lunum]) {
         i327x->telnet_opt[lunum] = 0;
         bfr3[0] = 0xff;  /* IAC */
         /* set won't/don't for all received commands */
         bfr3[1] = (i327x->telnet_cmd[lunum] == 0xfd) ? 0xfc : 0xfe;
         bfr3[2] = c;
         if (i327x->lu_fd[lunum] > 0) {
            write_socket(i327x->lu_fd[lunum],bfr3,3);
         }

         continue;
      }
      if (i327x->telnet_iac[lunum]) {
         i327x->telnet_iac[lunum] = 0;

         switch (c) {
            case 0xFB:  /* TELNET WILL option cmd */
            case 0xFD:  /* TELNET DO option cmd */
               i327x->telnet_opt[lunum] = 1;
               i327x->telnet_cmd[lunum] = c;
               break;
            case 0xF4:  /* TELNET interrupt */
               if (!i327x->telnet_int[lunum]) {
                   i327x->telnet_int[lunum] = 1;
               }
               break;
            case EOR_MARK:
                                eor = 1;
               break;
            case 0xFF:  /* IAC IAC */
                        ioblk->inpbuf[i327x->rlen3270[lunum]++] = 0xFF;
               break;
            }
            continue;
         }
         if (c == 0xFF) {  /* TELNET IAC */
            i327x->telnet_iac[lunum] = 1;
            continue;
         } else {
            i327x->telnet_iac[lunum] = 0;
         }
         if (!i327x->is_3270[lunum]) {
            if (c == 0x0D) // CR in TTY mode ?
                i327x->eol_flag[lunum] = 1;
            c = host_to_guest(c);   // translate ASCII to EBCDIC for tty
         }
         ioblk->inpbuf[i327x->rlen3270[lunum]++] = c;

   }
   /* received data (rlen3270 > 0) is sufficient for 3270,
      but for TTY, eol_flag must also be set */
// printf("\n");

   if ((i327x->eol_flag[lunum] || i327x->is_3270[lunum]) && i327x->rlen3270[lunum]) {
      i327x->eol_flag[lunum] = 0;
      if (i327x->is_3270[lunum]) {
         if (eor) {
            ioblk->inpbufl = i327x->rlen3270[lunum];
            i327x->rlen3270[lunum] = 0; /* for next msg */
         } // End if eor
      } else {
         ioblk->inpbufl = i327x->rlen3270[lunum];
         i327x->rlen3270[lunum] = 0; /* for next msg */
      }  // End if (i327x->is_3270[lunum])
   }  // End i327x->eol_flag[lunum]
}

