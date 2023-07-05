/* Copyright (c) 2022, Henk Stegeman and Edwin Freekenhorst

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

   3705_chan_T2.c IBM 3705 Channel Adaptor Type 2 simulator
*/

#include "sim_defs.h"
#include "i3705_defs.h"
#include "i3705_Eregs.h"     // External regs defs
#include <signal.h>
#include <ctype.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>

#define IMAX 4096            // Input buffer size Random. Need to define a more rational value
#define RMAX 64              // Response buffer size Random. Need to define a more rational value

#define CAA 0                // Channel Adapter channel connection A
#define CAB 1                // Channel Adapter channel connection B
#define PORTCA1A 37051       // TCP/IP port number CA1 A
#define PORTCA1B 37052       // TCP/IP port number CA1 B
#define PORTCA2A 37053       // TCP/IP port number CA2 A
#define PORTCA2B 37054       // TCP/IP port number CA2 B
#define SA struct sockaddr_in
#define TRUE  1
#define FALSE 0

// CSW Unit Status conditions.  Channel status conditions not defined (yet).
#define CSW_ATTN 0x80        // Attention
#define CSW_SMOD 0x40        // Status Modifier
#define CSW_UEND 0x20        // Control Unit End
#define CSW_BUSY 0x10        // Busy
#define CSW_CEND 0x08        // Channel End (CE)
#define CSW_DEND 0x04        // Device End (DE)
#define CSW_UCHK 0x02        // Unit check
#define CSW_UEXC 0x01        // Unit Exception

// Sense return codes
#define SENSE_CR 0x80        // Command Reject

#define checkrc(expr) if(!(expr)) { perror(#expr); return -1; }

typedef enum { false, true } bool;

extern int32 debug_reg;
extern int32 Eregs_Inp[];
extern int32 Eregs_Out[];
extern uint8 M[];
extern int8  CA1_DS_req_L3;  // Chan Adap Data/Status request flag
extern int8  CA1_IS_req_L3;  // Chan Adap Initial/Sel request flag

// Trace variables
uint16_t Adbg_reg = 0x00;    // Bit flags for debug/trace
uint16_t Adbg_flag = OFF;    // 1 when Atrace.log open
FILE  *A_trace;

void *CAx_thread(void *arg);

uint16_t CAPORTS[MAXCHAN][2] = {{37051, 37052}, {37053, 37054}};  // 3705 supports 2 channels
uint16_t CAMASKS[MAXCHAN] = {0x0008, 0x0020};                     // 3705 supports 2 channels
int i;
uint8_t nobytes, tcount;

// SenseID
uint8_t sense_id[4] = {0xFF, 0x37, 0x05, 0x02};

// Declaration of thread condition variable
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

// Declaring mutex
extern pthread_mutex_t r77_lock;


void exec_attn();
void exec_pci();
void exec_ccw(struct IO3705 *iob);
int reg_bit(int reg, int bit_mask);
int Ireg_bit(int reg, int bit_mask);
void wait();

struct CCW {    /* Channel Command Word */
   uint8_t  code;
   uint8_t  dataddress[3];
   uint8_t  flags;
   uint8_t  chain;           // Format 0. Chain is not correct (yet), but aligned to Hercules
   uint16_t count;
} ccw;

struct CSW {    /* Channel Status Word */
   uint8_t  key;
   uint8_t  dataddress[3];
   uint8_t  unit_conditions;
   uint8_t  channel_conditions;
   uint16_t count;
} csw;

struct IO3705*  iobs[MAXCHAN];  /* IBM 3705 I/O Block pointer array */

struct pth_args {
   void* arg1;
   void* arg2;
} *args;

int wrkid, offset;
int CAready;

char abswid[2] = {"AB"};
int epoll_fd;
struct epoll_event event, events[MAXCHAN*2];

// ************************************************************
// Function to format and display incomming data from host
// ************************************************************
void print_hex(char *buffptr, int buf_len) {
   if ((Adbg_flag == ON) && (Adbg_reg & 0x01)) {   // Trace channel adapter activities ?
      fprintf(A_trace, "\nRecord length: %X  (hex)\n\r", buf_len);
         for (int i = 0; i < buf_len; i++) {
            fprintf(A_trace, "%02X ", (unsigned char)buffptr[i]);
            if ((i + 1) % 16 == 0)
               fprintf(A_trace, "\n\r");
         }  // End for  int i
      fprintf(A_trace, "\n\r");
      }  // End if Adbg
   return;
}

// ************************************************************
// Function to display the CA registers
// ************************************************************
void print_regs(struct IO3705 *iob, char *text) {
   if ((Adbg_flag == ON) && (Adbg_reg & 0x01)) {   // Trace channel adapter activities ?
      fprintf(A_trace, "\n************************** Channel Adapter %c Register display **********************\n\r", iob->CA_id);
      fprintf(A_trace, "CA code location: %s\n\r", text);
      fprintf(A_trace, "     x'50' x'51' x'52' x'53' x'54' x'55' x'56' x'57' x'58' x'59' x'5A' x'5B' x'5C' \n\r");
      fprintf(A_trace, "In : ");
      for (uint8_t h = 0x50; h <= 0x5C; h++) {
         fprintf(A_trace, "%04X  ", Eregs_Inp[h] );
      }
      fprintf(A_trace, "\n\r");
      fprintf(A_trace, "Out: ");
      for (uint8_t h = 0x50; h <= 0x5B; h++) {
         fprintf(A_trace, "%04X  ", Eregs_Out[h] );
      }
      fprintf(A_trace, "\n\r");
      fprintf(A_trace, "************************************************************************************\n\r");
      fflush(A_trace);
   }
}

// ************************************************************
// Function to check if socket is (still) connected
// ************************************************************
static bool IsSocketConnected(int sockfd) {
   int rc;
   struct sockaddr_in *ccuptr;
   socklen_t *addrlen;
   ccuptr = (struct sockaddr_in*)malloc(sizeof(struct sockaddr_in));
   addrlen = (socklen_t*)malloc(sizeof(socklen_t));

   rc = getpeername(sockfd, ccuptr, addrlen);

   free(ccuptr);
   free(addrlen);

   if (rc == 0)
      return true;
   else
      return false;
}

// ************************************************************
// Function to accept TCP connection from host
// ************************************************************
int host_connect(struct IO3705 *iob, int abport) {
   int alive = 1;     // Enable KEEP_ALIVE
   int idle = 5;      // First  probe after 5 seconds
   int intvl = 3;     // Subsequent probes after 3 seconds
   int cntpkt = 3;    // Timeout after 3 failed probes
   int timeout = 1000;
   int rc;

   // Accept the incoming connection
   iob->addrlen[abport] = sizeof(iob->address[abport]);

   iob->bus_socket[abport] = accept(iob->CA_socket[abport], (struct sockaddr *)&iob->address[abport], (socklen_t*)&iob->addrlen[abport]);
   if (iob->bus_socket[abport] < 0) {
      printf("\nCA%c: Host accept failed for bus connection...\r" ,iob->CA_id);
      return -1;
   } else {
      if (setsockopt(iob->bus_socket[abport], SOL_SOCKET, SO_KEEPALIVE, (void *)&alive, sizeof(alive))) {
         perror("ERROR: setsockopt(), SO_KEEPALIVE");
         return -1;
      }

      if (setsockopt(iob->bus_socket[abport], IPPROTO_TCP, TCP_KEEPIDLE, (void *)&idle, sizeof(idle))) {
         perror("ERROR: setsockopt(), SO_KEEPIDLE");
         return -1;
      }

      if (setsockopt(iob->bus_socket[abport], IPPROTO_TCP, TCP_KEEPINTVL, (void *)&intvl, sizeof(intvl))) {
         perror("ERROR: setsockopt(), SO_KEEPINTVL");
         return -1;
      }

      if (setsockopt(iob->bus_socket[abport], IPPROTO_TCP, TCP_KEEPCNT, (void *)&cntpkt, sizeof(cntpkt))) {
         perror("ERROR: setsockopt(), SO_KEEPCNT");
         return -1;
      }

      printf("\nCA%c: New bus connection on 3705 port %d, socket fd is %d, ip is : %s, port : %d \n\r",
            iob->CA_id, CAPORTS[(iob->CA_id - '0')-1][abport], iob->bus_socket[abport], inet_ntoa(iob->address[abport].sin_addr),
            (ntohs(iob->address[abport].sin_port)));
   }

   // Get the tag connection
   while (1) {
      iob->tag_socket[abport] = accept(iob->CA_socket[abport], (struct sockaddr *)&iob->address[abport], (socklen_t*)&iob->addrlen[abport]);

      if (iob->tag_socket[abport] > 0) {
         printf("\nCA%c: New tag connection on 3705 port %d, socket fd is %d, ip is : %s, port : %d \n\r",
                  iob->CA_id, CAPORTS[(iob->CA_id - '0')-1][abport], iob->tag_socket[abport], inet_ntoa(iob->address[abport].sin_addr),
                  (ntohs(iob->address[abport].sin_port)));
         break;
      } else {
         if (errno != EAGAIN)  {
            printf("\nCA%c: Host accept failed with errno %d for tag connection...\n\r", iob->CA_id, errno);
            return -1;
         }
      }
   }
   return 0;
}


// ************************************************************
// Function to send CA return status to the host
// ************************************************************
void send_carnstat(int sockptr, char *carnstat, uint8_t *ackbuf, char CA_id) {
   int rc, retry;                  // Return code

   while (Ireg_bit(0x77, 0x0028) == ON)
      wait();                      // Wait for CA 1 L3 interrupt reset

   // If DE and CE and reset Write Break Remember and Channel Active
   if (*carnstat & CSW_DEND) {
      Eregs_Inp[0x55] &= ~0x0040;  // Reset Write Break Remember
      Eregs_Inp[0x55] &= ~0x0100;  // Reset Channel Active
      *carnstat |= CSW_CEND;       // CA sets channel end
   }
   // If CE...
   if (*carnstat & CSW_CEND)
      Eregs_Inp[0x55] &= ~0x4000;  // Reset zero override flag

   if ((Adbg_flag == ON) && (Adbg_reg & 0x01))    // Trace channel adapter activities ?
      fprintf(A_trace, "CA%c: CARNSTAT %02X via socket %d\n\r", CA_id, *carnstat, sockptr);
   if (sockptr != -1) {
      rc = send(sockptr, carnstat, 1, 0);
   if ((Adbg_flag == ON) && (Adbg_reg & 0x01))    // Trace channel adapter activities ?
         fprintf(A_trace, "CA%c: Send %d bytes on socket %d\n\r", CA_id, rc, sockptr);
   } else
      rc = -1;

   if (rc < 0) {
      printf("\nCA%c: CA status send to host failed...\n\r", CA_id);
      return;
   }
   Eregs_Out[0x54] &= ~0xFFFF;                      // Reset CA status bytes
   if (CA_id == '1')
      Eregs_Inp[0x55] &= ~0x0101;                   // Reset CA Active and CA 1 selected
   else
      Eregs_Inp[0x55] &= ~0x0102;                   // Reset CA Active  and CA 2 selected
   return;
}  // end function send_carnstat

// ************************************************************
// Function to wait for an ACK from the host
// ************************************************************
void recv_ack(int sockptr) {
   uint8_t ackbuf;
   int rc;
   rc = read(sockptr, &ackbuf, 1);
   return;
}

// ************************************************************
// Function to send an ACK to the host
// ************************************************************
void send_ack(int sockptr) {
   uint8_t ackbuf;
   int rc;
   rc = send(sockptr, &ackbuf, 1, 0);
   return;
}

// ************************************************************
// Function to read data from TCP socket
// ************************************************************
int read_socket(int sockptr, char *buffptr, int buffsize) {
   int reclen;
   bzero(buffptr, buffsize);
   reclen = read(sockptr, buffptr, buffsize);
   if (reclen < 1)
      printf("\nCA: Read failed with error %s \n\r", strerror(errno));
   return reclen;
}

// ************************************************************
// Function to close TCP socket
// ************************************************************
int close_socket(struct IO3705 *iob, int abchannel) {

   /* First shutdown the sockets to terminate active blocked reads */
   /* The BUS and TAG sockets are closed by the active CA1/2 thread */

   if (iob->bus_socket[abchannel] > 0)
   shutdown(iob->bus_socket[abchannel], SHUT_RDWR);

   if (iob->tag_socket[abchannel] > 0)
      shutdown(iob->tag_socket[abchannel], SHUT_RDWR);

   if (iob->CA_socket[abchannel] > 0) {
      close(iob->CA_socket[abchannel]);
      iob->CA_socket[abchannel] = -1;

      printf("\nCA%c: Channel connection %c closed\n\r", iob->CA_id, abswid[abchannel]);
   }
   return 0;
}

// ************************************************************
// Function to send response data to the host
// ************************************************************
int send_socket(int sockptr, char *respp, int respsize) {
   int rc;                      /* Return code */
   rc = send (sockptr, respp, respsize, 0);

   if (rc < 0) {
      printf("\nCA: Send to host failed...\n\r");
      return -1;
   }
   return 0;
}  // end function send_socket

// ***********************************************************
// Function to enable the A or B port of a CA.
// ***********************************************************
void start_listen(struct IO3705 *iob, int abport) {

   int flag = 1;

   if ((iob->CA_socket[abport] = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0)) == -1) {
      printf("\nCA%c: Endpoint creation for channel %c failed with error %s ", iob->CA_id, abswid[abport], strerror(errno));
      exit(EXIT_FAILURE);
   }

   iob->address[abport].sin_family = AF_INET;
   iob->address[abport].sin_addr.s_addr = INADDR_ANY;
   iob->address[abport].sin_port = htons( CAPORTS[(iob->CA_id - '0')-1][abport] );

   if (-1 == setsockopt(iob->CA_socket[abport], SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
      printf("\nCA%c: Setsockopt failed for Channel %c with error %s\n\r", iob->CA_id, abswid[abport], strerror(errno));
   }
   // Bind the socket to localhost port PORT
   if (bind(iob->CA_socket[abport], (struct sockaddr *)&iob->address[abport], sizeof(iob->address[abport])) < 0) {
      printf("\nCA%c: bind failed for port %d\n\r", iob->CA_id, CAPORTS[(iob->CA_id - '0')-1][abport] );
      exit(EXIT_FAILURE);
   }
   // Listen and verify
   if ((listen(iob->CA_socket[abport], 5)) != 0) {
      printf("\nCA%c: Listen failed for port %c\n\r", iob->CA_id, abswid[abport]);
      exit(-1);
   }
   // Add polling events for the port
   event.events = EPOLLIN | EPOLLONESHOT;
   event.data.fd = iob->CA_socket[abport];
   if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, iob->CA_socket[abport], &event)) {
      printf("\nCA%c: Add polling event failed for port %c with error %s \n\r", iob->CA_id, abswid[abport], strerror(errno));
      close(epoll_fd);
      exit(-2);
   }
   // Now server is ready to listen
   printf("CA%c: Waiting for channel connection on TCP port %d \n\r", iob->CA_id, CAPORTS[(iob->CA_id - '0')-1][abport] );
}

// ************************************************************
// Function for sending Attention interrupts to the host
// ************************************************************
void exec_attn() {
   int rc, j;
   uint8_t carnstat;
   uint8_t ackbuf;
   ackbuf = 0x00;

   // Determine which CA needs to react
   if (Eregs_Out[0x57] & 0x0008)
      j = 0;
   else
      j = 1;
   if ((Adbg_flag == ON) && (Adbg_reg & 0x01))       // Trace channel adapter activities ?
      fprintf(A_trace, "CA%c: L3 register 55 %04X \n\r", iobs[j]->CA_id, Eregs_Out[0x55]);
   Eregs_Inp[0x55] |= 0x0200;                        // Set Attention Request
   pthread_mutex_lock(&r77_lock);
   Eregs_Inp[0x77] |= iobs[j]->CA_mask;              // Set CA1 L3 Interrupt Request
   pthread_mutex_unlock(&r77_lock);
   CA1_IS_req_L3 = ON;
   while (Ireg_bit(0x77, iobs[j]->CA_mask) == ON)
      wait();
   Eregs_Out[0x55] &= ~0x0200;                       // Reset attention request
   print_regs(iobs[j], "ATTN");
   carnstat = (carnstat &0x00) | CSW_ATTN;           // Set ATTN CA return status
   //carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF);  // Get CA return status
   if (iobs[j]->CA_active == TRUE) {
      if ((Adbg_flag == ON) && (Adbg_reg & 0x01))    // Trace channel adapter activities ?
          fprintf(A_trace, "CA%c: Sending ATTN\n\r", iobs[j]->CA_id);
      // Send CA retun status to host
      send_carnstat(iobs[j]->tag_socket[iobs[j]->abswitch], &carnstat, &ackbuf, iobs[j]->CA_id);
   } else {
      printf("CA%c: Channel not active, ATTN not send \n\r");
   }
   return;
}

// ************************************************************
// Function for requisting a L3 int from the control program
// ************************************************************
void exec_pci() {
   int j;
      // Determine which CA needs to react
      if (Eregs_Out[0x57] & 0x0008)
         j = 0;
      else
         j = 1;
      if ((Adbg_flag == ON) && (Adbg_reg & 0x01))    // Trace channel adapter activities ?
         fprintf(A_trace, "CA%c: L3 register 57 %04X \n\r", iobs[j]->CA_id, Eregs_Out[0x57]);
      while (Ireg_bit(0x77, iobs[j]->CA_mask) == ON)
         wait();
      Eregs_Inp[0x55] |= 0x0800;                     // Set Program Requested L3 interrupt
      Eregs_Out[0x55] |= 0x3000;                     // Set INCWAR and OUTCWAR valid for IPL
      pthread_mutex_lock(&r77_lock);
      Eregs_Inp[0x77] |= iobs[j]->CA_mask;           // Set CA L3 interrupt request
      pthread_mutex_unlock(&r77_lock);
      CA1_IS_req_L3 = ON;
      if ((Adbg_flag == ON) && (Adbg_reg & 0x01))    // Trace channel adapter activities ?
         fprintf(A_trace, "CA%c: Requested L3 interrupt\n\r", iobs[j]->CA_id);
      while (Ireg_bit(0x77, iobs[j]->CA_mask) == ON) wait();
         wait();
      Eregs_Out[0x57] &= ~0x0080;                    // Reset L3 request
   return;
}

// ************************************************************
// Function for handling diagnostic wrap mode
// ************************************************************
void exec_diag() {
         printf("CA: 3705 Diagnostic request \n\r");
      // Determine if diagnstic mode needs to be set
      if ((Eregs_Out[0x57] & 0x0001) && !(Eregs_Inp[0x55] & 0x8000))  {
         Eregs_Inp[0x55] |= 0x8000;           // Diagnostic wrap mode on
         //iob->CA_active = FALSE;              // set CA to inactive
         printf("CA: 3705 Diagnostic mode turned on\n\r");
      }
      // Determine if diagnstic mode needs to be turned off
      if ((Eregs_Out[0x57] & 0x0000) && (Eregs_Inp[0x55] & 0x8000))  {
         Eregs_Inp[0x55] &= ~0x8000;           // Diagnostic wrap mode off
         //iob->CA_active = TRUE  ;              // set CA to active
         printf("CA: 3705 Diagnostic mode turned off\n\r");
      }
   return;
}

// ********************************************************************
// Channel adaptor type 2 thread
// ********************************************************************
void *CA_T2_thread(void *arg) {
   int rc, sig, event_count;
   uint32_t CAx_tid[2];
   struct sockaddr_in address;
   typedef union epoll_data {
      void    *ptr;
      int      fd;
      uint32_t u32;
      uint64_t u64;
   } epoll_Data_t;

   printf("\nCA-T2: Main thread %ld started succesfully...\n", syscall(SYS_gettid));

   // ********************************************************************
   //  Channel Adapter debug trace facility
   // ********************************************************************
   if (Adbg_flag == OFF) {
      A_trace = fopen("trace_A.log", "w");
      fprintf(A_trace, "     ****** 3705 CHANNEL ADAPTER log file ****** \n\n"
                       "                   01 - trace CCW activity \n"
                       );
      Adbg_flag = ON;
   }
   Adbg_reg = 0x00;
   pthread_t id1, id2, id3;
   args = malloc(sizeof(struct pth_args) * 1);
   /***************************************/
   /* Allocate the CA IO Blocks           */
   /***************************************/
   for (int i = 0; i < MAXCHAN; i++)
      iobs[i] = malloc(sizeof(struct IO3705) * 1);
   /***************************************/
   /* Initialize the CA IO Blocks         */
   /***************************************/
   for (int i = 0; i < MAXCHAN; i++) {
      iobs[i]->CA_id = 0x31 + i;           // First channel id starts with 1
      iobs[i]->abswitch = 0;               // A/B switch is default set to A
      iobs[i]->CA_active = FALSE;          // Initial state is not active (No TCP connection yet)
      iobs[i]->abswhist = CAB;             // This forces a listen on CAx port A
      iobs[i]->CA_mask = CAMASKS[i];       // Mask to enable port A
      iobs[i]->chainbl = 0;                // Initial chain data buffer length=0
   }

   epoll_fd = epoll_create(10);
   if (epoll_fd == -1) {
      printf("\nCA_T2: failed to created epoll file descriptor\n\r");
      return 0;
   }

   rc = pthread_create(&id1, NULL, CAx_thread, NULL);
   if (rc  != 0) {
      printf("\nCA_T2: Adapter thread creation failed with rc = %d \n\r", rc);
      return 0;
   }  // End if rc !=0

   while(1) {
      // check if the A/B switch has been thrown
      for (int i = 0; i < MAXCHAN; i++) {
         if (iobs[i]->abswhist != iobs[i]->abswitch) {
            close_socket(iobs[i], iobs[i]->abswhist);     // Close previous port
            start_listen(iobs[i], iobs[i]->abswitch);     // Listen and add polling
            iobs[i]->abswhist = iobs[i]->abswitch;
         }  // End if iobs[i]
      }  // End for i=0

      for (int i = 0; i < MAXCHAN; i++) {
         if (!IsSocketConnected(iobs[i]->CA_socket[iobs[i]->abswitch])) {
            event.events = EPOLLIN | EPOLLONESHOT;
            event.data.fd = iobs[i]->CA_socket[iobs[i]->abswitch];
            if (epoll_ctl(epoll_fd, EPOLL_CTL_MOD, iobs[i]->CA_socket[iobs[i]->abswitch], &event)) {
               printf("\nModifying polling event error %d for CA%c-%c\n\r", errno, iobs[i]->CA_id, abswid[iobs[i]->abswitch]);
               close(epoll_fd);
               return 0;
            }  // End if !isSocketConnected
         }  //End for i = 0
      }  // End while(1)


      /*******************************************************************************/
      /* This section monitors incomming channel connection requests.                */
      /* If a request is received, it is passed to the connection handler funtion    */
      /*    followed by the creation of a thread that emulates the CA hardware,       */
      /*******************************************************************************/
      event_count = epoll_wait(epoll_fd, events, MAXCHAN*2, 5000);

      for (int i = 0; i < event_count; i++) {
         for (int j = 0; j < MAXCHAN; j++) {                    // For evey possibe connected Channel
            for (int k = 0; k <= CAB; k++) {                    // For port A and B

               if ((events[i].data.fd == iobs[j]->CA_socket[k]) && (iobs[j]->abswitch == k)) {
                  // Accept the incoming connection
                  rc = host_connect(iobs[j], k);
                  if (rc == 0) {
                     // Get device number
                     rc = read_socket(iobs[j]->bus_socket[iobs[j]->abswitch], iobs[j]->buffer, sizeof(iobs[j]->buffer));
                     if (rc != 0) {
                        iobs[j]->devnum = (iobs[j]->buffer[0] << 8) | iobs[j]->buffer[1];
                        printf("CA%c: Connected to device %04X\n\r", iobs[j]->CA_id, iobs[j]->devnum);
                        // Change the CA status to active
                        iobs[j]->CA_active = TRUE;
                     } else {
                        close_socket(iobs[j], k);
                     }  // End if rc !=0
                  } else {
                     close_socket(iobs[j], k);
                  }  // End if rc ==0
               }  // End if events[i].data.fd iobs[0] CAA
            }  // End for k=0 (A/B ports)
         }  // End for j=0  (Channels)

      }  // End for event_count
   }  // End While(1)

   if (close(epoll_fd)) {
      printf("\nCA_T2: failed to close epoll file descriptor\n\r");
      return 0;
   }
   return 0 ;
}

// ************************************************************
// The channel adaptor hardware emulation thread starts here...
// ************************************************************
/* Function to be run as a thread always must have the same
   signature: it has one void* parameter and returns void    */
void *CAx_thread(void *arg) {

   int  pendingrcv;

   printf("\nCA: Adapter thread %d started sucessfully... \n\r", getpid());

   pthread_mutex_lock(&r77_lock);
   CA1_DS_req_L3 = OFF;                    // Chan Adap Data/Status request flag
   CA1_IS_req_L3 = OFF;                    // Chan Adap Initial Sel request flag
   Eregs_Inp[0x77] &= ~0x0028;             // Reset CA L3 interrupt
   pthread_mutex_unlock(&r77_lock);
   Eregs_Inp[0x55]  = 0x0000;              // Reset CA control register
   Eregs_Inp[0x58] |= 0x0008;              // Enable CA I/F A
   Eregs_Inp[0x55] |= 0x0010;              // Flag System Reset
   Eregs_Inp[0x53] |= 0x0200;              // Set not initialized on (in)
   Eregs_Inp[0x76] |= 0x0400;              // Set CA L1 interrupt


   while(1) {
      // We do this for ever and ever...
      /***************************************************************/
      /*  Read channel command from host                             */
      /*                                                             */
      /*  This is the raw version: it assumes no pending operation   */
      /*  Channel status tests need to be added                      */
      /*                                                             */
      /***************************************************************/
      for (int j = 0; j < MAXCHAN; j++) {
         if (Eregs_Out[0x55] & 0x0200) {                 // ATTN request ?
            // Execute ATTN request
            exec_attn();
         }
         if (Eregs_Out[0x57] & 0x0080) {                 // PCI request ?
            // Execute pci request
            exec_pci();
         }
         if (iobs[j]->CA_active == TRUE) {
            pendingrcv = 0;
            ioctl(iobs[j]->bus_socket[iobs[j]->abswitch], FIONREAD, &pendingrcv);
            if (pendingrcv > 0) {
               exec_ccw(iobs[j]);
            }  // End if pendingrcv
         }  // End if iobs[j]
      }  // End for int j
   }  // End of while(1)... */
}

// ************************************************************
// Function to execute Channel Command Words.
// ************************************************************
void exec_ccw(struct IO3705 *iob) {
   int rc, i;
   int cc = 0;
   int sockfc = -1;
   int bufbase, condition;
   int pendingrcv;
   pthread_t id;
   char carnstat, ackbuf;
   uint16_t incwar, outcwar, wdcnt, wdcnttmp, wdcnttot, cacw1;
   uint32_t cacw2;
   uint8_t sense_byte = 0x00;

   /***************************************************************/
   /*    Read channel command from host                           */
   /*                                                             */
   /*    This is the raw version: it assumes no pending operation */
   /*    Channel status tests need to be added                    */
   /*                                                             */
   /***************************************************************/
   if (iob->bus_socket[iob->abswitch] < 1) {
      printf("\nCA%c: Aborting due to loss of active channel connection...\n\r", iob->CA_id);
      return;
   }

   rc = read_socket( iob->bus_socket[iob->abswitch], iob->buffer, sizeof(iob->buffer));

   if (rc == 0) {
      // Host disconnected, get details and print it
      printf("\nCA%c: Error reading CCW, closing channel connection\n\r", iob->CA_id);
      // Change the CA status to inactive
      iob->CA_active = FALSE;

      // Close the bus and tag socket and mark for reuse
      close(iob->bus_socket[iob->abswitch]);
      close(iob->tag_socket[iob->abswitch]);

      iob->bus_socket[iob->abswitch] = -1;
      iob->tag_socket[iob->abswitch] = -1;
   } else {
      // All data transfers are preceded by a CCW.
      ccw.code  =  0x00;
      ccw.code  =  iob->buffer[0];
      ccw.flags =  iob->buffer[4];
      ccw.chain =  iob->buffer[5];
      ccw.count = (iob->buffer[6] << 8) | iob->buffer[7];

      Eregs_Inp[0x5A] = ccw.code << 8;                   // Set Chan command in CA Data Buffer
      Eregs_Inp[0x5C] &= ~0xFFFF;                        // Clear command flags CA Command Register
      Eregs_Inp[0x55] &= ~0x0800;                        // Program Requested L3 interrupt flag should be off
      if ((Adbg_flag == ON) && (Adbg_reg & 0x01))        // Trace channel adapter activities ?
         fprintf(A_trace, "\nCA%c: Channel Command: %02X, length: %d, Flags: %02X, Chained: %02X \n\r",
             iob->CA_id, ccw.code, ccw.count, ccw.flags, ccw.chain);

      // **************************************************************
      // Check and process channel command.
      // **************************************************************
      switch (ccw.code) {
         case 0x00:       // Test I/O
            Eregs_Inp[0x5C] |= 0x8000;                   // Set CA Command Register
            // Send channel end and device end to the host. Sufficient for now (might need to send x00).
            // Send CA return status to host
            carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF);  // Get CA return status
            send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
            break;

         case 0x02:       // Read
            Eregs_Inp[0x55] |= 0x0100;                   // Set Channel Active
            Eregs_Inp[0x5C] |= 0x2000;                   // Set CA Command Register
            Eregs_Inp[0x53] &= 0x00FF;                   // Reset sense byte (in)
            Eregs_Out[0x53] &= 0x00FF;                   // Reset sense byte (out)

            while (reg_bit(0x55, 0x1000) == OFF)
               wait();                                   // Wait for OUTCWAR to become valid
            bufbase = 0;                                 // Set buffer base...
            wdcnttot = 0;                                // ... we will need this in case of chaining

            do {   // While condition remains 0
               condition = 0;
               outcwar = Eregs_Out[0x51];
               cacw1 = (M[outcwar] << 8) | M[outcwar+1] & 0x00FF;      // Get first half of CA Control word
               wdcnt = (cacw1 >> 2) & 0x03FF;            // Fetch Counter
               Eregs_Inp[0x52] &= 0x0000;                // Clear Byte Count Register
               Eregs_Inp[0x52] = wdcnt;

               // Get data fetch address
               cacw2 = 0;
               cacw2 = ((M[outcwar+1] & 0x0003) << 16) + (M[outcwar+2] << 8) + (M[outcwar+3] & 0x00FF);
               if ((Adbg_flag == ON) && (Adbg_reg & 0x01))   // Trace channel adapter activities ?
                  fprintf(A_trace, "OUTCWAR %04X, CW %02X%02X %02X%02X\n\r",
                       outcwar, M[outcwar], M[outcwar+1], M[outcwar+2], M[outcwar+3]);
               Eregs_Out[0x51] = Eregs_Out[0x51] + 4;

               Eregs_Inp[0x5C] |= 0x0080;                // Set command register to OUT Control Word
               Eregs_Inp[0x59]  = cacw2;                 // Load cycle steal address with data load start address
               if ((Adbg_flag == ON) && (Adbg_reg & 0x01)) {   // Trace channel adapter activities ?
                  fprintf(A_trace, "CW %04X\n\r", cacw1);
                  fprintf(A_trace, "Fetch starts at %06X, count = %04X\n\r", cacw2, wdcnt);
               }
               wdcnttmp = wdcnt;                         // Bytes to be transferred for this CW
               wdcnttot = wdcnttot + wdcnt;              // Total byte count
               for (i = 0; i < wdcnttmp; i++) {
                  iob->buffer[bufbase + i] = M[cacw2 + i];   // Load data directly into memory
                  Eregs_Inp[0x59] = Eregs_Inp[0x59] + 1; // Increment cycle steal counter
                  wdcnt = wdcnt - 1;                     // Decrement byte counter
                  Eregs_Inp[0x52] = Eregs_Inp[0x52] - 1;
               }  // End for stmt
               bufbase = bufbase + i;                    // Point after last byte stored in buffer

               if (cacw1 & 0x4000) {                     // If OUT STOP
                  if ((cacw1 & 0x1000) && !(cacw1 & 0x2000))  // Chaining On, Zero Override Off
                     condition = 2;
                  if (!(cacw1 & 0x1000))                 // Chaining Off
                     condition = 1;
                  if ((cacw1 & 0x3000) == 0x3000) {      // Chaning On, Zero Override On
                     condition = 0;
                     while (Ireg_bit(0x77, iob->CA_mask) == ON)
                        wait();                          // Wait for CA1 L3 interrupt reset
                     pthread_mutex_lock(&r77_lock);
                     Eregs_Inp[0x77] |= iob->CA_mask;    // Set CA1 L3 interrupt
                     pthread_mutex_unlock(&r77_lock);
                     CA1_IS_req_L3 = ON;                 // Chan Adap Initial Sel request flag
                     while (Ireg_bit(0x77, 0x008) == ON)
                        wait();                          // Wait for initial selection reset
                  }
               } else {
                  if ((cacw1 & 0x1000) && !(cacw1 & 0x2000))  // Chaining On, Zero Override Off
                     condition = 0;
                  if (!(cacw1 & 0x3000))                 // Chaining Off, Zero Override Off,
                     condition = 1;
                  if (cacw1 & 0x2000)                    // Zero Override On
                     condition = 3;
               }
               if ((Adbg_flag == ON) && (Adbg_reg & 0x01))   // Trace channel adapter activities ?
                  fprintf(A_trace, "Condition = %d\n\r", condition);

            }  while (condition == 0);     // End of do stmt.

            if (condition != 2) {
               while (Ireg_bit(0x77, iob->CA_mask) == ON)
                  wait();                                // Wait for CA1 L3 interrupt reset
               pthread_mutex_lock(&r77_lock);
               Eregs_Inp[0x77] |= iob->CA_mask;          // Set CA1  L3 interrupt
               pthread_mutex_unlock(&r77_lock);
               CA1_IS_req_L3 = ON;                       // Chan Adap Initial Sel request flag
               while (Ireg_bit(0x77, 0x008) == ON)
                  wait();                                // Wait for initial selection reset
            }

            rc = send(iob->bus_socket[iob->abswitch], (void*)&iob->buffer, wdcnttot, 0);

            // Send CA return status to host
            if (condition != 3) {
               carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF);  // Get CA return status
               if (condition == 2)
                  carnstat = CSW_DEND;
               send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
            }
            break;

         case 0x03:       // NO-OP ?
            Eregs_Inp[0x5C] |= 0x1000;                   // Set CA Command Register
            // Send channel end and device end to host. Sufficient for now (might need to send x00).
            while (Ireg_bit(0x77, iob->CA_mask) == ON)
               wait();                                   // Wait for CA1 L3 interrupt request reset
            carnstat = 0x00;
            carnstat |= CSW_DEND;
            send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
            break;

         case 0x04:       // Sense ?
            if (iob->IPL_exception) {                    // if IPL unit exception
               print_regs(iob, "CCW 04 IPL Exception");
               carnstat = CSW_CEND | CSW_DEND | CSW_UEXC;  // set unit exception
               iob->buffer[0] = 0x00;
            } else {
               Eregs_Inp[0x5C] |= 0x0800;                // Set CA Command Register
               while (Ireg_bit(0x77, iob->CA_mask) == ON)
                  wait();                                // Wait for CA1 L3 reset
               //pthread_mutex_lock(&r77_lock);
               //Eregs_Inp[0x77] |= iob->CA_mask;        // Set CA1 L3 interrupt request
               //pthread_mutex_unlock(&r77_lock);
               //CA1_IS_req_L3 = ON;                     // Chan Adap L3 request flag
               //while (Ireg_bit(0x77, iob->CA_mask) == ON)
               //   wait();                              // Wait for L3 interrupt request reset
               print_regs(iob, "CCW 04 L3");
               iob->buffer[0] = Eregs_Out[0x53] >> 8;    // Load sense data byte 0
               if (Eregs_Out[0x57] & 0x0100)             // If not initialized
                  iob->buffer[0] |= 0x02;                // Set not initialized sense
               carnstat = 0x00;
               carnstat = CSW_CEND | CSW_DEND;
               Eregs_Out[0x53] &= ~0x8000;
            }
            if ((Adbg_flag == ON) && (Adbg_reg & 0x01))  // Trace channel adapter activities ?
               fprintf(A_trace, "CA%c: Sending sense Byte 0 %02X \n\r", iob->CA_id, iob->buffer[0]);

            rc = send_socket(iob->bus_socket[iob->abswitch], (void*)&iob->buffer, 1);

            // Send CA return status to host
            send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
            break;

         case 0x05:       // IPL command
         case 0x01:       // Write
         case 0x09:       // Write Break
            Eregs_Inp[0x53] &= 0x00FF;                   // Reset sense byte (in)
            Eregs_Out[0x53] &= 0x00FF;                   // Reset sense byte (out)
            switch (ccw.code) {
               case 0x01:
                  Eregs_Inp[0x5C] |= 0x4000;             // Set CA Command Register
                  Eregs_Inp[0x55] |= 0x0100;             // Set Channel Active
                  break;
               case 0x05:
                  Eregs_Inp[0x5C] |= 0x0001;             // Set CA Command Register
                  Eregs_Inp[0x55] |= 0x0100;             // Set Channel Active
                  Eregs_Out[0x55] |= 0x3000;             // Set INCWAR and OUTCWAR valid for IPL (MAXIROS doesn't do this)
                  while (Ireg_bit(0x77, iob->CA_mask) == ON)
                      wait();                            // Wait for CA1 L3 request reset
                  pthread_mutex_lock(&r77_lock);
                  Eregs_Inp[0x77] |= iob->CA_mask;       // Set CA1 L3 interrupt request
                  pthread_mutex_unlock(&r77_lock);
                  CA1_IS_req_L3 = ON;
                  break;
               case 0x09:
                  Eregs_Inp[0x55] |= 0x0100;             // Set Channel Active
                  Eregs_Inp[0x5C] |= 0x0200;             // Set CA Command Register
                  Eregs_Inp[0x55] |= 0x0040;             // Set Write Break Remember flag
                  break;
            }  // End of nested switch ccw.code

            while (Ireg_bit(0x77, iob->CA_mask) == ON)
                wait();                                  // Wait for CA1 L3 Request reset
            print_regs(iob, "CCW 05, 09, 01 Pre");

            // Read data from host, but first make sure host has finished writing all data to the TCP buffer
            pendingrcv = 0;
            while (pendingrcv != ccw.count)
               ioctl(iob->bus_socket[iob->abswitch], FIONREAD, &pendingrcv);
            rc = recv( iob->bus_socket[iob->abswitch], iob->chainbuf + iob->chainbl, sizeof(iob->chainbuf)-iob->chainbl, 0);
            if ((Adbg_flag == ON) && (Adbg_reg & 0x01))  // Trace channel adapter activities ?
               fprintf(A_trace, "CA%c: received: %d bytes from host\n\r", iob->CA_id, rc);
            iob->bufferl = rc;
            iob->chainbl = iob->chainbl + rc;
            if (ccw.flags & 0x80) {
               if ((Adbg_flag == ON) && (Adbg_reg & 0x01))   // Trace channel adapter activities ?
                  fprintf(A_trace, "CA%c: data chaining \n\r", iob->CA_id);
               carnstat = CSW_CEND | CSW_DEND;   // Set CA return status
               // Send CA return status to host
               send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
               return;
            }
            bufbase = 0;                                 // Set buffer base.
                                                         // We will need this in case of chaining

            iob->bufferl = iob->chainbl;                 // save data chain buffer length
            iob->chainbl = 0;                            // Reset data chain buffer length (no chaining or chain end)
            // ************************************************************
            // Data transfer loop starts here
            // ************************************************************
            print_hex(iob->chainbuf, iob->bufferl);

            while (iob->bufferl) {
               do {   // While condition remains 0
                  condition = 1;
                  if ((Adbg_flag == ON) && (Adbg_reg & 0x01))   // Trace channel adapter activities ?
                     fprintf(A_trace, "InpReg 55 %04X, OutReg 55 %04X\n\r", Eregs_Inp[0x55], Eregs_Out[0x55]);
                  while (reg_bit(0x55, 0x2000) == OFF)   // Wait for INCWAR to become valid
                     wait();

                  incwar = Eregs_Out[0x50];
                  cacw1 = (M[incwar] << 8) | M[incwar+1] & 0x00FF;  // Get first half of CA Control word
                  if ((cacw1 & 0x1000) == 0x0000) {      // If chain bit is off...
                     Eregs_Inp[0x55] &= ~0x2000;         // ...reset INCWAR valid latch...
                     Eregs_Out[0x55] &= ~0x2000;         // ...in both IN and OUT reg
                  }
                  if (cacw1 & 0x2000)                    // If zero override bit is on...
                     Eregs_Inp[0x55] |= 0x4000;          // ...set zero override register flag
                  else                                   // else...
                     Eregs_Inp[0x55] &= ~0x4000;         // ...clear zero override bit

                  if (cacw1 & 0x1000)                    // If chain flag is on...
                     Eregs_Inp[0x55] |= 0x2000;          // ...set INCWAR valid register flag
                  else                                   // else...
                     Eregs_Inp[0x55] &= ~0x2000;         // ...clear INCWAR valid bit

                  wdcnt = 0x0000;                        // clear count
                  wdcnt = (cacw1 >> 2) & 0x03FF;         // Load Counter
                  // Get data fetch address
                  cacw2 = 0;
                  cacw2 = ((M[incwar+1] & 0x0003) << 16) + (M[incwar+2] << 8) + (M[incwar+3] & 0x00FF);
                  if ((Adbg_flag == ON) && (Adbg_reg & 0x01))   // Trace channel adapter activities ?
                     fprintf(A_trace, "INCWAR %04X, CW %02X%02X %02X%02X\n\r", incwar, M[incwar], M[incwar+1], M[incwar+2], M[incwar+3]);
                  Eregs_Out[0x50] = Eregs_Out[0x50] + 4;

                  Eregs_Inp[0x5C] |= 0x0020;             // Set command register to IN Control Word
                  Eregs_Inp[0x59] = cacw2;               // Load cycle steal address with data load start address
                  if ((Adbg_flag == ON) && (Adbg_reg & 0x01)) {   // Trace channel adapter activities ?
                     fprintf(A_trace, "CW %04X\n\r", cacw1);
                     fprintf(A_trace, "Load starts at %06X, count=%04X\n\r", cacw2, wdcnt);
                  }
                  wdcnttmp = wdcnt < iob->bufferl?wdcnt:iob->bufferl;
                  if ((Adbg_flag == ON) && (Adbg_reg & 0x01))   // Trace channel adapter activities ?
                     fprintf(A_trace, "(1) wdcnttmp=%d, wdcnt=%d, iob->bufferl=%d\n\r", wdcnttmp, wdcnt, iob->bufferl);

                  for (i = 0; i < wdcnttmp; i++) {
                     M[cacw2 + i] = iob->chainbuf[bufbase + i];  // Load data directly into memory
                     Eregs_Inp[0x59] = Eregs_Inp[0x59] + 1;  // Increment cycle steal counter
                     wdcnt = wdcnt - 1;                      // Decrement byte counter
                  }  // End For
                  iob->bufferl = iob->bufferl - wdcnttmp;
                  bufbase = bufbase + i;                 // Buffer base points to start of remaing data
                  if ((cacw1 & 0x8000) && wdcnt == 0) {  // If IN and count zero
                     if ((cacw1 & 0x2000) == 0x2000)  {  // Zero Override On
                        //Eregs_Inp[0x55] |= 0x4000;     // Set Zero Override in reg 55
                        condition = 0;
                        pthread_mutex_lock(&r77_lock);
                        Eregs_Inp[0x77] |= iob->CA_mask; // Set CA1 L3 interrupt request
                        pthread_mutex_unlock(&r77_lock);
                        CA1_IS_req_L3 = ON;              // Chan Adap L3 request flag
                        while (Ireg_bit(0x77, iob->CA_mask) == ON)
                           wait();
                     } // End Zero override on
                     if ((cacw1 & 0x3000) == 0x0000)     // Chaining Off, Zero Override Off
                        condition = 1;
                     if ((cacw1 & 0x3000) == 0x1000)     // Chaining On, Zero override off,  count zero
                        condition = 0;
                  }  // End If cacw1

               } while (condition == 0);  // End of Do stmt
            }  // End of while iob->bufferl


            if ((Adbg_flag == ON) && (Adbg_reg & 0x01))  // Trace channel adapter activities ?
               fprintf(A_trace, "CA%c: Data transfer complete, loaded %04X, remainder %04X\n\r",
                 iob->CA_id, wdcnttmp, wdcnt);
            // If byte count is zero, and there is no chaining
            // we will send a L3 interrupt to to CCU, otherwise...
            // ...we will countinue loading data. In case of chaining, we will fetch a new CW

            Eregs_Inp[0x52] &= 0x0000;                   // Clear Byte Count Register
            Eregs_Inp[0x52] = wdcnt;                     // Load Register with Byte count
            while (Ireg_bit(0x77, iob->CA_mask) == ON)
               wait();                                   // Wait for L3 interrupt reset

            //if (ccw.code == 0x05)
            //   Eregs_Out[0x55] &= ~0x3000;             // Reset INCWAR and OUTCWAR valid after IPL
            print_regs(iob, "After Write data transfer");
           //<-if ((ccw.code == 0x05) || (ccw.code == 0x09) ||
           //<-   ((ccw.code == 0x01) && !(Eregs_Inp[0x55] & 0x4000) && (wdcnt != 0))) {
           //? if (ccw.flags & 0x40)  {
            if (wdcnt != 0)  {
               Eregs_Inp[0x55] |= 0x0020;                // Set channel stop
               if ((Adbg_flag == ON) && (Adbg_reg & 0x01))   // Trace channel adapter activities ?
                  fprintf(A_trace, "CA%c: Channel Stop\n\r", iob->CA_id);
            }
            Eregs_Inp[0x55] &= ~0x4000;                  // Reset zero override flag
          //<- }
            if (Eregs_Inp[0x55] & 0x4000)  {             // if Zero Count Override
               if ((Adbg_flag == ON) && (Adbg_reg & 0x01))    // Trace channel adapter activities ?
                  fprintf(A_trace, "CA%c: Zero Override on\n\r", iob->CA_id);
            } // End if Eregs_Inp[0x55]
            //Eregs_Inp[0x55] |= 0x3000;


            pthread_mutex_lock(&r77_lock);
            Eregs_Inp[0x77] |= iob->CA_mask;             // Set CA1 L3 interrupt request
            pthread_mutex_unlock(&r77_lock);
            CA1_IS_req_L3 = ON;
            while (Ireg_bit(0x77, iob->CA_mask) == ON)
               wait();                                   // Wait for CA1 L3 Request reset
            print_regs(iob, "CCW 05, 09, 01 Post");
            if (condition != 2) {                        // If Zero overide is on
               carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF);   // Get CA return status
               // Send CA return status to host
               send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
            }
            break;

         case 0x31:         // Initial Write
         case 0x51:         // Write start 1
         case 0x32:         // Initial Read
         case 0x52:         // Read start 1
         case 0x61:         // Write XID
         case 0x62:         // Read XID
         case 0x93:         // Reset command
         case 0xA3:         // Discontact
         case 0xC3:         // Contact

            Eregs_Inp[0x55] |= 0x0100;                   // Set Channel Active
            Eregs_Inp[0x5C] |= 0x0008;                   // Set non-standard command in CA Command Register
            Eregs_Inp[0x53] &= 0x00FF;                   // Reset sense byte (in)
            Eregs_Out[0x53] &= 0x00FF;                   // Reset sense byte (out)

            while (Ireg_bit(0x77, iob->CA_mask) == ON)
               wait();                                   // Wait for CA1 L3 interrupt request reset
            pthread_mutex_lock(&r77_lock);
            Eregs_Inp[0x77] |= iob->CA_mask;             // Set CA1 L3 interrupt request
            pthread_mutex_unlock(&r77_lock);
            CA1_IS_req_L3 = ON;                          // Chan Adap L3 interrupt request flag
            while (Ireg_bit(0x77, iob->CA_mask) == ON)
               wait();                                   // Wait for L3 iterrupt request reset
            print_regs(iob, "CCW's 31, 32, etc");
            // Send CA return status to host
            carnstat = ((Eregs_Out[0x54] >> 8 ) & 0x00FF); // Get CA return status
            send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
            break;

         default:       // Send command reject sense
            //iob->buffer[0] = SENSE_CR;                  // Load sense data byte 0
            //if (debug_reg & 0x80)
            //   printf("CA%c: Sending sense Byte 0 %02X \n\r", iob->CA_id, iob->buffer[0]);
            //Eregs_Out[0x53] = ((SENSE_CR << 8));           // Set sense byte
              Eregs_Out[0x53] = 0x8200;                      // Set sense byte

            //rc = send_socket(iob->bus_socket[iob->abswitch], (void*)&iob->buffer, 1);
            // Wait for the ACK from the host
            //recv_ack(iob->bus_socket[iob->abswitch]);

            // Send CA return status to host
            carnstat = CSW_CEND + CSW_DEND + CSW_UCHK;   // Get CA return status
            send_carnstat(iob->bus_socket[iob->abswitch], &carnstat, &ackbuf, iob->CA_id);
            break;

      }  // End of switch (ccw.code)
   }  // End of if - else
   return;
}


// ************************************************************
// This subroutine test for 1 bit in a External Output reg.
// If '0' OFF is returned, if 1 'ON' returned.
// ************************************************************
int reg_bit(int reg, int bit_mask) {
   if ((Eregs_Out[reg] & bit_mask) == 0x00)
      return(OFF);
   else
      return(ON);
}


// ************************************************************
// This subroutine test for 1 bit in a External Input reg.
// If '0' OFF is returned, if 1 'ON' returned.
// ************************************************************
int Ireg_bit(int reg, int bit_mask) {
   if ((Eregs_Inp[reg] & bit_mask) == 0x00)
      return(OFF);
   else
      return(ON);
}


// ************************************************************
// This subroutine waits 1 usec
// ************************************************************
void wait() {
   usleep(1);
   return;
}

