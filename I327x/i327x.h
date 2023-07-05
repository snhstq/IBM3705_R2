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

   i327x.h (c) Copyright Edwin Freekenhorst & Henk Stegeman
   Includes definitions take from Max H. Parke's comm3705.h
*/
/*-------------------------------------------------------------------*/
/* IBM 3271/3274 common definitions                                  */
/*-------------------------------------------------------------------*/
#define MAXSNAPU       2         /* Maximum number of SNA PU's           */
#define MAXCLSTR       2         /* Maximum number of BSC clusters's     */
#define MAXLU          4         /* Maximum nr of LU's per PU or cluster */
#define SDLCLBASE    37520       /* Port number of first SDLC line       */
#define BSCLBASE     37530       /* Port number of first BSC line        */

#define BUFLEN_3270  65536       /* 3270 Send/Receive buffer  */
#define BUFLEN_1052    150       /* 1052 Send/Receive buffer  */

typedef  uint8_t     BYTE;
typedef  uint16_t    U16;        /* unsigned 16-bits  */
typedef  uint32_t    U32;        /* unsigned 32-bits  */
typedef  uint64_t    U64;        /* unsigned 64-bits  */
#define CRC_POLY_16  0xA001      /* Polynominal for CRC-16 checking */

#define SNA_CMD        1         /* THRH with SNA command     */
#define SNA_SENSE      2         /* THRH with SENSE code      */
#define DATA_ONLY      3         /* THRH with 3270 data, only segment  */
#define DATA_FIRST     4         /* THRH with 3270 data, 1st segment   */
#define DATA_MIDDLE    5         /* THRH with 3270 data, middle or last seg. */
#define DATA_LAST      6         /* THRH with 3270 data, middle or last seg. */

#ifndef TRUE
   #define TRUE        1
#endif
#ifndef FALSE
   #define FALSE       0
#endif
#define OFF            0
#define ON             1

#define FILLED         1
#define EMPTY          0

/*-------------------------------------------------------------------*/
/*3271 / 3274 Data Structure                                         */
/*-------------------------------------------------------------------*/
struct CB327x {
   int      lu_fd[MAXLU];
   BYTE     punum;
   BYTE     lunum;
   BYTE     lunumr;
   int      pu_fd;
   int      epoll_fd;
   uint32_t actlu[MAXLU];
   uint32_t readylu[MAXLU];
   uint32_t reqcont[MAXLU];
   uint32_t is_3270[MAXLU];
   uint32_t rlen3270[MAXLU];           /* size of data in 3270 receive buffer   */
   uint32_t bindflag[MAXLU];
   uint32_t initselfflag[MAXLU];
   uint32_t telnet_opt[MAXLU];         /* expecting telnet option char          */
   uint32_t telnet_iac[MAXLU];         /* expecting telnet command char         */
   uint32_t telnet_int[MAXLU];         /* telnet intterupt received             */
   uint32_t eol_flag[MAXLU];           /* Carriage Return received              */
   int      ncpa_sscp_seqn;
   int      lu_lu_seqn[MAXLU];
   uint8_t  telnet_cmd[MAXLU];         /* telnet command                        */
   uint8_t  not_ready[MAXLU];          /* Not Ready flag                        */
   uint8_t  dri[MAXLU];                /* Definitive Response Indicator         */
   uint8_t  chaining[MAXLU];           /* Chaining Indicator                    */
   uint8_t  seq_Nr;                    /* Sequence Number Received              */
   uint8_t  seq_Ns;                    /* Sequence Number Send                  */
   uint8_t  sscp_addr0;
   uint8_t  sscp_addr1;
   uint8_t  pu_addr0;
   uint8_t  pu_addr1;
   uint8_t  lu_addr0;
   uint8_t  lu_addr1;
   uint8_t  daf_addr1[MAXLU];
   uint8_t  last_lu;
};

/*-------------------------------------------------------------------*/
/* IBM 3270 Data Structure                                           */
/*-------------------------------------------------------------------*/
struct IO3270 {
   uint8_t  inpbuf[65536];
   uint32_t inpbufl;
};

/*-------------------------------------------------------------------*/
/* Telnet command definitions                                        */
/*-------------------------------------------------------------------*/
#define BINARY          0       /* Binary Transmission */
#define IS              0       /* Used by terminal-type negotiation */
#define SEND            1       /* Used by terminal-type negotiation */
#define ECHO_OPTION     1       /* Echo option */
#define SUPPRESS_GA     3       /* Suppress go-ahead option */
#define TIMING_MARK     6       /* Timing mark option */
#define TERMINAL_TYPE   24      /* Terminal type option */
#define NAWS            31      /* Negotiate About Window Size */
#define EOR             25      /* End of record option */
#define EOR_MARK        239     /* End of record marker */
#define SE              240     /* End of subnegotiation parameters */
#define NOP             241     /* No operation */
#define DATA_MARK       242     /* The data stream portion of a Synch.
                                   This should always be accompanied
                                   by a TCP Urgent notification */
#define BRK             243     /* Break character */
#define IP              244     /* Interrupt Process */
#define AO              245     /* Abort Output */
#define AYT             246     /* Are You There */
#define EC              247     /* Erase character */
#define EL              248     /* Erase Line */
#define GA              249     /* Go ahead */
#define SB              250     /* Subnegotiation of indicated option */
#define WILL            251     /* Indicates the desire to begin
                                   performing, or confirmation that
                                   you are now performing, the
                                   indicated option */
#define WONT            252     /* Indicates the refusal to perform,
                                   or continue performing, the
                                   indicated option */
#define DO              253     /* Indicates the request that the
                                   other party perform, or
                                   confirmation that you are expecting
                                   the other party to perform, the
                                   indicated option */
#define DONT            254     /* Indicates the demand that the
                                   other party stop performing,
                                   or confirmation that you are no
                                   longer expecting the other party
                                   to perform, the indicated option  */
#define IAC             255     /* Interpret as Command              */

