There are two main updates with Release 2 of the IBM 3705 SIMH emulator:

The separation of the 3274 emulation code from the 3705 emulation code. This means that the 3274 emulation can run on a different host as the 3705 host.

The addition of BSC support in the form of a 3271 cluster emulation (BSC addition inspired by Mattis Lind) 
      
Below is a detailed overview of the Release 2 updates:

Central Control Unit (CCU/CPU)
Various fixes to instructions that use 18 bit addressing. The 3750 Emulator can now run with up to 256K memory.
Hardware diagnostic capabilities added. This enables IFLOADRN to run with DIAG=Y6 or DIAG=Y8  
Cycle Utilization Counter added. This is used by the new 3705 front-panel code.
Register (x’??’) defining the 3705 storage size is now automatically set based on the 3705 startup configuration file.
Channel Adapter
Code rationalization resulting in significant throughput improvements. I.e. The NCP will load much faster.
Support for CCW data chaining.
Scanner
Addition of BSC support
SDLC LIC
Complete rework to support a connection with the 3274 emulation over TCP/IP.
BSC LIC
Added in support of BSC. Supports a connection to the 3271 emulator over TCP/IP
3274
Code separated from the 3705 code. The 3274 may now run on a different host (but also the same) and connect over TCP/IP to the 3705 SDLC line.
Multi-PU and Multi-LU support. On a single line, multiple PU’s each with multiple LU’s can be defined in the NCP. Multiple LU’s can be useful if LU’s e.g. have different logmodes, or if you want to connect a 3287 next to one or more 3270’s.
Various improvements/enhancements to the SDLC protocol handling.
3271
Addition of a 3271 cluster emulation. As with the 3274, can run on a different host and connect via TCP/IP to the BSC line.
3705 Front-Panel
Completely re-coded. The previous 3270 based code was unmaintainable. The new front panel now runs on the Xterm screen used to start the 3705.
