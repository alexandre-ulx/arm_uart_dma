# Implementation of DMA UART Idle


In this project, the functionality of UART Idle DMA was implemented to improve the efficiency of serial communication. DMA (Direct Memory Access) is used to transfer data between memory and peripherals without direct intervention from the CPU, reducing processing load.

In the `main.c` file, a transmission and validation package was developed using UART Idle DMA. The package consists of sending a sequence of data through UART and verifying if the transmission was successfully completed.

The implementation of UART Idle DMA involves configuring relevant registers, such as the DMA control register and UART configuration registers. Additionally, it is necessary to define appropriate transmission and reception buffers.

By using UART Idle DMA, it is possible to take advantage of the idle time of UART to perform other tasks, increasing the overall system efficiency.


