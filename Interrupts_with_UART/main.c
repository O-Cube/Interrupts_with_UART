//======================================================================================================
// Author  : Obed Oyandut
// Date    : 04.08.2022
// Version : v1
//======================================================================================================
// This program uses TIVA TM4C1294XL Evaluation Board
//======================================================================================================
// This program demonstrates interrupts in microcontroller and particularly in UART.
// Interrupts are desirable in microcontroller designs because they make efficient use of the
// processor. The processor only needs to service ISR from different peripherals. When there are
// ISR the processor can be put to sleep thus saving energy.
// The testing of this code is done with Realterm. The speed of the communication line is 115200MHz
// and a 8N1 format. The debugging is done in code composer studio.
// TIVA TM4C1294XL uses a single channel for both Tx and Rx interrupts. Software has to determine what
// causes the interrupt by using the MIS.
//========================================================================================================
//                     ! IMPORTANT !
// This program runs endless. Stop with the "Red Square Button"
// in Debug Mode (Terminate = CTRL + F2)
//========================================================================================================

#include "inc/tm4c1294ncpdt.h"
#include <stdint.h>
#include <stdio.h>

// Size of receive buffer and length of transmitted/received characters in a single ISR

#define LEN 32
#define RXTXLEN 12

// Buffer declarations

unsigned char rxBuffer[LEN];
unsigned char txBuffer[] = "send more...";

//=========================================================================================
// Keeps count of the number of ASCII character received/sent
//=========================================================================================

unsigned char i;

//=========================================================================================
// ISR of UART2:
// When an event happens, the ISR is executed. The processor does not know whether the Rx/Tx
// caused the interrupt. What causes interrupt is determined from MIS. The interrupt is cleared
// using ICR. An interrupt event is produced when Rx FIFO is 3/4 full and when the Tx FIFO is em
// empty. During a single ISR, up to 12 character is read from Rx FIFO or up to 12 characters is
// written into the Tx FIFO.
//==========================================================================================

void UartRxTxHandler(void) {
    if (UART2_MIS_R & 0x10) {
        UART2_ICR_R |= (0x01<<4);
        for(i=0;i<RXTXLEN;i++) {
            rxBuffer[i] = UART2_DR_R;
        }
        rxBuffer[RXTXLEN] = '\0';
        printf("Payload: %s\n", rxBuffer);
    }

    if (UART2_MIS_R & 0x20) {
        UART2_ICR_R |= (0x01<<5);
        for(i=0;i<RXTXLEN;i++) {
            UART2_DR_R = txBuffer[i];
        }
    }
}

//=========================================================================================
// Configure UART2. First assign a clock to UART2. Wait for the clock
// to be assigned using a blocking while loop. Disable UART2 to perform configurations.
// Set the bit rate to 115200bps. Remember clock speed is 16MHz. Receiver speed is 16 times
// faster than transmit speed. Set the frame format to 8N1. Turn on all 16 FIFOs for receive
// transmit. Re enable UART2 after configuration.
//==========================================================================================

void configUart2(void) {
    SYSCTL_RCGCUART_R |= (1<<2);
    while((SYSCTL_PRUART_R & (1<<2))==0);
    UART2_CTL_R &= ~(1<<0);
    UART2_IBRD_R = 8;
    UART2_FBRD_R = 44;
    UART2_LCRH_R = 0x00000070;

    //===========================================================================================
    // Configuration UART2 interrupts:
    // Define the event that caused the interrupt. In this case Rx FIFO is 3/4 full and Tx FIFO is
    // empty. The IFLS is used to configure these interrupt. In the CTRL, set EOT so that TxRIS is
    // produced when the Tx FIFO is empty. Allow default settings of TxRIS in IFLS. Mask the interrupt in
    // IM and enable it in NVIC_EN1. Enable UART after configuration.
    // =================================================================================================

    UART2_IM_R |= 0x30;
    NVIC_EN1_R |= (0x1<<1);
    UART2_IFLS_R |= 0x18; // rx is 3/4 full and tx 3/4 empty
    UART2_CTL_R |= 0x311;
}

//==========================================================================================
// Configure port D for UART. Assign clock to PD. Wait for clock to become stable using
//  a blocking while loop. Set alternate select of P(4) and PD(5). Set alternate function
// of PD(4) and PD(5)to UART.
//==========================================================================================

void configPortD(void) {
    SYSCTL_RCGCGPIO_R |= (1<<3);
    while((SYSCTL_PRGPIO_R & (1<<3)) == 0);
    GPIO_PORTD_AHB_DEN_R |= 0x030;
    GPIO_PORTD_AHB_AFSEL_R |= 0x030;
    GPIO_PORTD_AHB_PCTL_R |= 0x110000;
}

void main(void) {

    //==========================================================================================
    // Initialization UART2 and PD
    //==========================================================================================

    configUart2();
    configPortD();

    UART2_DR_R = '>';

    while(1)
    {

        //========================================================================================================
        // Infinite loop. Processor waits for evvents to occur. When these events occur, the processor goes into
        // an interrupt service routine.
        //========================================================================================================

    }
}
