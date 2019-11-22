/*
 * Serial.cpp
 *
 *  Created on: Nov 3, 2019
 *      Author: DomanR
 */

#include "Serial.h"
#include "driverlib.h"

char Serial::string[10];
unsigned int Serial::i;

void Serial::init(void)
{
    //SMCLK = 1MHz, Baudrate = 115200
    //UCBRx = 8, UCBRFx = 0, UCBRSx = 0xD6, UCOS16 = 0
    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 6;
    param.firstModReg = 8;
    param.secondModReg = 0x20;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param)) {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);
}

void Serial::puts(char* input)
{
    i = 0;
    do
    {
        string[i] = input[i];
    } while (input[i++] != NULL);
    i = 0;
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, string[i++]);
}

Serial::Serial(void) { }

//******************************************************************************
//
//This is the USCI_A0 interrupt vector service routine.
//
//******************************************************************************
#pragma vector=USCI_A0_VECTOR
__interrupt void Serial::EUSCI_A0_ISR(void)
{
    switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    {
    case USCI_NONE:
        break;
    case USCI_UART_UCRXIFG:
        break;
    case USCI_UART_UCTXIFG:
        if (string[i] == NULL)
        {
            EUSCI_A_UART_disableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);
        }
        else
        {
            EUSCI_A_UART_transmitData(EUSCI_A0_BASE, string[i++]);
        }
        break;
    case USCI_UART_UCSTTIFG:
        break;
    case USCI_UART_UCTXCPTIFG:
        break;
    }
}
