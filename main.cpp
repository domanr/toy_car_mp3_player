/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//******************************************************************************
//!  EUSCI_A0 External Loopback test using EUSCI_A_UART_init API
//!
//!  Description: This demo connects TX to RX of the MSP430 UART
//!  The example code shows proper initialization of registers
//!  and interrupts to receive and transmit data.
//!
//!  SMCLK = MCLK = BRCLK = DCOCLKDIV = ~1MHz, ACLK = 32.768kHz
//!
//!
//!           MSP430FR2xx_4xx Board
//!             -----------------
//!       RST -|          UCA0TXD|----|
//!            |                 |    |
//!            |                 |    |
//!            |          UCA0RXD|----|
//!            |                 |
//!
//! This example uses the following peripherals and I/O signals. You must
//! review these and change as needed for your own board:
//! - UART peripheral
//! - GPIO Port peripheral (for UART pins)
//! - UCA0TXD
//! - UCA0RXD
//!
//! This example uses the following interrupt handlers. To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - USCI_A0_VECTOR.
//******************************************************************************
#include "driverlib.h"
#include "Board.h"
#include "Serial.h"
#include "DFPlayer.h"

//*****************************************************************************
//
//Target frequency for MCLK in kHz
//
//*****************************************************************************
#define CS_MCLK_DESIRED_FREQUENCY_IN_KHZ   1000

//*****************************************************************************
//
//MCLK/FLLRef Ratio
//
//*****************************************************************************
#define CS_MCLK_FLLREF_RATIO   30

#define GPIO_PORT_MUSIC_BUTTON  GPIO_PORT_P1
#define GPIO_PIN_MUSIC_BUTTON   GPIO_PIN6
#define GPIO_PORT_VOL_DOWN_BUTTON  GPIO_PORT_P2
#define GPIO_PIN_VOL_DOWN_BUTTON   GPIO_PIN3
#define GPIO_PORT_VOL_UP_BUTTON  GPIO_PORT_P2
#define GPIO_PIN_VOL_UP_BUTTON   GPIO_PIN7

void WDT_Init(void);
void Clock_Init(void);
void GPIO_Init(void);

Serial serial;
DFPlayer dfplayer;

void main(void)
{
    WDT_Init();
    Clock_Init();
    GPIO_Init();

    /*
     * Disable the GPIO power-on default high-impedance mode to activate
     * previously configured port settings
     */
    PMM_unlockLPM5();

    serial.init();
    dfplayer.setSerial(serial);

    //Enter LPM0, enable interrupts
    __bis_SR_register(LPM0_bits + GIE);
    //For debugger
    __no_operation();
}

void WDT_Init(void)
{
    //Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);
}

void Clock_Init(void)
{
    //Create struct variable to store proper software trim values
    CS_initFLLParam param = {0};
    //Set DCO FLL reference = REFO
    CS_initClockSignal(CS_FLLREF, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    //Set ACLK = REFOCLK with clock divider of 1
    CS_initClockSignal(CS_ACLK,CS_REFOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    //Set Ratio/Desired MCLK Frequency, initialize DCO, save trim values
    CS_initFLLCalculateTrim(CS_MCLK_DESIRED_FREQUENCY_IN_KHZ, CS_MCLK_FLLREF_RATIO, &param);
    //Clear all OSC fault flag
    CS_clearAllOscFlagsWithTimeout(1000);
    //Reload DCO trim values that were calculated earlier
    CS_initFLLLoadTrim(CS_MCLK_DESIRED_FREQUENCY_IN_KHZ, CS_MCLK_FLLREF_RATIO, &param);
    //Clear all OSC fault flag
    CS_clearAllOscFlagsWithTimeout(1000);
}

void GPIO_Init(void)
{
    //Configure UART pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_UCA0TXD, GPIO_PIN_UCA0TXD, GPIO_FUNCTION_UCA0TXD);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_UCA0RXD, GPIO_PIN_UCA0RXD, GPIO_FUNCTION_UCA0RXD);

    //Configure LED1
    GPIO_setAsOutputPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
    GPIO_setOutputLowOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);

    //Configure the music button
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_MUSIC_BUTTON, GPIO_PIN_MUSIC_BUTTON);
    GPIO_enableInterrupt(GPIO_PORT_MUSIC_BUTTON, GPIO_PIN_MUSIC_BUTTON);
    GPIO_selectInterruptEdge(GPIO_PORT_MUSIC_BUTTON, GPIO_PIN_MUSIC_BUTTON, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_MUSIC_BUTTON, GPIO_PIN_MUSIC_BUTTON);

    //Configure the volume down button
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_VOL_DOWN_BUTTON, GPIO_PIN_VOL_DOWN_BUTTON);
    GPIO_enableInterrupt(GPIO_PORT_VOL_DOWN_BUTTON, GPIO_PIN_VOL_DOWN_BUTTON);
    GPIO_selectInterruptEdge(GPIO_PORT_VOL_DOWN_BUTTON, GPIO_PIN_VOL_DOWN_BUTTON, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_VOL_DOWN_BUTTON, GPIO_PIN_VOL_DOWN_BUTTON);

    //Configure the volume up button
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_VOL_UP_BUTTON, GPIO_PIN_VOL_UP_BUTTON);
    GPIO_enableInterrupt(GPIO_PORT_VOL_UP_BUTTON, GPIO_PIN_VOL_UP_BUTTON);
    GPIO_selectInterruptEdge(GPIO_PORT_VOL_UP_BUTTON, GPIO_PIN_VOL_UP_BUTTON, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_VOL_UP_BUTTON, GPIO_PIN_VOL_UP_BUTTON);
}

//******************************************************************************
//
//This is the PORT1_VECTOR interrupt vector service routine
//
//******************************************************************************
#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR (void)
{
    //LED1 = toggle
    GPIO_toggleOutputOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);

    switch(dfplayer.getPlayingStatus())
    {
    case DFPL_STATUS_PAUSED:
        dfplayer.play();
        dfplayer.setPlayingStatus(DFPL_STATUS_PLAYING);
        break;
    case DFPL_STATUS_PLAYING:
        dfplayer.pause();
        dfplayer.setPlayingStatus(DFPL_STATUS_PAUSED);
        break;
    default:
        break;
    }

    //S1 IFG cleared
    GPIO_clearInterrupt(GPIO_PORT_MUSIC_BUTTON, GPIO_PIN_MUSIC_BUTTON);
}

//******************************************************************************
//
//This is the PORT2_VECTOR interrupt vector service routine
//
//******************************************************************************
#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR (void)
{
    if (GPIO_getInterruptStatus (GPIO_PORT_VOL_DOWN_BUTTON, GPIO_PIN_VOL_DOWN_BUTTON)) {
        dfplayer.decreaseVol();
        GPIO_clearInterrupt(GPIO_PORT_VOL_DOWN_BUTTON, GPIO_PIN_VOL_DOWN_BUTTON);
    }

    if (GPIO_getInterruptStatus (GPIO_PORT_VOL_UP_BUTTON, GPIO_PIN_VOL_UP_BUTTON)) {
        dfplayer.increaseVol();
        GPIO_clearInterrupt(GPIO_PORT_VOL_UP_BUTTON, GPIO_PIN_VOL_UP_BUTTON);
    }
}
