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

#define BUTTON_VOL_DOWN_PORT    GPIO_PORT_P2
#define BUTTON_VOL_DOWN_PIN     GPIO_PIN3

#define BUTTON_RIGHT_PORT       GPIO_PORT_P1
#define BUTTON_RIGHT_PIN        GPIO_PIN2

#define BUTTON_LEFT_PORT        GPIO_PORT_P2
#define BUTTON_LEFT_PIN         GPIO_PIN4

#define BUTTON_MUSIC_PORT       GPIO_PORT_P2
#define BUTTON_MUSIC_PIN        GPIO_PIN7

#define BUTTON_HORN_PORT        GPIO_PORT_P1
#define BUTTON_HORN_PIN         GPIO_PIN3

#define GAS_PEDAL_PORT          GPIO_PORT_P2
#define GAS_PEDAL_PIN           GPIO_PIN2

#define DEFAULT_VOLUME          18

void WDT_Init(void);
void Clock_Init(void);
void GPIO_Init(void);
void ConfigureButton(uint8_t port, uint16_t pin);

void ButtonVolDownHandler(void);
void ButtonLeftHandler(void);
void ButtonRightHandler(void);
void ButtonMusicHandler(void);
void ButtonHornHandler(void);
void GasPedalHandler(void);

void DataReceivedHandler(void);

Serial serial;
DFPlayer dfplayer;

uint16_t systemCounter = 0;

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

    serial.init(&DataReceivedHandler);
    dfplayer.setSerial(serial);

    //Enter LPM0, enable interrupts
    __bis_SR_register(LPM0_bits + GIE);
    //For debugger
    __no_operation();
}

void WDT_Init(void)
{
    //ACLK / 512 ==> cycle time = 15.625 ms
    WDT_A_initIntervalTimer(WDT_A_BASE, WDT_A_CLOCKSOURCE_ACLK, WDT_A_CLOCKDIVIDER_512);
    WDT_A_start(WDT_A_BASE);
    SFR_clearInterrupt(SFR_WATCHDOG_INTERVAL_TIMER_INTERRUPT);
    SFR_enableInterrupt(SFR_WATCHDOG_INTERVAL_TIMER_INTERRUPT);
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

    //Configure LED2
    GPIO_setAsOutputPin(GPIO_PORT_LED2, GPIO_PIN_LED2);
    GPIO_setOutputLowOnPin(GPIO_PORT_LED2, GPIO_PIN_LED2);

    //Configure the music button
    ConfigureButton(BUTTON_MUSIC_PORT, BUTTON_MUSIC_PIN);

    //Configure the left button
    ConfigureButton(BUTTON_LEFT_PORT, BUTTON_LEFT_PIN);

    //Configure the right button
    ConfigureButton(BUTTON_RIGHT_PORT, BUTTON_RIGHT_PIN);

    //Configure the horn button
    ConfigureButton(BUTTON_HORN_PORT, BUTTON_HORN_PIN);

    //Configure the volume down button
    ConfigureButton(BUTTON_VOL_DOWN_PORT, BUTTON_VOL_DOWN_PIN);

    ConfigureButton(GAS_PEDAL_PORT, GAS_PEDAL_PIN);
}

void ConfigureButton(uint8_t port, uint16_t pin)
{
    //Configure the music button
    GPIO_setAsInputPinWithPullUpResistor(port, pin);
    GPIO_enableInterrupt(port, pin);
    GPIO_selectInterruptEdge(port, pin, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(port, pin);
}

//******************************************************************************
//
//This is the PORT1_VECTOR interrupt vector service routine
//
//******************************************************************************
#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR (void)
{
    ButtonHornHandler();
    ButtonRightHandler();
}

//******************************************************************************
//
//This is the PORT2_VECTOR interrupt vector service routine
//
//******************************************************************************
#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR (void)
{
    ButtonVolDownHandler();
    ButtonMusicHandler();
    ButtonLeftHandler();
    GasPedalHandler();
}

//Watchdog Timer interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void WDT_A_ISR (void)
{
    systemCounter++;

    if(dfplayer.checkResponse()) {
        if(DFPL_CMD_RESP_ONLINE == dfplayer.readRespCommand()) {
            GPIO_setOutputHighOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
            dfplayer.setVol(DEFAULT_VOLUME);
            dfplayer.setInitStatus(DFPL_INITIALIZED);
        }
    }
}

void ButtonVolDownHandler(void)
{
    if (GPIO_getInterruptStatus (BUTTON_VOL_DOWN_PORT, BUTTON_VOL_DOWN_PIN)) {
        dfplayer.decreaseVol();
        GPIO_clearInterrupt(BUTTON_VOL_DOWN_PORT, BUTTON_VOL_DOWN_PIN);
    }
}

void ButtonLeftHandler(void)
{
    if (GPIO_getInterruptStatus (BUTTON_LEFT_PORT, BUTTON_LEFT_PIN)) {
        dfplayer.previous();
        GPIO_clearInterrupt(BUTTON_LEFT_PORT, BUTTON_LEFT_PIN);
    }
}

void ButtonRightHandler(void)
{
    if (GPIO_getInterruptStatus (BUTTON_RIGHT_PORT, BUTTON_RIGHT_PIN)) {
        dfplayer.next();
        GPIO_clearInterrupt(BUTTON_RIGHT_PORT, BUTTON_RIGHT_PIN);
    }
}

void ButtonMusicHandler(void)
{
    if (GPIO_getInterruptStatus (BUTTON_MUSIC_PORT, BUTTON_MUSIC_PIN)) {
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
        GPIO_clearInterrupt(BUTTON_MUSIC_PORT, BUTTON_MUSIC_PIN);
    }
}

void ButtonHornHandler(void)
{
    if (GPIO_getInterruptStatus (BUTTON_HORN_PORT, BUTTON_HORN_PIN)) {
        dfplayer.playAdvertisment(1);
        GPIO_clearInterrupt(BUTTON_HORN_PORT, BUTTON_HORN_PIN);
    }
}

void GasPedalHandler(void)
{
    if (GPIO_getInterruptStatus (GAS_PEDAL_PORT, GAS_PEDAL_PIN)) {
            dfplayer.playAdvertisment(3);
            GPIO_clearInterrupt(GAS_PEDAL_PORT, GAS_PEDAL_PIN);
    }
}

void DataReceivedHandler(void) {}
