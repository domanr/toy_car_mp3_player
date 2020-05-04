/**
 * @file main.cpp
 * @author Rajmund Doman (domanr@gmail.com)
 *
 * @brief The main module of the toy car mp3 player
 *
 * The MSP430 detects the buttons pressed on the toy car and sends serial
 * commands to the DFPlayer mini mp3 player.
 * The advertisement functionality of the DFPlayer module is utilized to
 * interrupt the currently payed song when the horn or the throttle pedal is
 * pressed. The corresponding sound effect is played, then the playback of the
 * song goes on.
 *
 */

#include "driverlib.h"
#include "Board.h"
#include "Serial.h"
#include "DFPlayer.h"

/** @brief Target frequency for MCLK in kHz */
#define CS_MCLK_DESIRED_FREQUENCY_IN_KHZ   1000

/** @brief MCLK/FLLRef Ratio */
#define CS_MCLK_FLLREF_RATIO   30

/** @brief GPIO port and pin of the volume down button */
#define BUTTON_VOL_DOWN_PORT    GPIO_PORT_P2
#define BUTTON_VOL_DOWN_PIN     GPIO_PIN3

/** @brief GPIO port and pin of the right button on the steering wheel */
#define BUTTON_RIGHT_PORT       GPIO_PORT_P1
#define BUTTON_RIGHT_PIN        GPIO_PIN2

/** @brief GPIO port and pin of the left button on the steering wheel */
#define BUTTON_LEFT_PORT        GPIO_PORT_P2
#define BUTTON_LEFT_PIN         GPIO_PIN4

/** @brief GPIO port and pin of the music button on the steering wheel */
#define BUTTON_MUSIC_PORT       GPIO_PORT_P2
#define BUTTON_MUSIC_PIN        GPIO_PIN7

/** @brief GPIO port and pin of the horn button on the steering wheel */
#define BUTTON_HORN_PORT        GPIO_PORT_P1
#define BUTTON_HORN_PIN         GPIO_PIN3

/** @brief GPIO port and pin of the gas pedal */
#define GAS_PEDAL_PORT          GPIO_PORT_P2
#define GAS_PEDAL_PIN           GPIO_PIN2

/** @brief The default volume which is set after startup */
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

/** @brief The serial port used for communication with the DFPlayer */
Serial serial;
/** @brief Instance of the DFPlayer class */
DFPlayer dfplayer;
/** @brief The system counter is increased by a hardware time and used to trigger certain events */
uint16_t systemCounter = 0;

void main(void)
{
    WDT_Init();
    Clock_Init();
    GPIO_Init();

    // Disable the GPIO power-on default high-impedance mode to activate previously configured port settings
    PMM_unlockLPM5();

    serial.init(&DataReceivedHandler);
    dfplayer.setSerial(serial);

    //After initialization enter LPM0 and enable interrupts
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
    CS_initFLLParam param = {0};

    CS_initClockSignal(CS_FLLREF, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    CS_initClockSignal(CS_ACLK,CS_REFOCLK_SELECT,CS_CLOCK_DIVIDER_1);
    CS_initFLLCalculateTrim(CS_MCLK_DESIRED_FREQUENCY_IN_KHZ, CS_MCLK_FLLREF_RATIO, &param);
    CS_clearAllOscFlagsWithTimeout(1000);
    CS_initFLLLoadTrim(CS_MCLK_DESIRED_FREQUENCY_IN_KHZ, CS_MCLK_FLLREF_RATIO, &param);
    CS_clearAllOscFlagsWithTimeout(1000);
}

void GPIO_Init(void)
{
    //Configure UART pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_UCA0TXD, GPIO_PIN_UCA0TXD, GPIO_FUNCTION_UCA0TXD);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_UCA0RXD, GPIO_PIN_UCA0RXD, GPIO_FUNCTION_UCA0RXD);

    //Configure LED1 (used for debug purposes)
    GPIO_setAsOutputPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
    GPIO_setOutputLowOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);

    //Configure LED2 (used for debug purposes)
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

    //Configure the gas pedal
    ConfigureButton(GAS_PEDAL_PORT, GAS_PEDAL_PIN);
}

void ConfigureButton(uint8_t port, uint16_t pin)
{
    GPIO_setAsInputPinWithPullUpResistor(port, pin);
    GPIO_enableInterrupt(port, pin);
    GPIO_selectInterruptEdge(port, pin, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(port, pin);
}

/**
 * @brief This is the PORT1_VECTOR interrupt vector service routine
*/
#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR (void)
{
    ButtonHornHandler();
    ButtonRightHandler();
}

/**
 * @brief This is the PORT2_VECTOR interrupt vector service routine
*/
#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR (void)
{
    ButtonVolDownHandler();
    ButtonMusicHandler();
    ButtonLeftHandler();
    GasPedalHandler();
}

/**
 * @brief This is the watchdog timer interrupt vector service routine
*/
#pragma vector=WDT_VECTOR
__interrupt void WDT_A_ISR (void)
{
    systemCounter++;

    if(dfplayer.checkResponse()) {
        uint8_t response;
        response = dfplayer.readRespCommand();
        switch(response)
        {
        case DFPL_CMD_RESP_ONLINE:
            dfplayer.setVol(DEFAULT_VOLUME);
            break;
        case DFPL_CMD_RESP_FEEDBACK:
            if(DFPL_CMD_SET_VOL == dfplayer.getLastSentCmd()) {
                // Automatic start after power-up
                dfplayer.next();
            }
            break;
        case DFPL_CMD_RESP_FINISHED_USB:
        case DFPL_CMD_RESP_FINISHED_SD:
            dfplayer.setPlayingStatus(DFPL_STATUS_PAUSED);
            break;
        default:
            break;
        }
    }

    if(dfplayer.getPlayingStatus() == DFPL_STATUS_PLAYING) {
        GPIO_setOutputHighOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
    } else {
        GPIO_setOutputLowOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
    }

    // Reset the counter every 60 sec
    if(systemCounter == (64 * 60)) {
        systemCounter = 0;
        if(dfplayer.getPlayingStatus() == DFPL_STATUS_PAUSED) {
            dfplayer.next();
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
            break;
        case DFPL_STATUS_PLAYING:
            dfplayer.pause();
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
