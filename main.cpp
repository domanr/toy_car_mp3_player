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
 * Initialization sequence:
 * Phase 1: waiting for online message from the DFPlayer
 * Phase 2: when the DFPlayer is online, set the default volume
 * Phase 3: when the volume is set, query the number of tracks
 * Phase 4: when the number of tracks is known, start playing the first track
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
/** @brief The ID of the folder where the tracks are stored */
#define MUSIC_FOLDER            1
/** @brief The ID of the folder where the notification sounds are located */
#define NOTIF_FOLDER            2
/** @brief The ID of the first track */
#define FIRST_TRACK             1
/** @brief The ID of the horn sound */
#define HORN_SOUND              1
/** @brief The ID of the notification track */
#define CYCLIC_NOTIF_TRACK      2
/** @brief The ID of the throttle sound */
#define THROTTLE_SOUND          3
/** @brief Timeout of the state waiting for feedback from the DFPlayer. Unit: 15.625 ms */
#define TIMEOUT_WAITING         64
/** @brief The length of the LED error pattern. Unit: 15.625 ms */
#define ERROR_PATTERN_LENGTH    32

enum class InitPhase {
    PHASE_1_WAIT_FOR_ONLINE_FDBCK,
    PHASE_2_WAIT_FOR_VOLUME_FDBCK,
    PHASE_3_WAIT_FOR_TRACKNO_FDBCK,
    PHASE_4_READY_TO_PLAY
};

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
void InitStateMachine(void);
void PlayLedErrorPattern(void);
void LedHandler();

/** @brief The serial port used for communication with the DFPlayer */
Serial serial;
/** @brief Instance of the DFPlayer class */
DFPlayer dfplayer;
/** @brief The system counter is increased by a hardware time and used to trigger certain events */
uint16_t systemCounter = 0;
/** @brief Number of tracks on the SD card */
uint8_t numberOfTracks = 0;
/** @brief Current track */
uint8_t currentTrack = 1;
/** @brief Initialization status */
InitPhase initStatus = InitPhase::PHASE_1_WAIT_FOR_ONLINE_FDBCK;
/** @brief This flag is indicates if the LED error pattern is being played or not */
bool indicatingError = false;
/** @brief Number of the remaining loops of the LED error pattern play */
uint8_t errorIndLoops = 0;

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

    if(initStatus == InitPhase::PHASE_4_READY_TO_PLAY) {
        if(dfplayer.checkResponse()) {
            uint8_t response;
            response = dfplayer.readRespCommand();

            switch(response)
            {
            case DFPL_CMD_RESP_FINISHED_USB:
            case DFPL_CMD_RESP_FINISHED_SD:
                dfplayer.setPlayingStatus(DFPL_STATUS_PAUSED);
                break;
            case DFPL_CMD_RESP_ERROR:
                PlayLedErrorPattern();
                GPIO_setOutputLowOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
                GPIO_setOutputHighOnPin(GPIO_PORT_LED2, GPIO_PIN_LED2);
            default:
                break;
            }
        }
    } else {
        InitStateMachine();
    }

    LedHandler();

    // Reset the counter every 60 sec
    if(systemCounter == (64 * 60)) {
        systemCounter = 0;
        // Warning message: the toy car is left turned on
        if(dfplayer.getPlayingStatus() == DFPL_STATUS_PAUSED) {
            dfplayer.playTrackInFolder(NOTIF_FOLDER, CYCLIC_NOTIF_TRACK);
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
        if(initStatus == InitPhase::PHASE_4_READY_TO_PLAY) {
            currentTrack--;
            // Check if reached the first track
            if(currentTrack < FIRST_TRACK) {
                currentTrack = numberOfTracks;
            }
            dfplayer.playTrackInFolder(MUSIC_FOLDER, currentTrack);
        } else {
            PlayLedErrorPattern();
        }
        GPIO_clearInterrupt(BUTTON_LEFT_PORT, BUTTON_LEFT_PIN);
    }
}

void ButtonRightHandler(void)
{
    if (GPIO_getInterruptStatus (BUTTON_RIGHT_PORT, BUTTON_RIGHT_PIN)) {
        if(initStatus == InitPhase::PHASE_4_READY_TO_PLAY) {
            currentTrack++;
            // Check if reached the last track
            if(currentTrack > numberOfTracks) {
                currentTrack = FIRST_TRACK;
            }
            dfplayer.playTrackInFolder(MUSIC_FOLDER, currentTrack);
        } else {
            PlayLedErrorPattern();
        }
        GPIO_clearInterrupt(BUTTON_RIGHT_PORT, BUTTON_RIGHT_PIN);
    }
}

void ButtonMusicHandler(void)
{
    if (GPIO_getInterruptStatus (BUTTON_MUSIC_PORT, BUTTON_MUSIC_PIN)) {
        if(initStatus == InitPhase::PHASE_4_READY_TO_PLAY) {
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
        } else {
            PlayLedErrorPattern();
        }
        GPIO_clearInterrupt(BUTTON_MUSIC_PORT, BUTTON_MUSIC_PIN);
    }
}

void ButtonHornHandler(void)
{
    if (GPIO_getInterruptStatus (BUTTON_HORN_PORT, BUTTON_HORN_PIN)) {
        if(initStatus == InitPhase::PHASE_4_READY_TO_PLAY) {
            if(dfplayer.getPlayingStatus() == DFPL_STATUS_PLAYING) {
                dfplayer.playAdvertisment(HORN_SOUND);
            } else {
                // If the DFPlayer is not playing a track, it's not possible to insert an advertisement
                // The same sound can be played in this case from the notification folder
                dfplayer.playTrackInFolder(NOTIF_FOLDER, HORN_SOUND);
            }
        } else {
            PlayLedErrorPattern();
        }
        GPIO_clearInterrupt(BUTTON_HORN_PORT, BUTTON_HORN_PIN);
    }
}

void GasPedalHandler(void)
{
    if (GPIO_getInterruptStatus (GAS_PEDAL_PORT, GAS_PEDAL_PIN)) {
        if(initStatus == InitPhase::PHASE_4_READY_TO_PLAY) {
            if(dfplayer.getPlayingStatus() == DFPL_STATUS_PLAYING) {
                dfplayer.playAdvertisment(THROTTLE_SOUND);
            } else {
                // If the DFPlayer is not playing a track, it's not possible to insert an advertisement
                // The same sound can be played in this case from the notification folder
                dfplayer.playTrackInFolder(NOTIF_FOLDER, THROTTLE_SOUND);
            }
        } else {
            PlayLedErrorPattern();
        }
        GPIO_clearInterrupt(GAS_PEDAL_PORT, GAS_PEDAL_PIN);
    }
}

void DataReceivedHandler(void) {}

void InitStateMachine(void)
{
    static uint16_t queryTime = 0;

    switch(initStatus)
    {
    case InitPhase::PHASE_1_WAIT_FOR_ONLINE_FDBCK:
        if(dfplayer.checkResponse() && (dfplayer.readRespCommand() == DFPL_CMD_RESP_ONLINE)) {
            initStatus = InitPhase::PHASE_2_WAIT_FOR_VOLUME_FDBCK;
            dfplayer.setVol(DEFAULT_VOLUME);
            queryTime = systemCounter;
        } else {
            // No response or not the one we are waiting for
            if(systemCounter >= TIMEOUT_WAITING) {
                // Ping the DFPlayer
                dfplayer.queryIsOnline();
            } else {
                // Keep waiting
            }
        }
        break;
    case InitPhase::PHASE_2_WAIT_FOR_VOLUME_FDBCK:
        if(dfplayer.checkResponse() && (dfplayer.readRespCommand() == DFPL_CMD_RESP_FEEDBACK)) {
            initStatus = InitPhase::PHASE_3_WAIT_FOR_TRACKNO_FDBCK;
            dfplayer.queryNoOfTracks();
            queryTime = systemCounter;
        } else {
            // Trying again after 1 second
            if((systemCounter - queryTime) > TIMEOUT_WAITING) {
                dfplayer.setVol(DEFAULT_VOLUME);
                queryTime = systemCounter;
            } else {
                // Keep waiting
            }
        }
        break;
    case InitPhase::PHASE_3_WAIT_FOR_TRACKNO_FDBCK:
        if(dfplayer.checkResponse() && (dfplayer.readRespCommand() == DFPL_CMD_Q_NO_OF_TRACKS)) {
            initStatus = InitPhase::PHASE_4_READY_TO_PLAY;
            numberOfTracks = dfplayer.readRespData2();
        } else {
            // Trying again after 1 second
            if((systemCounter - queryTime) > TIMEOUT_WAITING) {
                dfplayer.queryNoOfTracks();
                queryTime = systemCounter;
            } else {
                // Keep waiting
            }
        }
        break;
    default:
        break;
    }
}

void PlayLedErrorPattern(void)
{
    indicatingError = true;
    errorIndLoops = ERROR_PATTERN_LENGTH;
}

void LedHandler()
{
    if(indicatingError == false) {
        if(initStatus == InitPhase::PHASE_4_READY_TO_PLAY) {
            // LED1 follows the playing status
            if(dfplayer.getPlayingStatus() == DFPL_STATUS_PLAYING) {
                GPIO_setOutputHighOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
            } else {
                GPIO_setOutputLowOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
            }
            // LED2 shows the initialization status
            GPIO_setOutputHighOnPin(GPIO_PORT_LED2, GPIO_PIN_LED2);
        } else {
            // Toggle LED2 while being initialized
            if((systemCounter % 8) == 0) {
                GPIO_toggleOutputOnPin(GPIO_PORT_LED2, GPIO_PIN_LED2);
            }
        }
    } else {
        // Play the error indication pattern
        if(errorIndLoops > 0) {
            if((systemCounter % 4) == 0) {
                GPIO_toggleOutputOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
                GPIO_toggleOutputOnPin(GPIO_PORT_LED2, GPIO_PIN_LED2);
            }
            errorIndLoops--;
        } else {
            // The error indication is finished
            indicatingError = false;
        }
    }
}
