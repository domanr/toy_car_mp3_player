/*
 * DFPlayer.h
 *
 *  Created on: Nov 23, 2019
 *      Author: doman
 */

#ifndef DFPLAYER_H_
#define DFPLAYER_H_

#include "Serial.h"
#include "stdint.h"

#define DFPL_MSG_LEN        10
#define DFPL_CS_CALC_LEN    7

#define DFPL_START_BYTE     0x7E
#define DFPL_VERSION        0xFF
#define DFPL_LEN            0x06
#define DFPL_FEEDBACK_ON    0x01
#define DFPL_FEEDBACK_OFF   0x00
#define DFPL_NO_DATA        0x00
#define DFPL_END_BYTE       0xEF

#define DFPL_CMD_RESP_ONLINE    0x3F

enum {
    DFPL_POS_START = 0,
    DFPL_POS_VESRION,
    DFPL_POS_LEN,
    DFPL_POS_CMD,
    DFPL_POS_FEEDBACK,
    DFPL_POS_DATA1,
    DFPL_POS_DATA2,
    DFPL_POS_CS_HIGH,
    DFPL_POS_CS_LOW,
    DFPL_POS_END
};

enum {
    DFPL_CMD_NEXT = 1u,
    DFPL_CMD_PREV,
    DFPL_CMD_TRACK_NUM,
    DFPL_CMD_INC_VOL,
    DFPL_CMD_DEC_VOL,
    DFPL_CMD_SET_VOL,
    DFPL_CMD_SET_EQ,
    DFPL_CMD_PLAY_MODE,
    DFPL_CMD_PLAY_SRC,
    DFPL_CMD_STANDBY,
    DFPL_CMD_NORMAL,
    DFPL_CMD_RESET,
    DFPL_CMD_PLAY,
    DFPL_CMD_PAUSE,
    DFPL_CMD_FOLDER,
    DFPL_CMD_VOL_ADJ_SET,
    DFPL_CMD_REPEAT,
    DFPL_CMD_PLAY_FOLDER_TRACK,
    DFPL_CMD_PLAY_ADV,
    DFPL_CMD_PLAY_FOLDER_TRACK_16,
    DFPL_CMD_STOP_ADV
};
#define GET_HIGH_BYTE(word) (uint8_t(((word) & 0xFF00) >> 8))
#define GET_LOW_BYTE(word) (uint8_t((word) & 0x00FF))
#define DFPL_CMD_CS_NO_FEEDBACK_NO_DATA(cmd) (0x10000 - (DFPL_VERSION + DFPL_LEN + (cmd)))
#define DFPL_CMD_CS_NO_FEEDBACK(cmd, data) (0x10000 - (DFPL_VERSION + DFPL_LEN + (data) + (cmd)))

typedef enum {
    DFPL_STATUS_PLAYING,
    DFPL_STATUS_PAUSED
} PlayingStatus_t;

typedef enum {
    DFPL_UNINITIALIZED,
    DFPL_INITIALIZED
} Init_t;

typedef enum {
    DFPL_RESPONSE_INITIATED,
    DFPL_RESPONSE_PARSING,
    DFPL_RESPONSE_WAITING
} ResponseStatus_t;

class DFPlayer
{
private:
    Serial * serial;
    PlayingStatus_t PlayingStatus = DFPL_STATUS_PAUSED;
    Init_t initStatus = DFPL_UNINITIALIZED;
    ResponseStatus_t responseStatus = DFPL_RESPONSE_WAITING;
    uint16_t calculateCheckSum(char* buf);
    void sendMsg(char* msg);
    void parseMsg(void);
public:
    DFPlayer();
    void setSerial(Serial &s);
    void setPlayingStatus(PlayingStatus_t p);
    PlayingStatus_t getPlayingStatus(void);
    Init_t getInitStatus() const;
    void setInitStatus(Init_t status);
    ResponseStatus_t getResponseStatus() const;
    void setResponseStatus(ResponseStatus_t status);

    /* Commands */
    void next(void);
    void previous(void);
    void playTrack(uint16_t num);
    void increaseVol(void);
    void decreaseVol(void);
    void setVol(uint8_t vol);
    void setEQ(uint8_t eq);
    void setPlayMode(uint8_t mode);
    void enterStandby(void);
    void normalWorking(void);
    void reset(void);
    void play(void);
    void pause(void);
    void specifyFolder(uint8_t folder);
    void volumeAdjustSet(uint8_t openVolAdj, uint8_t volGain);
    void repeatPlay(uint8_t repeatOnOff);
    void playAdvertisment(void);
    void startup(void);
};

#endif /* DFPLAYER_H_ */
