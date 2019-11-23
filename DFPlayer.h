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

#define DFPL_START_BYTE     0x7E
#define DFPL_VERSION        0xFF
#define DFPL_LEN            0x06
#define DFPL_FEEDBACK_ON    0x01
#define DFPL_FEEDBACK_OFF   0x00
#define DFPL_NO_DATA        0x00
#define DFPL_END_BYTE       0xEF

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
    DFPL_CMD_REPEAT
};
#define GET_HIGH_BYTE(word) (uint8_t(((word) & 0xFF00) >> 8))
#define GET_LOW_BYTE(word) (uint8_t((word) & 0x00FF))
#define DFPL_CMD_CS_NO_FEEDBACK_NO_DATA(cmd) (0x10000 - (DFPL_VERSION + DFPL_LEN + (cmd)))

typedef enum {
    DFPL_STATUS_PLAYING,
    DFPL_STATUS_PAUSED
} PlayingStatus_t;

class DFPlayer
{
private:
    Serial * serial;
    PlayingStatus_t PlayingStatus = DFPL_STATUS_PAUSED;
    uint16_t calculateCheckSum(char* buf);
    void sendMsg(char* msg);
public:
    DFPlayer();
    void setSerial(Serial &s);
    void setPlayingStatus(PlayingStatus_t p);
    PlayingStatus_t getPlayingStatus(void);

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

};

#endif /* DFPLAYER_H_ */
