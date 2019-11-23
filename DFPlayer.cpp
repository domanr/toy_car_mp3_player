/*
 * DFPlayer.cpp
 *
 *  Created on: Nov 23, 2019
 *      Author: doman
 */

#include <DFPlayer.h>

DFPlayer::DFPlayer()
{
    // TODO Auto-generated constructor stub

}

uint16_t DFPlayer::calculateCheckSum(char* buf)
{
    return 0x0000;
}

void DFPlayer::setSerial(Serial &s)
{
    serial = &s;
}

void DFPlayer::setPlayingStatus(PlayingStatus_t p)
{
    PlayingStatus = p;
}

PlayingStatus_t DFPlayer::getPlayingStatus()
{
    return PlayingStatus;
}

void DFPlayer::sendMsg(char* msg)
{
    int i = 0;
    for(i = 0; i < DFPL_MSG_LEN; i++)
    {
        serial->send(msg[i]);
    }
}

void DFPlayer::next(void)
{
    //char buf[]
}

void DFPlayer::previous(void)
{
}

void DFPlayer::increaseVol(void)
{
}

void DFPlayer::decreaseVol(void)
{
}

void DFPlayer::play(void)
{
    char message[DFPL_MSG_LEN] = {
            DFPL_START_BYTE,
            DFPL_VERSION,
            DFPL_LEN, DFPL_CMD_PLAY,
            DFPL_FEEDBACK_OFF,
            DFPL_NO_DATA,
            DFPL_NO_DATA,
            GET_HIGH_BYTE(DFPL_CMD_CS_NO_FEEDBACK_NO_DATA(DFPL_CMD_PLAY)),
            GET_LOW_BYTE(DFPL_CMD_CS_NO_FEEDBACK_NO_DATA(DFPL_CMD_PLAY)),
            DFPL_END_BYTE
    };
    sendMsg(message);
}

void DFPlayer::pause(void)
{
    char message[DFPL_MSG_LEN] = {
            DFPL_START_BYTE,
            DFPL_VERSION,
            DFPL_LEN, DFPL_CMD_PAUSE,
            DFPL_FEEDBACK_OFF,
            DFPL_NO_DATA,
            DFPL_NO_DATA,
            GET_HIGH_BYTE(DFPL_CMD_CS_NO_FEEDBACK_NO_DATA(DFPL_CMD_PAUSE)),
            GET_LOW_BYTE(DFPL_CMD_CS_NO_FEEDBACK_NO_DATA(DFPL_CMD_PAUSE)),
            DFPL_END_BYTE
    };
    sendMsg(message);
}
