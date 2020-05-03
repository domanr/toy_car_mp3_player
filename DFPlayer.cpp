/*
 * DFPlayer.cpp
 *
 *  Created on: Nov 23, 2019
 *      Author: doman
 */

#include <DFPlayer.h>

DFPlayer::DFPlayer() :
        PlayingStatus(DFPL_STATUS_PAUSED), lastSentCmd(DFPL_CMD_NO_CMD),
        respReadPos(0u), responseBuffer{}
{
}

uint16_t DFPlayer::calculateCheckSum(char* buf)
{
    uint16_t sum = 0;
    for(uint8_t i = 1; i < DFPL_CS_CALC_LEN; i++)
    {
        sum += buf[i];
    }
    return -sum;
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
    lastSentCmd = msg[DFPL_POS_CMD];
    for(i = 0; i < DFPL_MSG_LEN; i++)
    {
        serial->send(msg[i]);
    }
}

void DFPlayer::next(void)
{
    char message[DFPL_MSG_LEN] = {
                                  DFPL_START_BYTE,
                                  DFPL_VERSION,
                                  DFPL_LEN, DFPL_CMD_NEXT,
                                  DFPL_FEEDBACK_ON,
                                  DFPL_NO_DATA,
                                  DFPL_NO_DATA,
                                  GET_HIGH_BYTE(DFPL_CMD_CS_NO_DATA(DFPL_CMD_NEXT)),
                                  GET_LOW_BYTE(DFPL_CMD_CS_NO_DATA(DFPL_CMD_NEXT)),
                                  DFPL_END_BYTE
    };
    sendMsg(message);
    setPlayingStatus(DFPL_STATUS_PLAYING);
}

void DFPlayer::previous(void)
{
    char message[DFPL_MSG_LEN] = {
                                  DFPL_START_BYTE,
                                  DFPL_VERSION,
                                  DFPL_LEN, DFPL_CMD_PREV,
                                  DFPL_FEEDBACK_ON,
                                  DFPL_NO_DATA,
                                  DFPL_NO_DATA,
                                  GET_HIGH_BYTE(DFPL_CMD_CS_NO_DATA(DFPL_CMD_PREV)),
                                  GET_LOW_BYTE(DFPL_CMD_CS_NO_DATA(DFPL_CMD_PREV)),
                                  DFPL_END_BYTE
    };
    sendMsg(message);
    setPlayingStatus(DFPL_STATUS_PLAYING);
}

void DFPlayer::increaseVol(void)
{
    char message[DFPL_MSG_LEN] = {
                                  DFPL_START_BYTE,
                                  DFPL_VERSION,
                                  DFPL_LEN, DFPL_CMD_INC_VOL,
                                  DFPL_FEEDBACK_OFF,
                                  DFPL_NO_DATA,
                                  DFPL_NO_DATA,
                                  GET_HIGH_BYTE(DFPL_CMD_CS_NO_FEEDBACK_NO_DATA(DFPL_CMD_INC_VOL)),
                                  GET_LOW_BYTE(DFPL_CMD_CS_NO_FEEDBACK_NO_DATA(DFPL_CMD_INC_VOL)),
                                  DFPL_END_BYTE
    };
    sendMsg(message);
}

void DFPlayer::decreaseVol(void)
{
    char message[DFPL_MSG_LEN] = {
                                  DFPL_START_BYTE,
                                  DFPL_VERSION,
                                  DFPL_LEN, DFPL_CMD_DEC_VOL,
                                  DFPL_FEEDBACK_OFF,
                                  DFPL_NO_DATA,
                                  DFPL_NO_DATA,
                                  GET_HIGH_BYTE(DFPL_CMD_CS_NO_FEEDBACK_NO_DATA(DFPL_CMD_DEC_VOL)),
                                  GET_LOW_BYTE(DFPL_CMD_CS_NO_FEEDBACK_NO_DATA(DFPL_CMD_DEC_VOL)),
                                  DFPL_END_BYTE
    };
    sendMsg(message);
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
    setPlayingStatus(DFPL_STATUS_PLAYING);
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
    setPlayingStatus(DFPL_STATUS_PAUSED);
}

void DFPlayer::setVol(uint8_t vol)
{
    char message[DFPL_MSG_LEN] = {
            DFPL_START_BYTE,
            DFPL_VERSION,
            DFPL_LEN,
            DFPL_CMD_SET_VOL,
            DFPL_FEEDBACK_ON,
            DFPL_NO_DATA,
            0,
            0,
            0,
            DFPL_END_BYTE
    };
    uint16_t cs;

    message[DFPL_POS_DATA2] = vol;
    cs = calculateCheckSum(message);
    message[DFPL_POS_CS_HIGH] = GET_HIGH_BYTE(cs);
    message[DFPL_POS_CS_LOW] = GET_LOW_BYTE(cs);
    sendMsg(message);
}

void DFPlayer::playAdvertisment(uint8_t advNo)
{
    char message[DFPL_MSG_LEN] = {
            DFPL_START_BYTE,
            DFPL_VERSION,
            DFPL_LEN, DFPL_CMD_PLAY_ADV,
            DFPL_FEEDBACK_OFF,
            DFPL_NO_DATA,
            advNo,
            GET_HIGH_BYTE(DFPL_CMD_CS_NO_FEEDBACK(DFPL_CMD_PLAY_ADV, advNo)),
            GET_LOW_BYTE(DFPL_CMD_CS_NO_FEEDBACK(DFPL_CMD_PLAY_ADV, advNo)),
            DFPL_END_BYTE };
    sendMsg(message);
}

uint8_t DFPlayer::readRespCommand()
{
    return responseBuffer[DFPL_POS_CMD];
}

uint8_t DFPlayer::getLastSentCmd() const
{
    return lastSentCmd;
}

void DFPlayer::startup(void)
{
    char message[DFPL_MSG_LEN] = {
            DFPL_START_BYTE,
            DFPL_VERSION,
            DFPL_LEN, DFPL_CMD_PLAY_ADV,
            DFPL_FEEDBACK_OFF,
            DFPL_NO_DATA,
            0x02,
            GET_HIGH_BYTE(DFPL_CMD_CS_NO_FEEDBACK(DFPL_CMD_PLAY_ADV, 0x02)),
            GET_LOW_BYTE(DFPL_CMD_CS_NO_FEEDBACK(DFPL_CMD_PLAY_ADV, 0x02)),
            DFPL_END_BYTE };
    sendMsg(message);
}

bool DFPlayer::checkResponse()
{
    while((serial->GetBufferLength() > 0u) && (respReadPos < DFPL_MSG_LEN))
    {
        serial->SerialPortRead(responseBuffer + respReadPos);
        if(((DFPL_POS_START == respReadPos) && (DFPL_START_BYTE == responseBuffer[respReadPos])) || (DFPL_POS_START < respReadPos)) {
            respReadPos++;
        }
    }
    if(DFPL_MSG_LEN == respReadPos) {
        respReadPos = 0u;   // A complete message was received, reset the response position pointer
        return true;
    } else {
        return false;       // Message reception was not completed, will be continued in the next loop
    }
}
