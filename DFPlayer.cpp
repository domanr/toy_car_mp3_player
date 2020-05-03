/*
 * DFPlayer.cpp
 *
 *  Created on: Nov 23, 2019
 *      Author: doman
 */

#include <DFPlayer.h>

DFPlayer::DFPlayer() :
        PlayingStatus(DFPL_STATUS_PAUSED), lastSentCmd(DFPL_CMD_NO_CMD),
        respReadPos(0u), responseBuffer{},
        requestBuffer{
            DFPL_START_BYTE,
            DFPL_VERSION,
            DFPL_LEN,
            DFPL_CMD_NO_CMD,
            DFPL_FEEDBACK_OFF,
            DFPL_NO_DATA,
            DFPL_NO_DATA,
            0u,
            0u,
            DFPL_END_BYTE
        }
{
}

void DFPlayer::calculateCheckSum()
{
    uint16_t sum = 0;
    for(uint8_t i = 1; i < DFPL_CS_CALC_LEN; i++)
    {
        sum += requestBuffer[i];
    }
    sum = (-sum);
    requestBuffer[DFPL_POS_CS_HIGH] = getHighByte(sum);
    requestBuffer[DFPL_POS_CS_LOW] = getLowByte(sum);
}

uint8_t DFPlayer::getHighByte(uint16_t word)
{
    return (uint8_t)((word & 0xFF00) >> 8);
}

uint8_t DFPlayer::getLowByte(uint16_t word)
{
    return (uint8_t)(word & 0x00FF);
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

void DFPlayer::sendMsg(uint8_t requestFeedback)
{
    int i = 0;
    requestBuffer[DFPL_POS_FEEDBACK] = requestFeedback;
    calculateCheckSum();
    lastSentCmd = requestBuffer[DFPL_POS_CMD];
    for(i = 0; i < DFPL_MSG_LEN; i++)
    {
        serial->send(requestBuffer[i]);
    }
}

void DFPlayer::next(void)
{
    setCommand(DFPL_CMD_NEXT);
    setData1(DFPL_NO_DATA);
    setData2(DFPL_NO_DATA);
    sendMsg(DFPL_FEEDBACK_ON);
    setPlayingStatus(DFPL_STATUS_PLAYING);
}

void DFPlayer::previous(void)
{
    setCommand(DFPL_CMD_PREV);
    setData1(DFPL_NO_DATA);
    setData2(DFPL_NO_DATA);
    sendMsg(DFPL_FEEDBACK_ON);
    setPlayingStatus(DFPL_STATUS_PLAYING);
}

void DFPlayer::increaseVol(void)
{
    setCommand(DFPL_CMD_INC_VOL);
    setData1(DFPL_NO_DATA);
    setData2(DFPL_NO_DATA);
    sendMsg(DFPL_FEEDBACK_OFF);
}

void DFPlayer::decreaseVol(void)
{
    setCommand(DFPL_CMD_DEC_VOL);
    setData1(DFPL_NO_DATA);
    setData2(DFPL_NO_DATA);
    sendMsg(DFPL_FEEDBACK_OFF);
}

void DFPlayer::play(void)
{
    setCommand(DFPL_CMD_PLAY);
    setData1(DFPL_NO_DATA);
    setData2(DFPL_NO_DATA);
    sendMsg(DFPL_FEEDBACK_OFF);
    setPlayingStatus(DFPL_STATUS_PLAYING);
}

void DFPlayer::pause(void)
{
    setCommand(DFPL_CMD_PAUSE);
    setData1(DFPL_NO_DATA);
    setData2(DFPL_NO_DATA);
    sendMsg(DFPL_FEEDBACK_OFF);
    setPlayingStatus(DFPL_STATUS_PAUSED);
}

void DFPlayer::setVol(uint8_t vol)
{
    setCommand(DFPL_CMD_SET_VOL);
    setData1(DFPL_NO_DATA);
    setData2(vol);
    calculateCheckSum();
    sendMsg(DFPL_FEEDBACK_ON);
}

void DFPlayer::playAdvertisment(uint8_t advNo)
{
    setCommand(DFPL_CMD_PLAY_ADV);
    setData1(DFPL_NO_DATA);
    setData2(advNo);
    calculateCheckSum();
    sendMsg(DFPL_FEEDBACK_OFF);
}

uint8_t DFPlayer::readRespCommand()
{
    return responseBuffer[DFPL_POS_CMD];
}

void DFPlayer::setData(uint16_t data)
{
    requestBuffer[DFPL_POS_DATA1] = getHighByte(data);
    requestBuffer[DFPL_POS_DATA2] = getLowByte(data);
}

void DFPlayer::setData1(uint8_t data)
{
    requestBuffer[DFPL_POS_DATA1] = data;
}

void DFPlayer::setData2(uint8_t data)
{
    requestBuffer[DFPL_POS_DATA2] = data;
}

void DFPlayer::setCommand(uint8_t cmd)
{
    requestBuffer[DFPL_POS_CMD] = cmd;
}

uint8_t DFPlayer::getLastSentCmd() const
{
    return lastSentCmd;
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
