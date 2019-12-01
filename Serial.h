/*
 * Serial.h
 *
 *  Created on: Nov 3, 2019
 *      Author: DomanR
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "stdint.h"

#define NULL 0
#define BUFFER_SIZE 32

typedef enum
{
    eErrorBufferFull    = 0,
    eWriteOK            = 1
}CbWriteReturnType;

typedef enum
{
    eErrorBufferEmpty   = 0,
    eReadOK             = 1
}CbReadReturnType;

class Serial
{
private:
    static uint8_t SerialBuffer[BUFFER_SIZE];
    static uint8_t ReadPosition;
    static uint8_t WritePosition;
    static void (*UpperLayerCallout)(void);
public:
    static char string[10];
    static unsigned int i;
    Serial(void);
    void init(void (*pULNotificationCallout)(void));
    void puts(char*);
    void send(char);
    CbReadReturnType SerialPortRead(uint8_t* data);
    static uint8_t GetBufferLength(void);
    static __interrupt void EUSCI_A0_ISR(void);
};

#endif /* SERIAL_H_ */
