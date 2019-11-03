/*
 * Serial.h
 *
 *  Created on: Nov 3, 2019
 *      Author: DomanR
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#define NULL 0

class Serial
{
public:
    static char string[10];
    static unsigned int i;
    Serial(void);
    void init(void);
    void puts(char*);
    static __interrupt void EUSCI_A0_ISR(void);
};

#endif /* SERIAL_H_ */
