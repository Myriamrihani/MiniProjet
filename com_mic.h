/*
 * com_mic.h
 *
 *  Created on: 14 Apr 2021
 *      Author: myriamrihani
 */

#ifndef COM_MIC_H_
#define COM_MIC_H_


void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size);

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, float* data, uint16_t size);


#endif /* COM_MIC_H_ */
