/*
 * dynamicbuffer.h
 *
 *  Created on: Jun 18, 2023
 *      Author: vagrant
 */

#ifndef INC_DYNAMICBUFFER_H_
#define INC_DYNAMICBUFFER_H_

#include <stdint.h>
#include <stddef.h>

typedef struct DynamicBuffer {

    uint8_t* buff;
    size_t size;
    size_t tx_size;
    size_t pos;

} DynamicBuffer;

void DynamicBuffer_Init( DynamicBuffer* dbuff, size_t size );

void DynamicBuffer_Append( DynamicBuffer* dbuff, uint8_t* data, size_t size );

void DynamicBuffer_RollPosIndex( DynamicBuffer* dbuff, size_t size );

uint8_t * DynamicBuffer_GetBuffAddr( DynamicBuffer* dbuff );

size_t DynamicBuffer_GetBuffSize( DynamicBuffer* dbuff );

size_t DynamicBuffer_GetBuffPos( DynamicBuffer* dbuff );

void DynamicBuffer_SetTxSize( DynamicBuffer* dbuff, size_t size );

size_t DynamicBuffer_GetTxSize( DynamicBuffer* dbuff );

#endif /* INC_DYNAMICBUFFER_H_ */
