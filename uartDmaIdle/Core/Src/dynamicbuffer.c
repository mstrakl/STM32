/*
 * dynamicbuffer.c
 *
 *  Created on: Jun 18, 2023
 *      Author: vagrant
 */

#include "dynamicbuffer.h"
#include <string.h>
#include <stdlib.h>


void DynamicBuffer_Init( DynamicBuffer* dbuff, size_t size )
{
    dbuff->size = size;
    dbuff->pos = 0;
    dbuff->tx_size = 0;

    dbuff->buff = (uint8_t *) malloc( dbuff->size*sizeof(uint8_t ) );

}


void DynamicBuffer_Append( DynamicBuffer* dbuff, uint8_t* data, size_t size )
{
    size_t free = dbuff->size - dbuff->pos;

    if ( size <= free ) {

        memcpy( &dbuff->buff[ dbuff->pos ], data, size );
        dbuff->pos += size;

    }



}

void DynamicBuffer_RollPosIndex( DynamicBuffer* dbuff, size_t size )
{
    if ( ! size > 0 ) return;

    if ( size <= dbuff->pos ) {

        memcpy( dbuff->buff, &dbuff->buff[ size ], size );

        dbuff->pos -= size;

    }

    else {

        memcpy( dbuff->buff, &dbuff->buff[ dbuff->pos ], dbuff->pos );

        size = dbuff->pos;

        dbuff->pos = 0;

    }
}



uint8_t * DynamicBuffer_GetBuffAddr( DynamicBuffer* dbuff )
{
    return dbuff->buff;
}


size_t DynamicBuffer_GetBuffSize( DynamicBuffer* dbuff )
{
    return dbuff->size;
}


size_t DynamicBuffer_GetBuffPos( DynamicBuffer* dbuff )
{
    return dbuff->pos;
}

void DynamicBuffer_SetTxSize( DynamicBuffer* dbuff, size_t size )
{
    dbuff->tx_size = size;
}

size_t DynamicBuffer_GetTxSize( DynamicBuffer* dbuff )
{
    return dbuff->tx_size;
}















