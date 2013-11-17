
/*
   Copyright 2012 Guillem Vinals Gangolells <guillem@guillem.co.uk>

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */
#include <stdbool.h>
#include <stdlib.h>
#include "GenericTypeDefs.h"

void* BT_malloc(size_t uSize)
{
    return malloc(uSize);
}

void BT_free(void* pData)
{
    free(pData);
}

/*Read (little-endian) 16bits*/
WORD BT_readLE16(const BYTE *pData, UINT uOffset)
{
    if (NULL == pData)
    {
        return false;
    }
    return (pData[uOffset+1]<<8) + pData[uOffset];
}

/*Store (little-endian) 16bits*/
bool BT_storeLE16(WORD value, BYTE *pData, UINT uOffset)
{
    if (NULL == pData)
    {
        return false;
    }
    pData[uOffset+1] = (value&0xff00)>>8;
    pData[uOffset] = value&0x00ff;
    return true;
}

/*Store (little-endian) 32bits*/
bool BT_storeLE32(DWORD value, BYTE *pData, UINT uOffset)
{
    if (NULL == pData)
    {
        return false;
    }
    pData[uOffset+3] = (value&0xff000000)>>24;
    pData[uOffset+2] = (value&0xff0000)>>16;
    pData[uOffset+1] = (value&0xff00)>>8;
    pData[uOffset] = value&0x00ff;
    return true;
}

/*Read (big-endian) 16bits*/
WORD BT_readBE16(const BYTE *pData, UINT uOffset)
{
    if (NULL == pData)
    {
        return false;
    }
    return (pData[uOffset]<<8) + pData[uOffset+1];
}

/*Store (big-endian) 16bits*/
bool BT_storeBE16(WORD value, BYTE *pData, UINT uOffset)
{
    if (NULL == pData)
    {
        return false;
    }
    pData[uOffset] = (value&0xff00)>>8;
    pData[uOffset+1] = value&0x00ff;
    return true;
}

/*Store (big-endian) 32bits*/
bool BT_storeBE32(DWORD value, BYTE *pData, UINT uOffset){
    if (NULL == pData)
    {
        return false;
    }
    pData[uOffset] = (value&0xff000000)>>24;
    pData[uOffset+1] = (value&0xff0000)>>16;
    pData[uOffset+2] = (value&0xff00)>>8;
    pData[uOffset+3] = value&0xff;
    return true;
}

/*Read (big-endian) 32bits*/
DWORD BT_readBE32(const BYTE *pData, UINT uOffset)
{
    if (NULL == pData)
    {
        return false;
    }
    return ((DWORD)pData[uOffset]<<24) + ((DWORD)pData[uOffset+1]<<16) + ((DWORD)pData[uOffset+2]<<8) + pData[uOffset+3];
}

/*Store string*/
bool store_STR(const char *str, int str_size, BYTE *pData, UINT uOffset)
{
    int i;
    if (NULL == pData)
    {
        return false;
    }
    for(i=0; i<str_size; ++i)
    {
            pData[i+uOffset] = str[i];
    }
    return true;
}

bool BT_isEqualBD_ADDR(const BYTE *pBD_ADDR1, const BYTE *pBD_ADDR2)
{
    UINT i;
    if ((NULL == pBD_ADDR1) ||
         NULL == pBD_ADDR2)
    {
        return false;
    }
    for (i = 0; i < 6; ++i)
    {
        if (pBD_ADDR1[i] != pBD_ADDR2[i])
        {
            return false;
        }
    }
    return true;
}
