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

#ifndef __BTApp_H__
#define __BTApp_H__

#include "GenericTypeDefs.h"
#include "bt_common.h"

/*
 * Bluetooth application structure definitions
 */

/* Physical bus generic call-back interface */
typedef struct _PHY_BUS
{
    BYTE* (*getACLBuff)(void);
    BYTE* (*getEVTBuff)(void);

    INT32 (*writeACL)(const BYTE*, UINT);
    INT32 (*writeCTL)(const BYTE*, UINT);

    bool (*readACL)(const BYTE*, UINT);
    bool (*readEVT)(const BYTE*, UINT);
} PHY_BUS;

/* Device call-back interface */
typedef struct _BT_DEVICE
{
    /* PHY_BUS API */
    PHY_BUS sUSB;
    /* L2CAP_API */
    bool (*L2CAPdisconnect)(UINT16);
    /* RFCOMM API */
    bool (*SPPsendData)(const BYTE*, UINT);
    bool (*SPPdisconnect)(UINT8);
} BT_DEVICE;

/*
 * Bluetooth application public function prototypes
 */

bool BTAPP_Initialise(BT_DEVICE **ppsBTDevice);
bool BTAPP_Start(BT_DEVICE *psBTDevice);
bool BTAPP_Deinitialise();
bool BTAPP_API_putRFCOMMData(const BYTE *pData, UINT uLen);
bool BTAPP_API_confComplete();


#endif /*BTApp*/
