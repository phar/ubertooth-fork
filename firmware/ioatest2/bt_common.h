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

#ifndef __BT_COMMON__
#define __BT_COMMON__
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* USB and BTD dongle hardware definitions */
#define DATA_PACKET_LENGTH  680
#define CONTROL_PACKET_LENGTH 32
#define EVENT_PACKET_LENGTH 32
#define MAX_ACL_R_BUFF_SIZE 1
#define MAX_EVT_R_BUFF_SIZE 1

/*Protocol and service multiplexor*/
#define L2CAP_SDP_PSM 0x0001
#define L2CAP_RFCOMM_PSM 0x0003

/* MTU = DATA PACKET LENGTH (680) minus the L2CAP and the HCI headers (4 + 4)*/
#define L2CAP_MTU 672

/* RFCOMM common defines */
#define RFCOMM_NUM_CHANNELS 2
#define RFCOMM_CH_MUX 0x00
#define RFCOMM_CH_DATA 0x01

/* 
 * BT layers APIs:
 * Each layer have it's own API in order to interface with the lower and
 * upper layers.
 * All the APIs defined here can be accessed by the method "X_getAPI()" and
 * will be populated by the method "X_installY".
 */

typedef struct _HCIUSB_API
{
    BYTE* (*getACLBuff)(void);
    BYTE* (*getEVTBuff)(void);

    INT32(*USBwriteACL)(const BYTE*, UINT);
    INT32(*USBwriteCTL)(const BYTE*, UINT);
} HCIUSB_API;

typedef struct _HCI_API
{
    void (*cmdReset)(void);
    bool (*setLocalName)(const CHAR*,UINT);
    bool (*setPINCode)(const CHAR*,UINT);

    bool (*sendData)(const BYTE*, UINT);
    bool (*putData)(const BYTE*, UINT);
    bool (*putEvent)(const BYTE*, UINT);
} HCI_API;

typedef struct _L2CAP_API
{
    bool (*sendData)(UINT16, const BYTE*, UINT16);
    bool (*putData)(const BYTE*, UINT16, bool);
    bool (*disconnect)(UINT16);
} L2CAP_API;

typedef struct _RFCOMM_API
{
    bool (*sendData)(const BYTE*,UINT);
    bool (*putData)(const BYTE*,UINT);
    bool (*disconnect)(UINT8);
} RFCOMM_API;

typedef struct _SDP_API
{
    bool (*sendData)(const BYTE*,UINT);
    bool (*putData)(const BYTE*,UINT);
} SDP_API;

typedef struct _DEVICE_API
{
    bool (*confComplete)(void);
    bool (*putRFCOMMData)(const BYTE *,UINT);
} DEVICE_API;

/*
 * Common Bluetooth function definitions implemented on the respective layer.
 */

bool HCIUSB_create();
bool HCIUSB_destroy();
bool HCIUSB_getAPI(HCIUSB_API *psAPI);

bool HCI_create();
bool HCI_destroy();
bool HCI_installL2CAP(L2CAP_API *psAPI);
bool HCI_installDevCB(DEVICE_API *psAPI);
bool HCI_getAPI(HCI_API *psAPI);

bool L2CAP_create();
bool L2CAP_destroy();
bool L2CAP_getAPI(L2CAP_API *psAPI);
bool L2CAP_installRFCOMM(RFCOMM_API *psAPI);
bool L2CAP_installSDP(SDP_API *psAPI);

bool SDP_create();
bool SDP_destroy();

bool RFCOMM_create();
bool RFCOMM_destroy();
bool RFCOMM_getAPI(RFCOMM_API *psAPI);
bool RFCOMM_installDevCB(DEVICE_API *psAPI);

#endif //_BT_COMMON_
