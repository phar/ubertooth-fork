
/*
 * Copyright 2010 Michael Ossmann
 *
 * This file is part of Project Ubertooth.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "ubertooth.h"
#include "usb_serial.h"
#include "BTApp.h"




static int vendor_request_handler(u8 request, u16 *request_params, u8 *data, int *data_len)
{
        u32 command[5];
        u32 result[5];


        switch (request) {
        case UBERTOOTH_SET_ISP:
                command[0] = 57;
                iap_entry(command, result);
                *data_len = 0; /* should never return */
                break;

        case UBERTOOTH_FLASH:
                bootloader_ctrl = DFU_MODE;
                reset();
                break;
	}

}


/*
#define STATE_READY    0
#define STATE_OPCODE	1
#define STATE_LENGTH	2
#define STATE_PARAMS	3



#define HCI_MAX_PARAMETER_LENGTH	255

typedef struct hci_ctx_t{
	u16	state;
	u16	hci_opcode;
	u8	opc;
	u8	hci_length;
	u8	hci_parameter[HCI_MAX_PARAMETER_LENGTH];	
	u8	opp;

}hci_ctx_t;


void hci_init(hci_ctx_t *hci){
	memset(&hci,0,sizeof(hci_ctx_t));
	&hci->state = STATE_READY;
}


u16 hci_update(&hci_ctx_t *hci, u8 c){
int i;
int nparams,p;

	switch(hci->state){
		case  STATE_READY:
			hci->state = STATE_OPCODE;
			//set opcode[i++]

			if(hci->opc == 0){
				hci->opc++
			}else{
				state = STATE_LENGTH;
			}
			break;

		case STATE_LENGTH:
			hci->hci_length = c;
			hci->opp = 0;
			hci->state = STATE_PARAMS;
			break;

		case STATE_PARAMS:
			if(p==nparams){
				hci->state = STATE_HANDLE;
				hci_handler(hci_opcode,hci_length,hci_parameters);
				hci_init(hci);
			}else{
				hci->hci_parameters[hci->opp] = c;
				hci->opp++;
			}
			break;

		default:
			hci_init(hci);
	}
	return hci->state;
}


*/


BT_DEVICE *gpsBTAPP = NULL;

int main(){
	int c;
//hci_ctx_t	hci;



	ubertooth_init();
//	hci_init(&hci);
	usb_ioa_init(vendor_request_handler);

    BTAPP_Initialise(&gpsBTAPP);

	/*
	 * for each character received over USB serial connection, echo the
	 * character back over USB serial and toggle USRLED
	 */
	while (1) {
		c = VCOM_getchar();
		if (c != EOF) {
			/* toggle USRLED */
			if (USRLED)
				USRLED_CLR;
			else
				USRLED_SET;
//			hci_update(&hci, c);
//			VCOM_putchar(c);
		}
	}
}
