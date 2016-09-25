/* 

use at your own risk.

bt app
taken from the original espressif repo on
http://github.com/espressif

in the meantime until we have a full BT stack and functions 
we do Ride on the BT Identifier ;-)
 
for spread Messages on the BT Identifier in this example.
 
changes to a fun factor "News-spread-via-BLE-ID-riding"
can be a base for flexible setup the BLE ID with buttons or UART and so on.
simple show here: you can change the BLE ID again after the BLE is run.


(y: year; m:month; d:day; u:user)

y: 2016
m: 09
d: 24
u: rudi ;-)

base: 
esp-idf
 


play with the BLE 
this is a fun test  example with scan on a Windows 10 Tab named "WINTRON"

what was done:

added simple small pointers for the const ble id to change it
for a funny messaging by the BLE ID in the rhytmen of BLE setup
because i change few lines for BLE it does not go next steps until you change
the line back:

look at 
static void hci_cmd_send_ble_set_adv_data(void);

here was BLE-ID added and have a IDX on each run in the task

void bleAdvtTask(void *pvParameters);

changes was :

add the increment for message idx ( idx++; )

	case 2: hci_cmd_send_ble_set_adv_data(); ++cmd_cnt; idx++; break; 
 
add the restore ( cmd_cnt = 0 ) for the settup procedure that do not go next step
	
	case 3: hci_cmd_send_ble_adv_start(); ++cmd_cnt; cmd_cnt = 0; break;

	
what does it do:
after start, the BLE Setup proc is run normal,
which each new setup the BLE IDE is changed,
this happens by set the pointer address of BLE ID to a new with new Text ( 20 BYTE )
this then is the new "Message" in the BLE IDE
that you see in the other BLE device , example here in win 10 in the scan doing.

what you can do with this:
you can setup example buttons for your BLE network
on each other click buttons, the click procedure calls your set BLE-ID name
example:

	// the flexible STRING var for the Button, UART, or other doings 
	// char MyMsg[]		= "NewsSpreadViaBLEride"; 

// the pointer for CONST STRING and the STRING VAR for later swap 
// char *p = (char*)&MyMsg[0]; 


see the userbutton routine.

what you must do:

add your Button, or ISR, or UART or what ever you want use to change the BLE Identifier.


have phun
best wishes
rudi ;-)

http://esp32.de
http://twitter.com/eMbeddedHome
mail: 

rudi@eMbeddedHome.de


*/





#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bt.h"
#include <string.h>

#define HCI_H4_CMD_PREAMBLE_SIZE           (4)

/*  HCI Command opcode group field(OGF) */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS    (0x03 << 10)            /* 0x0C00 */
#define HCI_GRP_BLE_CMDS                   (0x08 << 10)

#define HCI_RESET                          (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_BLE_WRITE_ADV_ENABLE           (0x000A | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_PARAMS           (0x0006 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_DATA             (0x0008 | HCI_GRP_BLE_CMDS)

#define HCIC_PARAM_SIZE_WRITE_ADV_ENABLE        (1)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS    (15)
#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA      (31)

#define BD_ADDR_LEN     (6)                     /* Device address length */
typedef uint8_t bd_addr_t[BD_ADDR_LEN];         /* Device address */

#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define BDADDR_TO_STREAM(p, a)   {int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (uint8_t) a[BD_ADDR_LEN - 1 - ijk];}
#define ARRAY_TO_STREAM(p, a, len) {int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (uint8_t) a[ijk];}

enum {
    H4_TYPE_COMMAND = 1,
    H4_TYPE_ACL     = 2,
    H4_TYPE_SCO     = 3,
    H4_TYPE_EVENT   = 4
};



// BLE  Messaging VARS
/*
  short info:
  the BLE ID is shown in the Head of seraching devices ( scan devices )
  usually is this done as const string const.
  we set up a few messageblock vars as stringconstants
  for show the messageblock step by step
  this is only for show as fun factor..
  you can use it example for setup the ble id by buttons, uart or others
  
*/

// time interval for setup messaging / ticker
// be sure your BLE scanner support fast updates in scan
// let it do all 1000 ms as standard in demo and then you can test faster
uint16_t tickering = 1000;	// all 1000 ms 


// all Text must be the same length, 
// max 20 Byte 

char *adv_name  	= "ESP-BLE-HELLO       ";
char *adv_name0		= "NewsSpreadViaBLEride";
char *adv_name1 	= "ID-Rider 24.09.2016 ";
char *adv_name2 	= " grettings fly out  ";
char *adv_name3 	= "    to friends      ";
char *adv_name4 	= " all over the globe ";
char *adv_name5 	= "best wishes rudi ;-)";
char *adv_name6 	= "   txs espressif    ";
char *adv_name7 	= "for the ESP32-D0WDQ6";
char *adv_nameRST	= "ESP-BLE-HELLO       ";



/* the flexible STRING var for the Button, UART, or other doings */
/* init as 20 Byte dummy */

char MyMsg[]		= "NewsSpreadViaBLEride";

/* the pointer for CONST STRING and the STRING VAR for later swap/set */
char *p = (char*)&MyMsg[0];

/* message idx */
uint8_t idx = 0; 

/* the swap  */ 
/* this is only for your hand that you can use if you want SWAP addresses from CONST :) */
/* small examples are here, but commented */
void swap(char **a, char **b);

/* newset  */
void setnew(char **a, char **b);

/* the user button proc */
/* you can use what you mant */
void userbutton(int myButtonId);

/* the swap routine */ 
/* we swap here simple with pointers the CONST String / String var */ 
void swap(char **a, char **b) {
  char * t = *a;
  *a = *b;
  *b = t;
}

/* the setnew routine */
/* we set here simple the value by set to a new value using pointer/address */
void setnew(char **a, char **b) {
  *a = *b;
 }

/* the user button routine */
void userbutton(int myButton) {
	
	switch (myButton) {
		
		case 0: strcpy(MyMsg, "User Button 00000000"); setnew(&adv_name, &p);break;
		case 1: strcpy(MyMsg, "User Button 11111111"); setnew(&adv_name, &p);break;
		case 2: strcpy(MyMsg, "User Button 22222222"); setnew(&adv_name, &p);break;
	}
}



/* end of NewsSpreadViaBLEride */ 
/*24 sept 2016 done!*/




static uint8_t hci_cmd_buf[128];

/* 
 * @brief: BT controller callback function, used to notify the upper layer that
 *         controller is ready to receive command
 */
static void controller_rcv_pkt_ready(void)
{
    printf("controller rcv pkt ready\n");
}

/* 
 * @brief: BT controller callback function, to transfer data packet to upper
 *         controller is ready to receive command
 */
static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    printf("host rcv pkt: ");
    for (uint16_t i=0; i<len; i++)
        printf("%02x", data[i]);
    printf("\n");
    return 0;
}

static vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,
    host_rcv_pkt
};

static uint16_t make_cmd_reset(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_RESET);
    UINT8_TO_STREAM (buf, 0);
    return HCI_H4_CMD_PREAMBLE_SIZE;
}

static uint16_t make_cmd_ble_set_adv_enable (uint8_t *buf, uint8_t adv_enable)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM (buf, adv_enable);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_WRITE_ADV_ENABLE;
}

static uint16_t make_cmd_ble_set_adv_param (uint8_t *buf, uint16_t adv_int_min, uint16_t adv_int_max,
                                            uint8_t adv_type, uint8_t addr_type_own,
                                            uint8_t addr_type_dir, bd_addr_t direct_bda,
                                            uint8_t channel_map, uint8_t adv_filter_policy)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_PARAMS);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS );

    UINT16_TO_STREAM (buf, adv_int_min);
    UINT16_TO_STREAM (buf, adv_int_max);
    UINT8_TO_STREAM (buf, adv_type);
    UINT8_TO_STREAM (buf, addr_type_own);
    UINT8_TO_STREAM (buf, addr_type_dir);
    BDADDR_TO_STREAM (buf, direct_bda);
    UINT8_TO_STREAM (buf, channel_map);
    UINT8_TO_STREAM (buf, adv_filter_policy);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS;
}


static uint16_t make_cmd_ble_set_adv_data(uint8_t *buf, uint8_t data_len, uint8_t *p_data)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_DATA);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1);

    memset(buf, 0, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA);

    if (p_data != NULL && data_len > 0) {
        if (data_len > HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA) {
            data_len = HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA;
        }

        UINT8_TO_STREAM (buf, data_len);

        ARRAY_TO_STREAM (buf, p_data, data_len);
    }
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1;
}

static void hci_cmd_send_reset(void)
{
    uint16_t sz = make_cmd_reset (hci_cmd_buf);
    API_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_adv_start(void)
{
    uint16_t sz = make_cmd_ble_set_adv_enable (hci_cmd_buf, 1);
    API_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_param(void)
{
    uint16_t adv_intv_min = 256; // 160ms
    uint16_t adv_intv_max = 256; // 160ms
    uint8_t adv_type = 0; // connectable undirected advertising (ADV_IND)
    uint8_t own_addr_type = 0; // Public Device Address
    uint8_t peer_addr_type = 0; // Public Device Address
    uint8_t peer_addr[6] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85};
    uint8_t adv_chn_map = 0x07; // 37, 38, 39
    uint8_t adv_filter_policy = 0; // Process All Conn and Scan
    
    uint16_t sz = make_cmd_ble_set_adv_param(hci_cmd_buf,
                                             adv_intv_min,
                                             adv_intv_max,
                                             adv_type,
                                             own_addr_type,
                                             peer_addr_type,
                                             peer_addr,
                                             adv_chn_map,
                                             adv_filter_policy);
    API_vhci_host_send_packet(hci_cmd_buf, sz);
}


static void hci_cmd_send_ble_set_adv_data(void)
{
	
	// change was to do this as global
	// char *adv_name = "ESP-BLE-HELLO";
	
	// for uart debug 
	ets_printf("***********>DEBUG IDX: %d: %s \n", idx, adv_name);

	// we use 12  message here
	// 0..11 = 12 messages
	// if 12 then set it to 0
	// we use cases for the idx
	
	if (idx == 12) idx = 0;
	

	
	switch (idx) {
	
	/* Variant A: swap pointer const string */
	/*	
    case 0: swap(&adv_name, &adv_name6);	break;
    case 1: swap(&adv_name, &adv_name0);    break;
	case 2: swap(&adv_name, &adv_name1);	break;
	case 3: swap(&adv_name, &adv_name2);	break;
	case 4: swap(&adv_name, &adv_name3);	break;
	case 5: swap(&adv_name, &adv_name4);	break;
	
	// and so on... you must sort by your self if you use "swap"
	
	*/

	/* Variant B: simple newset over pointer address */
	/* */
	case 0: setnew(&adv_name, &adv_nameRST);	break;				// ESP-BLE-HELLO       
    case 1: setnew(&adv_name, &adv_name0);  	break;				// NewsSpreadViaBLEride
	case 2: setnew(&adv_name, &adv_name1);		break; 				// ID-Rider 24.09.2016 
	case 3: setnew(&adv_name, &adv_name2);		break; 				//  grettings fly out  
	case 4: setnew(&adv_name, &adv_name3);		break; 				//     to friends      
	case 5: setnew(&adv_name, &adv_name4);		break; 				//  all over the globe 
	case 6: setnew(&adv_name, &adv_name5);		break; 				// best wishes rudi ;-)
	case 7: setnew(&adv_name, &adv_name6);		break; 				//    txs espressif    
	case 8: setnew(&adv_name, &adv_name7);		break; 				// for the ESP32-D0WDQ6
	
	
	// create user button cb, uart isr or others, here as example by simple string var
	// in your button call put simple your new valu by 
	// 
	// strcpy(MyMsg, "YOUR 20 BYTE MAX MSG"); 
	// setnew(&adv_name, &p); 
	// here as case sample..
	// see userbutton(...)
	
	case  9: userbutton(0); break;	// strcpy(MyMsg, "User Button 00000000"); setnew(&adv_name, &p); break;
	case 10: userbutton(1); break;  // strcpy(MyMsg, "User Button 11111111"); setnew(&adv_name, &p); break;
	case 11: userbutton(2); break;  // strcpy(MyMsg, "User Button 22222222"); setnew(&adv_name, &p); break;

 }
 
 /* end of added */ 
 
	
	uint8_t name_len = (uint8_t)strlen(adv_name);
    uint8_t adv_data[31] = {0x02, 0x01, 0x06, 0x0, 0x09};
    uint8_t adv_data_len;
    
    adv_data[3] = name_len + 1;
    for (int i=0; i<name_len; i++) {
        adv_data[5+i] = (uint8_t)adv_name[i];
    }
    adv_data_len = 5 + name_len;

    uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
    API_vhci_host_send_packet(hci_cmd_buf, sz);
}

/*
 * @brief: send HCI commands to perform BLE advertising;
 */
void bleAdvtTask(void *pvParameters)
{
    int cmd_cnt = 0;
    bool send_avail = false;
    API_vhci_host_register_callback(&vhci_host_cb);
    printf("BLE advt task start\n");
    while (1) {
		/* NOTE: you can change the task rhytmen here: */
		/* Standard is: 1000 ( millisecond 1000 = 1 second ) */
		// vTaskDelay(1000 / portTICK_PERIOD_MS);
		// changed for Message Sprew the 1000 to a VAR
        vTaskDelay(tickering / portTICK_PERIOD_MS);
        send_avail = API_vhci_host_check_send_available();
        if (send_avail) {
            switch (cmd_cnt) {
            case 0: hci_cmd_send_reset(); ++cmd_cnt; break;
            case 1: hci_cmd_send_ble_set_adv_param(); ++cmd_cnt; break;
            case 2: hci_cmd_send_ble_set_adv_data(); ++cmd_cnt; 	/*added for Message Sprew*/ idx++; 			/* end of change */ break;
            case 3: hci_cmd_send_ble_adv_start(); ++cmd_cnt; 		/*added for Message Sprew*/ cmd_cnt = 0; 	/* end of change */ break;
			
            }
        }
        printf("BLE Advertise, flag_send_avail: %d, cmd_sent: %d\n", send_avail, cmd_cnt);
    }
	
}



void bt_app_main()
{
	
	xTaskCreatePinnedToCore(&bleAdvtTask, "bleAdvtTask", 2048, NULL, 5, NULL, 0);
	
	 
	
	
}


