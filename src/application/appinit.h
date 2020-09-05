#ifndef APPINIT_H_INCLUDED
#define APPINIT_H_INCLUDED

#include "compiler.h"
#include <strings.h>
#include "deca_types.h"
#include "deca_regs.h"
#include "instance.h"
#include "deca_spi.h"
#include "string.h"
#include "stdio.h"
#include "deca_device_api.h"
#include "mds.h"
#include <stddef.h>
#include <stdlib.h>
#include "rtGetInf.h"


#ifndef uint64
typedef unsigned long long uint64;
#endif

#ifndef  int64
typedef long long int64;
#endif

#ifndef TX_ANT_DLY
#define TX_ANT_DLY 16436
#endif
#ifndef RX_ANT_DLY
#define RX_ANT_DLY 16436
#endif

#define MODE_TAG (1)
#define MODE_ANCHOR (2)
#define MODE_MONITOR (3)

typedef struct
{
	unsigned int AppMode;
	unsigned int AncNum;//A0,A1,A2閵嗕堪f Tag,Monitor this is 0
	unsigned int AncSum;
	unsigned int PanId;
	unsigned int LocAddress;


}Slot;
#define SYS_STATUS_ALL_RX_ERR  (SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL | SYS_STATUS_RXSFDTO \
                                | SYS_STATUS_AFFREJ | SYS_STATUS_LDEERR)

/* User defined RX timeouts (frame wait timeout and preamble detect timeout) mask. */
#define SYS_STATUS_ALL_RX_TO   (SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO)

/* All TX events mask. */
#define SYS_STATUS_ALL_TX      (SYS_STATUS_AAT | SYS_STATUS_TXFRB | SYS_STATUS_TXPRS | \
                                SYS_STATUS_TXPHS | SYS_STATUS_TXFRS )

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 4
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define PAN_ID_IDX 3
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define DATA_FRAME_SEQ_NB_IDX 2
#define DATA_FRAME_PAN_ID_IDX 3
#define DATA_FRAME_DEST_ADDR_IDX 5
#define DATA_FRAME_SRC_ADDR_IDX 7
#define DATA_FRAME_APP_FCODE_IDX 9
#define DATA_FRAME_DIST_DEST_ADDR_IDX 10
#define DATA_FRAME_DIST_IDX 12
//the destination of frame function
#define MAC_COOEST_START 0xD1
#define MAC_COOEST_POLL 0xD2
#define MAC_COOEST_RESPONSE 0xD3
#define MAC_COOEST_FINAL 0xD4
#define MAC_COOEST_SUCC 0xD5
#define MAC_COOEST_RESP 0xD6
#define MAC_COOEST_DIST 0xD7
#define MAC_COOEST_AREACOORD 0xD8
#define MAC_COOEST_AREACOORDRESP 0xD9

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DATA_FRAME_COORD_AREA_NUM_IDX 10

#define DATA_FRAME_AREA_SECOND_ANCHOR_ID_IDX 12

#define DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX 13
#define DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX 16
#define DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX 19
#define DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX 23
#define DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX 26
#define DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX 29
#define DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX 33
#define DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX 36
#define DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX 39

#define DATA_FRAME_AREA_THIRD_ANCHOR_ID_IDX 22
#define DATA_FRAME_AREA_FOURTH_ANCHOR_ID_IDX 32
#define DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_ID_IDX 12
#define DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_ID_IDX 22
#define DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_ID_IDX 32
#define DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_ID_IDX 42

#define DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX 13
#define DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX 16
#define DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX 19
#define DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX 23
#define DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX 26
#define DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX 29
#define DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX 33
#define DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX 36
#define DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX 39
#define DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX 43
#define DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX 46
#define DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX 49
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define UUS_TO_DWT_TIME 65536

#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 3300

#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT 299702547
#endif

#define RX_BUF_LEN 56

extern uint8 rx_poll_msg[];
extern uint8 tx_resp_msg[];
extern uint8 rx_final_msg[];
extern uint64 poll_tx_ts;
extern uint64 resp_rx_ts;
extern uint64 final_tx_ts;
extern uint8 tx_poll_msg[] ;
extern uint8 rx_resp_msg[];
extern uint8 tx_final_msg[];



extern uint8 frame_seq_nb;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */

extern uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
extern uint32 status_reg;
extern uint64 poll_rx_ts;
extern uint64 resp_tx_ts;
extern uint64 final_rx_ts;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
extern double tof;
extern double distance;
/* String used to display measured distance on LCD screen (16 characters maximum). */
extern char dist_str[40];
extern Slot Slot_data;


/* Declaration of static functions. */
uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
void final_msg_set_ts(uint8 *ts_field, uint64 ts);

int CoordEstablish();
double CoordEstablish_Init(uint8,uint8);
int CheckMessage(int);
int CoordEstablish_Resp(void);
void MessageSet(unsigned int,uint8,uint8,uint8,uint8 arr[]);
double GetAverage(double arr[],int len);



#endif // APPINIT_H_INCLUDED
