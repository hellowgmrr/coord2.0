/*
* 文件名：
* 描  述：
* 作  者：@Allen
* 时  间：
* 版  权：
*/
#include "appinit.h"

uint8 tx_msg[] = { 0x41, 0x88, 0, 0xCB, 0xDE, 'V', 'E', 'W', 'A', 0xD3, 1,1,1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8 tx_coordmsg[] = { 0x41, 0x88, 0, 0xCB, 0xDE, 'V', 'E', 'W', 'A', 0xD8, 1,1,1,  0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 1, 2, 3, 4 ,5,6,0,0,0,0,0,0,0,0,0,0 };
uint8 tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xD1, 0, 0 };
uint8 tx_final_msg[] = { 0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

uint8 tx_test_msg[] = { 0,1,2,3,4,5,6,7,0xA,0xB,0xC };

/* Frame sequence number, incremented after each transmission. */
uint8 frame_seq_nb = 0;

/* Buffer to store received messages.
* Its size is adjusted to longest frame that this example code is supposed to handle. */

uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
uint32 status_reg = 0;
uint64 poll_rx_ts;
uint64 resp_tx_ts;
uint64 final_rx_ts;
uint64 poll_tx_ts;
uint64 resp_rx_ts;
uint64 final_tx_ts;
#define SingleCellMasterAnchor0 (0)
#define SingleCellSlaveAnchor1 (1)
#define SingleCellSlaveAnchor2 (2)
#define SingleCellSlaveAnchor3 (3)

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
double tof;
double distance;
char dist_str[40] = { 0 };
char dist_[50] = { 0 };


uint64 get_tx_timestamp_u64(void)
{
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readtxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
*
* @brief Get the RX time-stamp in a 64-bit variable.
*        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
*
* @param  none
*
* @return  64-bit value of the read time-stamp.
*/
uint64 get_rx_timestamp_u64(void)
{
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readrxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn final_msg_get_ts()
*
* @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
*        significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to read
*         ts  timestamp value
*
* @return none
*/
void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
	int i;
	*ts = 0;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		*ts += ts_field[i] << (i * 8);
	}
}
void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
	int i;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++)
	{
		ts_field[i] = (uint8)ts;
		ts >>= 8;
	}
}

int CheckMessage(int num)
{
	if (Slot_data.PanId == ((rx_buffer[PAN_ID_IDX] << 8) | rx_buffer[PAN_ID_IDX + 1]))
	{
		if ((rx_buffer[DATA_FRAME_DEST_ADDR_IDX] == Slot_data.AppMode) && (rx_buffer[DATA_FRAME_DEST_ADDR_IDX + 1] == Slot_data.AncNum))
		{
			if (rx_buffer[DATA_FRAME_APP_FCODE_IDX] == num)
			{
				return 1;
			}
		}
	}
	return 0;
}

void MessageSet(unsigned int Panid, uint8 mode, uint8 num, uint8 contbit, uint8 arr[])
{
	arr[0] = 0x41;
	arr[1] = 0x88;
	arr[PAN_ID_IDX] = (Panid & 0xff00) >> 8;
	arr[PAN_ID_IDX + 1] = (Panid & 0x00ff);
	arr[DATA_FRAME_DEST_ADDR_IDX] = mode;
	arr[DATA_FRAME_DEST_ADDR_IDX + 1] = num;
	arr[DATA_FRAME_SRC_ADDR_IDX] = Slot_data.AppMode;
	arr[DATA_FRAME_SRC_ADDR_IDX + 1] = Slot_data.AncNum;
	arr[DATA_FRAME_APP_FCODE_IDX] = contbit;
}

double GetAverage(double arr[], int len)
{
	int i;
	double min = arr[0], max = arr[0], aver = 0;
	for (i = 0; i<len; i++)
	{
		if (arr[i] <= min) min = arr[i];
		if (arr[i] >= max) max = arr[i];
		aver += arr[i];
	}
	//printf("min:%f\nmax:%f\n",min,max);
	aver -= (min + max);
	aver /= (len - 2);
	//printf("average:%f",aver);
	return aver;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AreaCoordInfor_2_tx_coordmsgFrame(double(*Result_Arr)[3], uint8 *tx_coordMsg)
{
	tx_coordMsg[DATA_FRAME_COORD_AREA_NUM_IDX] = MODE_ANCHOR;
	tx_coordMsg[DATA_FRAME_COORD_AREA_NUM_IDX + 1] = ((Slot_data.AncNum) + 2) / 2;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_ID_IDX] = Slot_data.AncNum + 1;
	tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_ID_IDX] = Slot_data.AncNum + 2;
	tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_ID_IDX] = Slot_data.AncNum + 3;
	double coord_res = 0;
	int temp2 = 0;
	coord_res = Result_Arr[1][0];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = Result_Arr[1][1];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = Result_Arr[1][2];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	////////////////////////
	coord_res = Result_Arr[2][0];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = Result_Arr[2][1];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = Result_Arr[2][2];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	///////////////////////
	coord_res = Result_Arr[3][0];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = Result_Arr[3][1];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 2] = temp2 & 0x0000ff;
	}

	coord_res = Result_Arr[3][2];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 2] = temp2 & 0x0000ff;
	}
}

void FrameInfor_2_AreaCoordArray(uint8 *rxbuffer, double(*areacoord)[9])
{
	int num3 = (int)(rxbuffer[DATA_FRAME_COORD_AREA_NUM_IDX + 1]);
	int tmp1 = 0;
	double dist_tmp1 = 0;
	char a[1];
	if (((rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][0] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][0] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][1] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][1] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][2] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][2] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][3] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][3] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][4] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][4] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][5] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][5] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][6] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][6] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][7] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][7] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][8] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		areacoord[num3][8] = (dist_tmp1*(-1));
	}
}

double CoordEstablish_Init(uint8 dest_mode, uint8 dest_num)
{
	int CurrNum = 0, NumOfTest = 10;
	double DistArr[10];
	while (CurrNum < NumOfTest)
	{


		//static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCB, 0xDE, 'V', 'E', 'W', 'A', 0xD3, 1,1,1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		MessageSet(Slot_data.PanId, dest_mode, dest_num, MAC_COOEST_START, tx_msg);
		dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
		dwt_writetxfctrl(sizeof(tx_msg), 0);
		dwt_starttx(DWT_START_TX_IMMEDIATE);
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
		{
		}
		if (status_reg&SYS_STATUS_TXFRS)
		{
			//TcpTx("发送成功！",10);
		}
		else {
			//TcpTx("发送失败！",10);
		}
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);


		dwt_setrxtimeout(10000);

		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

		/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
		{
		};

		if (status_reg & SYS_STATUS_RXFCG)
		{
			//TcpTx("Rx  test",20);
			uint32 frame_len;

			/* Clear good RX frame event in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
			if (frame_len <= RX_BUFFER_LEN)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}

			/* Check that the frame is a poll sent by "DS TWR initiator" example.
			* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			rx_buffer[ALL_MSG_SN_IDX] = 0;
			if (CheckMessage(MAC_COOEST_POLL))
			{
				uint32 resp_tx_time;
				int ret;

				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();

				/* Set send time for response. See NOTE 9 below. */
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

				/* Write and send the response message. See NOTE 10 below.*/
				tx_msg[ALL_MSG_SN_IDX] = CurrNum;
				if (CurrNum == NumOfTest - 1)tx_msg[ALL_MSG_SN_IDX] = 0x10;
				MessageSet(Slot_data.PanId, dest_mode, dest_num, MAC_COOEST_RESPONSE, tx_msg);
				dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_msg), 0); /* Zero offset in TX buffer, ranging. */
				ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

				/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
				if (ret == DWT_ERROR)
				{
					continue;
				}

				/* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{
				};

				/* Increment frame sequence number after transmission of the response message (modulo 256). */
				frame_seq_nb++;

				if (status_reg & SYS_STATUS_RXFCG)
				{
					/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

					/* A frame has been received, read it into the local buffer. */
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					/* Check that the frame is a final message sent by "DS TWR initiator" example.
					* As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (CheckMessage(MAC_COOEST_FINAL))
					{
						uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
						uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
						double Ra, Rb, Da, Db;
						int64 tof_dtu;

						/* Retrieve response transmission and final reception timestamps. */
						resp_tx_ts = get_tx_timestamp_u64();
						final_rx_ts = get_rx_timestamp_u64();

						/* Get timestamps embedded in the final message. */
						final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

						/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
						poll_rx_ts_32 = (uint32)poll_rx_ts;
						resp_tx_ts_32 = (uint32)resp_tx_ts;
						final_rx_ts_32 = (uint32)final_rx_ts;
						Ra = (double)(resp_rx_ts - poll_tx_ts);
						Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
						Da = (double)(final_tx_ts - resp_rx_ts);
						Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
						tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

						tof = tof_dtu * DWT_TIME_UNITS;
						distance = tof * SPEED_OF_LIGHT;

						/* Display computed distance on LCD. */
						//sprintf(dist_str, "DIST: %3.2f m", distance);
						//TcpTx(dist_str,15);
						//Sleep(10);
						DistArr[CurrNum] = distance;
						CurrNum++;
						//                        sprintf(dist_str,"%d",CurrNum);
						//                        TcpTx(dist_str,15);
						//  Sleep(200);
					}
				}
				else
				{
					/* Clear RX error/timeout events in the DW1000 status register. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

					/* Reset RX to properly reinitialise LDE operation. */
					dwt_rxreset();
				}
			}
		}
		else
		{
			/* Clear RX error/timeout events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

			/* Reset RX to properly reinitialise LDE operation. */
			dwt_rxreset();
		}

	}
	//return 	(DistArr,10);
	return GetAverage(DistArr, 10);
}

int CoordEstablish_Resp(void)
{
	static uint8 cnt, addr;
	//	  while(1){
	uint32 frame_len;
	int resp_enable = 0;
	uint8 dest_mode, dest_num;
	dwt_setrxtimeout(10000);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	{
	};
	if (status_reg & SYS_STATUS_RXFCG)
	{
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		if (frame_len <= RX_BUF_LEN)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}
		if (CheckMessage(MAC_COOEST_START))
		{
			resp_enable = 1;
			dest_mode = rx_buffer[DATA_FRAME_SRC_ADDR_IDX];
			dest_num = rx_buffer[DATA_FRAME_SRC_ADDR_IDX + 1];
		}
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
	}
	else {
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
		dwt_rxreset();
	}


	/* Loop forever initiating ranging exchanges. */
	if (resp_enable)
	{
		dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
		dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS + 700);
		/* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
		tx_poll_msg[ALL_MSG_SN_IDX] = 0;
		MessageSet(Slot_data.PanId, dest_mode, dest_num, MAC_COOEST_POLL, tx_poll_msg);
		dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
		dwt_writetxfctrl(sizeof(tx_poll_msg), 0); /* Zero offset in TX buffer, ranging. */

												  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
												  * set by dwt_setrxaftertxdelay() has elapsed. */
		dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

		/* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
		{
		};

		/* Increment frame sequence number after transmission of the poll message (modulo 256). */
		frame_seq_nb++;

		if (status_reg & SYS_STATUS_RXFCG)
		{
			//TcpTx("Rx succ",15)

			/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
			if (frame_len <= RX_BUF_LEN)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}

			/* Check that the frame is the expected response from the companion "DS TWR responder" example.
			* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			//rx_buffer[ALL_MSG_SN_IDX] = 0;
			if (CheckMessage(MAC_COOEST_RESPONSE))
			{
				uint32 final_tx_time;
				int ret;

				/* Retrieve poll transmission and response reception timestamp. */
				poll_tx_ts = get_tx_timestamp_u64();
				resp_rx_ts = get_rx_timestamp_u64();

				/* Compute final message transmission time. See NOTE 10 below. */
				final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(final_tx_time);

				/* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
				final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

				/* Write all timestamps in the final message. See NOTE 11 below. */
				final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
				final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
				final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

				/* Write and send final message. See NOTE 8 below. */
				tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				MessageSet(Slot_data.PanId, dest_mode, dest_num, MAC_COOEST_FINAL, tx_final_msg);
				dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_final_msg), 0); /* Zero offset in TX buffer, ranging. */
				ret = dwt_starttx(DWT_START_TX_DELAYED);

				/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
				if (ret == DWT_SUCCESS)
				{
					/* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
					while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
					{
					};

					/* Clear TXFRS event. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

					/* Increment frame sequence number after transmission of the final message (modulo 256). */
					frame_seq_nb++;
				}

				if ((rx_buffer[ALL_MSG_SN_IDX] == 0x10) && (rx_buffer[DATA_FRAME_SRC_ADDR_IDX + 1]) == Slot_data.AncNum - 1)
				{
					int p = 20;
					while (p--)
					{
						MessageSet(Slot_data.PanId, dest_mode, dest_num, MAC_COOEST_FINAL, tx_final_msg);
						dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
						dwt_writetxfctrl(sizeof(tx_final_msg), 0); /* Zero offset in TX buffer, ranging. */
						ret = dwt_starttx(DWT_START_TX_DELAYED);

						/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
						if (ret == DWT_SUCCESS)
						{
							/* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
							while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
							{
							};

							/* Clear TXFRS event. */
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

							/* Increment frame sequence number after transmission of the final message (modulo 256). */
							//frame_seq_nb++;
						}
						sleep_ms(500);



					}

					return 1;
				}
				//                        cnt = rx_buffer[ALL_MSG_SN_IDX];
				//                        addr = rx_buffer[DATA_FRAME_SRC_ADDR_IDX+1];
			}
		}
		else
		{
			//		        	if(status_reg & SYS_STATUS_ALL_RX_TO)
			//		        	{
			//		        		if((cnt == 0x10)&&(addr == Slot_data.AncNum-1))return 1;
			//		        	}
			/* Clear RX error/timeout events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

			/* Reset RX to properly reinitialise LDE operation. */
			dwt_rxreset();
		}

		/* Execute a delay between ranging exchanges. */
	}
	sleep_ms(500);
	return 0;
	//	  }

}

void MasterAnchorCoordestablish(int main_anchor_ID, double(*coordresultArr)[3], int *flag)
{
	double dist_arr[6][6] = { { 0 } };
	double result_arr[6][3] = { { 0 } };
	int j;
	int rec_num = 9;//此处是为了让收到的多组同样的距离数据只显示一次
	for (j = main_anchor_ID + 3; j>main_anchor_ID; j--)
	{
		double dist = CoordEstablish_Init(MODE_ANCHOR, j);
		sprintf(dist_str, "DIST%d %d:%4.3f m", main_anchor_ID, j, dist);
		TcpTx(dist_str, 15);
		sleep_ms(50);
		dist_arr[0][j - main_anchor_ID] = dist;
		dist_arr[j - main_anchor_ID][0] = dist;
	}
	while (1)
	{
		dwt_setrxtimeout(0);
		uint32 frame_len;
		uint8 num1, num2;
		double dist_tmp;
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
		{
		};
		if (status_reg & SYS_STATUS_RXFCG)
		{
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
			if (frame_len <= RX_BUF_LEN)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}
			if (CheckMessage(MAC_COOEST_DIST))
			{
				num1 = rx_buffer[DATA_FRAME_SRC_ADDR_IDX + 1];
				num2 = rx_buffer[DATA_FRAME_DIST_DEST_ADDR_IDX + 1];
				int tmp = (rx_buffer[DATA_FRAME_DIST_IDX] << 16) | (rx_buffer[DATA_FRAME_DIST_IDX + 1] << 8) | (rx_buffer[DATA_FRAME_DIST_IDX + 2]);
				dist_tmp = (double)(tmp / 10000.0);
				if (rec_num != num2) {
					sprintf(dist_str, "DIST%d%d:%4.3f m", num1, num2, dist_tmp);
					TcpTx(dist_str, 16);
				}
				dist_arr[num1 - main_anchor_ID][num2 - main_anchor_ID] = dist_tmp;
				dist_arr[num2 - main_anchor_ID][num1 - main_anchor_ID] = dist_tmp;
				if ((num2 == main_anchor_ID + Slot_data.AncSum - 1) && (num1 == main_anchor_ID + Slot_data.AncSum - 2))
					break;
				rec_num = num2;
			}
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
		}
		else
		{
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
			dwt_rxreset();

		}
	}
	sleep_ms(100);
	TcpTx("Break", 10);
	main_mds(dist_arr, Slot_data.AncSum, coordresultArr);
	int i;
	for (i = 0; i<Slot_data.AncSum; i++)
	{
		sleep_ms(100);
		sprintf(dist_str, "x:%4.3f y:%4.3f z:%4.3f", coordresultArr[i][0], coordresultArr[i][1], coordresultArr[i][2]);
		TcpTx(dist_str, 40);
	}
	(*flag) = 1;
}

void SlaveAnchorCoordestablish(int main_anchor_ID)
{
	int init_enable = 0;
	while (!init_enable)
	{
		init_enable = CoordEstablish_Resp();
	}
	if (init_enable)
	{
		int j;
		double dist;
		for (j = main_anchor_ID + 3; j>Slot_data.AncNum; j--)
		{
			dist = CoordEstablish_Init(MODE_ANCHOR, j);
			//			sprintf(dist_str, "DIST%d%d:%f.4 m", Slot_data.AncNum, j, dist);
			//			TcpTx(dist_str, 15);
			sleep_ms(10000);//需要避免信道繁忙，保证一个时刻只有一个设备在发送，这个时间很重要。
			int hh = 10;
			while (hh--)
			{

				tx_msg[DATA_FRAME_DIST_DEST_ADDR_IDX] = MODE_ANCHOR;
				tx_msg[DATA_FRAME_DIST_DEST_ADDR_IDX + 1] = j;
				int tmp1 = (int)(dist * 10000);
				tx_msg[DATA_FRAME_DIST_IDX] = (tmp1 & 0xff0000) >> 16;
				tx_msg[DATA_FRAME_DIST_IDX + 1] = (tmp1 & 0x00ff00) >> 8;
				tx_msg[DATA_FRAME_DIST_IDX + 2] = tmp1 & 0x0000ff;
				MessageSet(Slot_data.PanId, MODE_ANCHOR, main_anchor_ID, MAC_COOEST_DIST, tx_msg);
				dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
				dwt_writetxfctrl(sizeof(tx_msg), 0);
				dwt_starttx(DWT_START_TX_IMMEDIATE);
				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
				{
				}
				if (status_reg&SYS_STATUS_TXFRS)
				{
					//sprintf(dist_str,"%x%x%x",tx_msg[DATA_FRAME_DIST_IDX],tx_msg[DATA_FRAME_DIST_IDX+1],tx_msg[DATA_FRAME_DIST_IDX+2]);
					//TcpTx(dist_str,15);
					//Sleep(50);
				}
				else {
					//TcpTx("发送失败！",10);
				}
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
				sleep_ms(800);
			}
		}
	}

}

int SelectCellCoordEstablishID(int main_anchor_ID, int mode, int mode_num)
{
	int CellCoordEstablishID = 5;
	if ((mode == MODE_ANCHOR) && (mode_num == main_anchor_ID))
		CellCoordEstablishID = 0;
	if ((mode == MODE_ANCHOR) && (mode_num == main_anchor_ID + 1))
		CellCoordEstablishID = 1;
	if ((mode == MODE_ANCHOR) && (mode_num == main_anchor_ID + 2))
		CellCoordEstablishID = 2;
	if ((mode == MODE_ANCHOR) && (mode_num == main_anchor_ID + 3))
		CellCoordEstablishID = 3;
	return CellCoordEstablishID;
}

void SingleCellCoordResultTransmit2A0(int area_sum, double(*coordresultarr)[3], int *flag, double(*Areacoord)[9])
{
 if(*flag==1)
 {
	if (Slot_data.AncNum == (area_sum * 2 - 2))//(area_sum*2-2))
	{
		//sleep(30);
		sleep_ms(200);
		TcpTx("jieshou1", 10);
		AreaCoordInfor_2_tx_coordmsgFrame(coordresultarr, tx_coordmsg);
		int h2 = 20;
		uint8 bb = Slot_data.AncNum - 2;
		while (h2)
		{
			MessageSet(Slot_data.PanId, MODE_ANCHOR, bb, MAC_COOEST_AREACOORD, tx_coordmsg);
			//			    int i5;
			//			    for(i5 = 0; i5<45; i5++)
			//			    {
			//			    char tmpa[45];
			//			    sprintf(tmpa,"%x",tx_coordmsg[i5]);
			//			    TcpTx(tmpa,45);
			//			    sleep(1);
			//			    }
			dwt_writetxdata(sizeof(tx_coordmsg), tx_coordmsg, 0);
			dwt_writetxfctrl(sizeof(tx_coordmsg), 0);
			dwt_starttx(DWT_START_TX_IMMEDIATE);
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
			{
			}
			if (status_reg&SYS_STATUS_TXFRS)
			{
				//			    	sprintf(dist_str,"%x%x%x",tx_msg[DATA_FRAME_DIST_IDX],tx_msg[DATA_FRAME_DIST_IDX+1],tx_msg[DATA_FRAME_DIST_IDX+2]);
				//			    	TcpTx(dist_str,15);
				//			    	Sleep(50);
				h2--;
			}
			else
			{
				//TcpTx("发送失败！",10);
			}
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
			sleep_ms(200);
		}
		TcpTx("mydatasen",20);
	}
	else if (Slot_data.AncNum == 0)///////一直接收
	{
		sleep(1);
		//TcpTx("jieshou2", 10);
		//sleep(30);
		int j1=1;
		while (j1)
		{
			//sleep_ms(500);
			//TcpTx("jieshou3", 10);
			//sleep_ms(20);
			dwt_setrxtimeout(0);
			uint32 frame_len1;
			//uint8 num3;//area number
			//double AreaCoord[][9] = { { 0 } };
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
//			TcpTx("jieshou4", 10);
//			sleep_ms(20);
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
			{
			};
			if (status_reg & SYS_STATUS_RXFCG)
			{
				//TcpTx("jieshou5", 10);
				frame_len1 = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len1 <= RX_BUF_LEN)
				{
					//TcpTx("jieshou9", 10);
					dwt_readrxdata(rx_buffer, frame_len1, 0);
				}
				if (CheckMessage(MAC_COOEST_AREACOORD))
				{
					//TcpTx("jieshou8", 10);
				//	sleep_ms(10);
					FrameInfor_2_AreaCoordArray(rx_buffer, Areacoord);
					sleep(5);

					if (rx_buffer[DATA_FRAME_COORD_AREA_NUM_IDX + 1] == 2)
					{
						TcpTx("all_area_coord_have_received_succ", 40);
						int i1 = 0;
																	for (i1 = 2; i1 <= area_sum; i1++)
																	{
																		TcpTx("succdisplay", 40);
																		sleep_ms(200);
																		//sprintf(dist_str,"AREA:%d Sx:%4.3f Sy:%4.3f Sz:%4.3f\n Tx:%4.3f Ty:%4.3f Tz:%4.3f\n Fx:%4.3f Fy:%4.3f Fz:%4.3f\n",i1,AreaCoord[i1][0],AreaCoord[i1][1],AreaCoord[i1][2],AreaCoord[i1][3],AreaCoord[i1][4],AreaCoord[i1][5],AreaCoord[i1][6],AreaCoord[i1][7],AreaCoord[i1][8]);
																		//TcpTx(dist_str,20);
																		sprintf(dist_, "AREA:%d Sx:%4.3f Sy:%4.3f Sz:%4.3f\n", i1, Areacoord[i1][0], Areacoord[i1][1], Areacoord[i1][2]);
																		TcpTx(dist_, 50);
																		sleep_ms(200);
																		sprintf(dist_, "AREA:%d Tx:%4.3f Ty:%4.3f Tz:%4.3f\n", i1, Areacoord[i1][3], Areacoord[i1][4], Areacoord[i1][5]);
																		TcpTx(dist_, 50);
																		sleep_ms(200);
																		sprintf(dist_, "AREA:%d Fx:%4.3f Fy:%4.3f Fz:%4.3f\n", i1, Areacoord[i1][6], Areacoord[i1][7], Areacoord[i1][8]);
																		TcpTx(dist_, 50);
																		sleep_ms(200);
																	}
						j1=0;
					}
				}
				//TcpTx("jieshou6", 10);
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
			}
			else
			{
				//TcpTx("jieshou7", 10);
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
				dwt_rxreset();

			}
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	else
	{
		int j1=1;
		while (j1)//循环接收并转发来自main_anchor_ID+2的area_coord_infor,直至收到来自area_sum*2-2的信息转发完成后退出
		{
			TcpTx("zaizhe",20);
			sleep_ms(20);

			uint32 frame_len2;
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
			{
			};
			if (status_reg & SYS_STATUS_RXFCG)
			{
				frame_len2 = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len2 <= RX_BUF_LEN)
				{
					dwt_readrxdata(rx_buffer, frame_len2, 0);
				}
				if (CheckMessage(MAC_COOEST_AREACOORD))
				{
//					TcpTx("jieshou1", 10);
//					sleep_ms(50);
					//int i5;
//					for(i5 = 0; i5<45; i5++)
//					{
//					char tmpa[45];
//					sprintf(tmpa,"%x",rx_buffer[i5]);
//					TcpTx(tmpa,45);
//					sleep_ms(50);
//					}
//					TcpTx("outputrece", 20);
//					sleep_ms(50);
//					TcpTx("jieshou2", 10);
					memcpy(tx_coordmsg, rx_buffer, sizeof(rx_buffer));//最重要的验证此步是否可行
					TcpTx("copyover3", 10);
					sleep(5);
					int h4 = 30;
					int mycellid=(((Slot_data.AncNum)+2)/2);
					char cc=(Slot_data.AncNum-2);
					while (h4)
					{
						//TcpTx("jieshou4", 10);
						MessageSet(Slot_data.PanId, MODE_ANCHOR, cc, MAC_COOEST_AREACOORD, tx_coordmsg);
						//TcpTx("jieshou5", 10);
//						int i6;
//											for(i6 = 0; i6<45; i6++)
//											{
//											char tmpa[45];
//											sprintf(tmpa,"%x",tx_coordmsg[i6]);
//											TcpTx(tmpa,45);
//											sleep_ms(50);
//											}
//											TcpTx("senddata", 10);
//											sleep(60);
						dwt_writetxdata(sizeof(tx_coordmsg), tx_coordmsg, 0);
						dwt_writetxfctrl(sizeof(tx_coordmsg), 0);
						dwt_starttx(DWT_START_TX_IMMEDIATE);
						while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
						{
						}
						if (status_reg&SYS_STATUS_TXFRS)
						{
							//			    	sprintf(dist_str,"%x%x%x",tx_msg[DATA_FRAME_DIST_IDX],tx_msg[DATA_FRAME_DIST_IDX+1],tx_msg[DATA_FRAME_DIST_IDX+2]);
							//			    	TcpTx(dist_str,15);
							//			    	Sleep(50);
							h4--;
						}
						else
						{
							//TcpTx("发送失败！",10);
						}
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
						sleep_ms(200);
					}

					if (rx_buffer[DATA_FRAME_COORD_AREA_NUM_IDX + 1] == (mycellid+1))
					{
						TcpTx("all_area_coord_have_received_succ", 40);
						j1=0;

					}


				}
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
			}
			else
			{
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
				dwt_rxreset();

			}

		}

		sleep(15);
		TcpTx("jieshou90", 10);
		AreaCoordInfor_2_tx_coordmsgFrame(coordresultarr, tx_coordmsg);
		int h3 = 30;
		uint8 cc = Slot_data.AncNum - 2;
		TcpTx("sendmydata", 40);
		while (h3)
		{
			MessageSet(Slot_data.PanId, MODE_ANCHOR, cc, MAC_COOEST_AREACOORD, tx_coordmsg);
			dwt_writetxdata(sizeof(tx_coordmsg), tx_coordmsg, 0);
			dwt_writetxfctrl(sizeof(tx_coordmsg), 0);
			dwt_starttx(DWT_START_TX_IMMEDIATE);
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
			{
			}
			if (status_reg&SYS_STATUS_TXFRS)
			{
				//			    	sprintf(dist_str,"%x%x%x",tx_msg[DATA_FRAME_DIST_IDX],tx_msg[DATA_FRAME_DIST_IDX+1],tx_msg[DATA_FRAME_DIST_IDX+2]);
				//			    	TcpTx(dist_str,15);
				//			    	Sleep(50);
				h3--;
			}
			else
			{
				//TcpTx("发送失败！",10);
			}
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
			sleep_ms(200);
		}


	}
}
}


int CoordEstablish(int main_anchor_ID, int *area_num, double(*coordresultarr)[3], int *flag)
{
	int mode = Slot_data.AppMode;
	int mode_num = Slot_data.AncNum;
	int CellCoordEstablishID = 0;
	CellCoordEstablishID = SelectCellCoordEstablishID(main_anchor_ID, mode, mode_num);

	switch (CellCoordEstablishID)
	{
	case SingleCellMasterAnchor0:
		TcpTx("IDyigaiwei0", 20);
		MasterAnchorCoordestablish(main_anchor_ID, coordresultarr, &(*flag));
		(*area_num)++;
		break;
	case SingleCellSlaveAnchor1:
		SlaveAnchorCoordestablish(main_anchor_ID);
		(*area_num)++;
		break;
	case SingleCellSlaveAnchor2:
		SlaveAnchorCoordestablish(main_anchor_ID);
		(*area_num)++;
		break;
	case SingleCellSlaveAnchor3:
		SlaveAnchorCoordestablish(main_anchor_ID);
		(*area_num)++;
		break;
	default:
		(*area_num)++;
		sleep(30);
		break;
	}
}




void CoordSwitch2CellOne(double(*AreaCoord)[9], double(*SwitchAreaCoord)[12],int areasum)
{
	double temparr[4][3] = { { 0 } };
		int i1 = 0;
		for (i1 = 2; i1 <= areasum; i1++)
		{
			temparr[0][0] = 0;
			temparr[0][1] = 0;
			temparr[0][2] = 0;
			temparr[1][0] = AreaCoord[i1][0];
			temparr[1][1] = AreaCoord[i1][1];
			temparr[1][2] = AreaCoord[i1][2];
			temparr[2][0] = AreaCoord[i1][3];
			temparr[2][1] = AreaCoord[i1][4];
			temparr[2][2] = AreaCoord[i1][5];
			temparr[3][0] = AreaCoord[i1][6];
			temparr[3][1] = AreaCoord[i1][7];
			temparr[3][2] = AreaCoord[i1][8];
			int i2 = 0;
			int j2 = 0;
			for (i2 = 0; i2<4; i2++)
				for (j2 = 0; j2<3; j2++)
				{
					temparr[i2][j2] = temparr[i2][j2] + 2;
				}
			SwitchAreaCoord[i1][0] = temparr[0][0];
			SwitchAreaCoord[i1][1] = temparr[0][1];
			SwitchAreaCoord[i1][2] = temparr[0][2];
			SwitchAreaCoord[i1][3] = temparr[1][0];
			SwitchAreaCoord[i1][4] = temparr[1][1];
			SwitchAreaCoord[i1][5] = temparr[1][2];
			SwitchAreaCoord[i1][6] = temparr[2][0];
			SwitchAreaCoord[i1][7] = temparr[2][1];
			SwitchAreaCoord[i1][8] = temparr[2][2];
			SwitchAreaCoord[i1][9] = temparr[3][0];
			SwitchAreaCoord[i1][10] = temparr[3][1];
			SwitchAreaCoord[i1][11] = temparr[3][2];
		}
}

void SwitchedAreaCoordInfor_2_tx_coordmsgFrame(double(*SwitchAreaCoord)[12], uint8 *tx_coordMsg, int i1)
{
	tx_coordMsg[DATA_FRAME_COORD_AREA_NUM_IDX] = MODE_ANCHOR;
	tx_coordMsg[DATA_FRAME_COORD_AREA_NUM_IDX + 1] = i1;
	int c = 0;
	c = i1 * 2 - 2;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_ID_IDX] = c;
	tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_ID_IDX] = c + 1;
	tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_ID_IDX] = c + 2;
	tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_ID_IDX] = c + 3;
	double coord_res = 0;
	int temp2 = 0;
	coord_res = SwitchAreaCoord[i1][0];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = SwitchAreaCoord[i1][1];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = SwitchAreaCoord[i1][2];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	////////////////////////
	coord_res = SwitchAreaCoord[i1][3];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = SwitchAreaCoord[i1][4];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = SwitchAreaCoord[i1][5];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	///////////////////////
	coord_res = SwitchAreaCoord[i1][6];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = SwitchAreaCoord[i1][7];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX + 2] = temp2 & 0x0000ff;
	}

	coord_res = SwitchAreaCoord[i1][8];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = SwitchAreaCoord[i1][9];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = SwitchAreaCoord[i1][10];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	coord_res = SwitchAreaCoord[i1][11];
	if (coord_res >= 0)
	{
		temp2 = (int)(coord_res * 10000);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX] = (temp2 & 0xff0000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX + 2] = temp2 & 0x0000ff;
	}
	else
	{
		temp2 = ((int)(coord_res * 10000))*(-1);
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX] = ((temp2 & 0xff0000) | 0x800000) >> 16;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		tx_coordMsg[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX + 2] = temp2 & 0x0000ff;
	}
}

void FrameInfor_2_UnderCellOneCoordResultArr(uint8 *rxbuffer, double(*Undercellonecoord)[3])
{
	//int num3 = (int)(rxbuffer[DATA_FRAME_COORD_AREA_NUM_IDX + 1]);
	int tmp1 = 0;
	double dist_tmp1 = 0;
	char a[1];
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[0][0] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_X_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[0][0] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[0][1] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Y_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[0][1] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[0][2] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FIRST_ANCHOR_Z_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[0][2] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[1][0] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_X_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[1][0] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[1][1] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Y_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[1][1] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[1][2] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_SECOND_ANCHOR_Z_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[1][2] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[2][0] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_X_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[2][0] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[2][1] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Y_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[2][1] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[2][2] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_THIRD_ANCHOR_Z_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[2][2] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[3][0] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_X_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[3][0] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[3][1] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Y_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[3][1] = (dist_tmp1*(-1));
	}
	if (((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX]) & 0x80) == 0)
	{
		tmp1 = (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[3][2] = dist_tmp1;
	}
	else
	{
		a[0] = ((rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX]) & 0x7F);
		tmp1 = (a[0] << 16) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_SWITCH_CELL_RESULT_FOURTH_ANCHOR_Z_ID_IDX + 2]);
		dist_tmp1 = (double)(tmp1 / 10000.0);
		Undercellonecoord[3][2] = (dist_tmp1*(-1));
	}


}

void CoordSwitchResultTransmitFrom_A0_To_OtherCellMasteranchor(double(*switchsreacoord)[12], int *Flag, int areasum, double(*underCellOneCoord)[3])
{
	if ((*Flag) == 1)
	{
	  if (Slot_data.AncNum == 0)
		{
			int i1 = 0;
			sleep(5);
			for (i1 = 2; i1 <= areasum; i1++)
			{
				SwitchedAreaCoordInfor_2_tx_coordmsgFrame(switchsreacoord, tx_coordmsg, i1);
				int h2 = 20;
				uint8 bb = 2;
				while (h2)
				{
					MessageSet(Slot_data.PanId, MODE_ANCHOR, bb, MAC_COOEST_AREACOORDRESP, tx_coordmsg);
//					int i6;
//					for (i6 = 0; i6<56; i6++)
//					{
//						char tmpa[56];
//						sprintf(tmpa, "%x", tx_coordmsg[i6]);
//						TcpTx(tmpa, 56);
//						sleep_ms(50);
//					}
					dwt_writetxdata(sizeof(tx_coordmsg), tx_coordmsg, 0);
					dwt_writetxfctrl(sizeof(tx_coordmsg), 0);
					dwt_starttx(DWT_START_TX_IMMEDIATE);
					while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
					{
					}
					if (status_reg&SYS_STATUS_TXFRS)
					{
						//			    	sprintf(dist_str,"%x%x%x",tx_msg[DATA_FRAME_DIST_IDX],tx_msg[DATA_FRAME_DIST_IDX+1],tx_msg[DATA_FRAME_DIST_IDX+2]);
						//			    	TcpTx(dist_str,15);
						//			    	Sleep(50);
						h2--;
					}
					else
					{
						//TcpTx("发送失败！",10);
					}
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
					sleep_ms(500);//200
				}
				sleep(30);

			}
		}
		else
		{
			int MYAreaID = ((Slot_data.AncNum) + 2) / 2;
			int j2=1;
			while (j2)//循环接收并转发来自main_anchor_ID+2的area_coord_infor,直至收到来自area_sum*2-2的信息转发完成后退出
			{
				uint32 frame_len2;
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{
				};
				if (status_reg & SYS_STATUS_RXFCG)
				{
					frame_len2 = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len2 <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len2, 0);
					}
					if (CheckMessage(MAC_COOEST_AREACOORDRESP))
					{
						sleep(12);//5s
						if (rx_buffer[DATA_FRAME_COORD_AREA_NUM_IDX + 1] == MYAreaID)
						{
							FrameInfor_2_UnderCellOneCoordResultArr(rx_buffer, underCellOneCoord);
//							int i5;
//							for (i5 = 0; i5<56; i5++)
//							{
//								char tmpa[56];
//								sprintf(tmpa, "%x", rx_buffer[i5]);
//								TcpTx(tmpa, 56);
//								sleep_ms(50);
//							}
							int i7;
							for (i7 = 0; i7<Slot_data.AncSum; i7++)
							{
								sleep_ms(100);
								sprintf(dist_str, "myCellSwiResID%d x:%4.3f y:%4.3f z:%4.3f", i7, underCellOneCoord[i7][0], underCellOneCoord[i7][1], underCellOneCoord[i7][2]);
								TcpTx(dist_str, 60);
							}
							if((Slot_data.AncNum)==(areasum*2-2))
							{
								j2=0;

							}
						//	sleep(4);

						}
						else
						{
							memcpy(tx_coordmsg, rx_buffer, sizeof(rx_buffer));//最重要的验证此步是否可行
							int h4 = 20;
							uint cc = ((Slot_data.AncNum) + 2);
							while (h4)
							{
								MessageSet(Slot_data.PanId, MODE_ANCHOR, cc, MAC_COOEST_AREACOORDRESP, tx_coordmsg);
								dwt_writetxdata(sizeof(tx_coordmsg), tx_coordmsg, 0);
								dwt_writetxfctrl(sizeof(tx_coordmsg), 0);
								dwt_starttx(DWT_START_TX_IMMEDIATE);
								while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
								{
								}
								if (status_reg&SYS_STATUS_TXFRS)
								{
									//			    	sprintf(dist_str,"%x%x%x",tx_msg[DATA_FRAME_DIST_IDX],tx_msg[DATA_FRAME_DIST_IDX+1],tx_msg[DATA_FRAME_DIST_IDX+2]);
									//			    	TcpTx(dist_str,15);
									//			    	Sleep(50);
									h4--;
								}
								else
								{
									//TcpTx("发送失败！",10);
								}
								dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
								sleep_ms(200);
							}

							if (rx_buffer[DATA_FRAME_COORD_AREA_NUM_IDX + 1] == areasum)
							{
								TcpTx("all_area_coord_have_transmit_succ", 40);
								j2=0;
							}
						}
					}
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

				}
				else
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
					dwt_rxreset();
				}
			}
		}
	}

}


