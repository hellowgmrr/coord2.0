/*
* 文件名：
* 描  述：
* 作  者：@Allen
* 时  间：
* 版  权：
*/
#include "appinit.h"

uint8 tx_msg[] = {0x41, 0x88, 0, 0xCB, 0xDE, 'V', 'E', 'W', 'A', 0xD3, 1,1,1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8 tx_coordmsg[] = { 0x41, 0x88, 0, 0xCB, 0xDE, 'V', 'E', 'W', 'A', 0xD8, 1,1,1,  0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0,0};
uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xD1, 0, 0};
uint8 tx_final_msg[] = {0,0,0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8 tx_test_msg[] = {0,1,2,3,4,5,6,7,0xA,0xB,0xC};

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

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
double tof;
double distance;
char dist_str[40] = {0};



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
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

int CheckMessage(int num)
{
	if(Slot_data.PanId == ((rx_buffer[PAN_ID_IDX]<<8)|rx_buffer[PAN_ID_IDX+1]))
	{
		if((rx_buffer[DATA_FRAME_DEST_ADDR_IDX] == Slot_data.AppMode)&&(rx_buffer[DATA_FRAME_DEST_ADDR_IDX+1] == Slot_data.AncNum))
		{
			if(rx_buffer[DATA_FRAME_APP_FCODE_IDX] == num)
			{
				return 1;
			}
		}
	}
	return 0;
}

void MessageSet(unsigned int Panid,uint8 mode,uint8 num,uint8 contbit,uint8 arr[])
{
	arr[0] = 0x41;
	arr[1] = 0x88;
	arr[PAN_ID_IDX] = (Panid&0xff00)>>8;
	arr[PAN_ID_IDX + 1] = (Panid&0x00ff);
	arr[DATA_FRAME_DEST_ADDR_IDX] = mode;
	arr[DATA_FRAME_DEST_ADDR_IDX + 1] = num;
	arr[DATA_FRAME_SRC_ADDR_IDX] = Slot_data.AppMode;
	arr[DATA_FRAME_SRC_ADDR_IDX + 1] = Slot_data.AncNum;
	arr[DATA_FRAME_APP_FCODE_IDX] = contbit;
}

double GetAverage(double arr[],int len)
{
    int i;
    double min = arr[0], max = arr[0],aver = 0;
    for(i = 0; i<len; i++)
    {
        if(arr[i]<=min) min = arr[i];
        if(arr[i]>=max) max = arr[i];
        aver += arr[i];
    }
    //printf("min:%f\nmax:%f\n",min,max);
    aver -= (min + max);
    aver /= (len-2);
    //printf("average:%f",aver);
    return aver;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AreaCoordInfor_2_tx_coordmsgFrame(int main_Anchor_ID,int *area_Num,double (*Result_Arr)[3],uint8 *tx_coordMsg)
{
	tx_coordMsg[DATA_FRAME_COORD_AREA_NUM_IDX] = MODE_ANCHOR;
	tx_coordMsg[DATA_FRAME_COORD_AREA_NUM_IDX + 1] = (*area_Num);
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_ID_IDX] = main_Anchor_ID +1;
	tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_ID_IDX] = main_Anchor_ID + 2;
	tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_ID_IDX] = main_Anchor_ID + 3;
			double coord_res = 0;
			int temp2 = 0;
			coord_res = Result_Arr[1][0];
		    temp2 = (int)(coord_res * 10000);
		    tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX] = (temp2 & 0xff0000) >> 16;
		    tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
		    tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 2] = temp2 & 0x0000ff;
			coord_res = Result_Arr[1][1];
			temp2 = (int)(coord_res * 10000);
			tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX] = (temp2 & 0xff0000) >> 16;
			tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
			tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 2] = temp2 & 0x0000ff;
			coord_res = Result_Arr[1][2];
			temp2 = (int)(coord_res * 10000);
			tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX] = (temp2 & 0xff0000) >> 16;
			tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
			tx_coordMsg[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 2] = temp2 & 0x0000ff;
			////////////////////////
			coord_res = Result_Arr[2][0];
			temp2 = (int)(coord_res * 10000);
			tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX] = (temp2 & 0xff0000) >> 16;
			tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
			tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 2] = temp2 & 0x0000ff;
			coord_res = Result_Arr[2][1];
			temp2 = (int)(coord_res * 10000);
			tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX] = (temp2 & 0xff0000) >> 16;
			tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
			tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 2] = temp2 & 0x0000ff;
			coord_res = Result_Arr[2][2];
			temp2 = (int)(coord_res * 10000);
			tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX] = (temp2 & 0xff0000) >> 16;
			tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
			tx_coordMsg[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 2] = temp2 & 0x0000ff;
			///////////////////////
			coord_res = Result_Arr[3][0];
			temp2 = (int)(coord_res * 10000);
			tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX] = (temp2 & 0xff0000) >> 16;
			tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
			tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 2] = temp2 & 0x0000ff;
			coord_res = Result_Arr[3][1];
			temp2 = (int)(coord_res * 10000);
			tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX] = (temp2 & 0xff0000) >> 16;
			tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
			tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 2] = temp2 & 0x0000ff;
			coord_res = Result_Arr[3][2];
			temp2 = (int)(coord_res * 10000);
			tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX] = (temp2 & 0xff0000) >> 16;
			tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 1] = (temp2 & 0x00ff00) >> 8;
			tx_coordMsg[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 2] = temp2 & 0x0000ff;
}

void FrameInfor_2_AreaCoordArray(uint8 *rxbuffer,double (*areacoord)[9])
{
	int num3 = rxbuffer[DATA_FRAME_COORD_AREA_NUM_IDX + 1];
	int tmp1 = 0;
	double dist_tmp1 = 0;
	tmp1 = (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_X_COORD_IDX + 2]);
	dist_tmp1=(double)(tmp1 / 10000.0);
	areacoord[num3][0] = dist_tmp1;
	tmp1 = (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Y_COORD_IDX + 2]);
	dist_tmp1=(double)(tmp1 / 10000.0);
	areacoord[num3][1] = dist_tmp1;
	tmp1 = (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_SECOND_ANCHOR_Z_COORD_IDX + 2]);
	dist_tmp1=(double)(tmp1 / 10000.0);
	areacoord[num3][2] = dist_tmp1;
	tmp1 = (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_X_COORD_IDX + 2]);
	dist_tmp1=(double)(tmp1 / 10000.0);
	areacoord[num3][3] = dist_tmp1;
	tmp1 = (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Y_COORD_IDX + 2]);
	dist_tmp1=(double)(tmp1 / 10000.0);
	areacoord[num3][4] = dist_tmp1;
	tmp1 = (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_THIRD_ANCHOR_Z_COORD_IDX + 2]);
	dist_tmp1=(double)(tmp1 / 10000.0);
	areacoord[num3][5] = dist_tmp1;
	tmp1 = (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_X_COORD_IDX + 2]);
	dist_tmp1=(double)(tmp1 / 10000.0);
	areacoord[num3][6] = dist_tmp1;
	tmp1 = (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Y_COORD_IDX + 2]);
	dist_tmp1=(double)(tmp1 / 10000.0);
	areacoord[num3][7] = dist_tmp1;
	tmp1 = (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX] << 16) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 1] << 8) | (rxbuffer[DATA_FRAME_AREA_FOURTH_ANCHOR_Z_COORD_IDX + 2]);
	dist_tmp1=(double)(tmp1 / 10000.0);
	areacoord[num3][8] = dist_tmp1;
}

double CoordEstablish_Init(uint8 dest_mode,uint8 dest_num)
{
	int CurrNum = 0,NumOfTest = 10;
	double DistArr[10];
	while(CurrNum < NumOfTest)
    {


		    //static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCB, 0xDE, 'V', 'E', 'W', 'A', 0xD3, 1,1,1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		    MessageSet(Slot_data.PanId,dest_mode,dest_num,MAC_COOEST_START,tx_msg);
			dwt_writetxdata(sizeof(tx_msg),tx_msg,0);
			dwt_writetxfctrl(sizeof(tx_msg),0);
			dwt_starttx(DWT_START_TX_IMMEDIATE);
			while(!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
			{
			}
			if(status_reg&SYS_STATUS_TXFRS)
			{
							//TcpTx("发送成功！",10);
			}
			else {
							//TcpTx("发送失败！",10);
			}
			dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_TX);


        dwt_setrxtimeout(10000);

        /* Activate reception immediately. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };

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
                if(CurrNum == NumOfTest - 1)tx_msg[ALL_MSG_SN_IDX] = 0x10;
                MessageSet(Slot_data.PanId,dest_mode,dest_num,MAC_COOEST_RESPONSE,tx_msg);
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
                { };

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
	return GetAverage(DistArr,10);
}

int CoordEstablish_Resp(void)
{
	  static uint8 cnt,addr;
//	  while(1){
	  uint32 frame_len;
	  int resp_enable = 0;
	  uint8 dest_mode,dest_num;
	  dwt_setrxtimeout(10000);
	  dwt_rxenable(DWT_START_RX_IMMEDIATE);
	  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
	  		        { };
	  if(status_reg & SYS_STATUS_RXFCG)
	  {
		  frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
		  if (frame_len <= RX_BUF_LEN)
		  {
		  	  dwt_readrxdata(rx_buffer, frame_len, 0);
		  }
		  if(CheckMessage(MAC_COOEST_START))
		  {
			  resp_enable = 1;
			  dest_mode = rx_buffer[DATA_FRAME_SRC_ADDR_IDX];
			  dest_num = rx_buffer[DATA_FRAME_SRC_ADDR_IDX + 1];
		  }
		  dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXFCG);
	  }
	  else {
		  dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
		  dwt_rxreset();
	  }


		    /* Loop forever initiating ranging exchanges. */
		    if(resp_enable)
		    {
		    	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
		        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS + 700);
		        /* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
		        tx_poll_msg[ALL_MSG_SN_IDX] = 0;
                MessageSet(Slot_data.PanId,dest_mode,dest_num,MAC_COOEST_POLL,tx_poll_msg);
		        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
		        dwt_writetxfctrl(sizeof(tx_poll_msg), 0); /* Zero offset in TX buffer, ranging. */

		        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
		         * set by dwt_setrxaftertxdelay() has elapsed. */
		        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

		        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
		        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
		        { };

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
		                MessageSet(Slot_data.PanId,dest_mode,dest_num,MAC_COOEST_FINAL,tx_final_msg);
		                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
		                dwt_writetxfctrl(sizeof(tx_final_msg), 0); /* Zero offset in TX buffer, ranging. */
		                ret = dwt_starttx(DWT_START_TX_DELAYED);

		                /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
		                if (ret == DWT_SUCCESS)
		                {
		                    /* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
		                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
		                    { };

		                    /* Clear TXFRS event. */
		                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

		                    /* Increment frame sequence number after transmission of the final message (modulo 256). */
		                    frame_seq_nb++;
		                }

		                if((rx_buffer[ALL_MSG_SN_IDX] == 0x10)&&(rx_buffer[DATA_FRAME_SRC_ADDR_IDX+1]) == Slot_data.AncNum-1)
		                {int p=20;
		                while(p--)
		                {
		                	MessageSet(Slot_data.PanId,dest_mode,dest_num,MAC_COOEST_FINAL,tx_final_msg);
		                			                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
		                			                dwt_writetxfctrl(sizeof(tx_final_msg), 0); /* Zero offset in TX buffer, ranging. */
		                			                ret = dwt_starttx(DWT_START_TX_DELAYED);

		                			                /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
		                			                if (ret == DWT_SUCCESS)
		                			                {
		                			                    /* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
		                			                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
		                			                    { };

		                			                    /* Clear TXFRS event. */
		                			                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

		                			                    /* Increment frame sequence number after transmission of the final message (modulo 256). */
		                			                    //frame_seq_nb++;
		                			                }
		                			                sleep_ms(200);



		                }

		                	return 1;}
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

int CoordEstablish(int main_anchor_ID,int *area_num,int area_sum)
{
	int mode = Slot_data.AppMode;
	int mode_num = Slot_data.AncNum;
	if((mode == MODE_ANCHOR)&&(mode_num == main_anchor_ID))
	{
		double dist_arr[6][6] = {{0}};
		double result_arr[6][3] = {{0}};
		int j;
		int rec_num=9;//此处是为了让收到的多组同样的距离数据只显示一次
		for(j = main_anchor_ID+3; j>main_anchor_ID ; j--)
		{
             double dist = CoordEstablish_Init(MODE_ANCHOR,j);
             sprintf(dist_str,"DIST%d %d:%4.3f m",main_anchor_ID,j,dist);
             TcpTx(dist_str,15);
             sleep_ms(50);
             dist_arr[0][j-main_anchor_ID] = dist;
             dist_arr[j-main_anchor_ID][0] = dist;
		}
		while(1)
		{
			dwt_setrxtimeout(0);
			uint32 frame_len;
			uint8 num1,num2;
			double dist_tmp;
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
				{ };
			if(status_reg & SYS_STATUS_RXFCG)
			{
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len <= RX_BUF_LEN)
				{
				dwt_readrxdata(rx_buffer, frame_len, 0);
				}
				if(CheckMessage(MAC_COOEST_DIST))
				{
					num1 = rx_buffer[DATA_FRAME_SRC_ADDR_IDX+1];
					num2 = rx_buffer[DATA_FRAME_DIST_DEST_ADDR_IDX+1];
					int tmp = (rx_buffer[DATA_FRAME_DIST_IDX]<<16)|(rx_buffer[DATA_FRAME_DIST_IDX+1]<<8)|(rx_buffer[DATA_FRAME_DIST_IDX+2]);
					dist_tmp = (double)(tmp/10000.0);
					if(rec_num!=num2){
					sprintf(dist_str,"DIST%d%d:%4.3f m",num1,num2,dist_tmp);
					TcpTx(dist_str,16);}
					dist_arr[num1-main_anchor_ID][num2-main_anchor_ID] = dist_tmp;
					dist_arr[num2-main_anchor_ID][num1-main_anchor_ID] = dist_tmp;
					if((num2 == main_anchor_ID +Slot_data.AncSum-1)&&(num1 == main_anchor_ID +Slot_data.AncSum-2))
						break;
					rec_num=num2;
				}
				dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXFCG);
			}
			else
			{
				dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_RX_ERR|SYS_STATUS_ALL_RX_TO);
				dwt_rxreset();

			}
		}
		sleep_ms(200);
		TcpTx("Break",10);
		main_mds(dist_arr,Slot_data.AncSum,result_arr);
		int i;
		for(i = 0; i<Slot_data.AncSum; i++)
		{
			sleep_ms(500);
			sprintf(dist_str,"x:%4.3f y:%4.3f z:%4.3f",result_arr[i][0],result_arr[i][1],result_arr[i][2]);
			TcpTx(dist_str,40);
		}

		//////////////////////////////////////start transport coord information to A0
		if(main_anchor_ID==(area_sum*2-2))
		{
			AreaCoordInfor_2_tx_coordmsgFrame(main_anchor_ID,&area_num,result_arr,tx_coordmsg);
			int h2=10;
			while(h2--)
			{
			    MessageSet(Slot_data.PanId,MODE_ANCHOR,main_anchor_ID-2,MAC_COOEST_AREACOORD,tx_coordmsg);
				dwt_writetxdata(sizeof(tx_coordmsg),tx_coordmsg,0);
			    dwt_writetxfctrl(sizeof(tx_coordmsg),0);
			    dwt_starttx(DWT_START_TX_IMMEDIATE);
				while(!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
				{}
				if(status_reg&SYS_STATUS_TXFRS)
				{
			//			    	sprintf(dist_str,"%x%x%x",tx_msg[DATA_FRAME_DIST_IDX],tx_msg[DATA_FRAME_DIST_IDX+1],tx_msg[DATA_FRAME_DIST_IDX+2]);
			//			    	TcpTx(dist_str,15);
			//			    	Sleep(50);
				}
			    else
			    {
						    					    							//TcpTx("发送失败！",10);
				}
			    dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_TX);
			    sleep_ms(800);
			}
		/////////////////////////////////////////////send finish


		}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		else if(main_anchor_ID==0)///////一直接收
		{	while(1)
			{
			dwt_setrxtimeout(0);
			uint32 frame_len1;
			//uint8 num3;//area number
			double AreaCoord[][9]={{0}};
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
			{
			};
			if (status_reg & SYS_STATUS_RXFCG)
			{
				frame_len1 = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
				if (frame_len1 <= RX_BUF_LEN)
				{
					dwt_readrxdata(rx_buffer, frame_len1, 0);
				}
				if (CheckMessage(MAC_COOEST_AREACOORD))
				{
					FrameInfor_2_AreaCoordArray(rx_buffer,AreaCoord);
//此处需显示接收到的小区坐标系信息
					int i1=0;
					int j1=0;
					for(i1 = 0; i1<area_sum; i1++)
					{
						sleep_ms(500);
						sprintf(dist_str,"AREA:%d Sx:%4.3f Sy:%4.3f Sz:%4.3f\n Tx:%4.3f Ty:%4.3f Tz:%4.3f\n Fx:%4.3f Fy:%4.3f Fz:%4.3f\n",i,result_arr[i1][0],result_arr[i1][1],result_arr[i1][2],result_arr[i1][3],result_arr[i1][4],result_arr[i1][5],result_arr[i1][6],result_arr[i1][7],result_arr[i1][8]);
						TcpTx(dist_str,200);
					}
					if (rx_buffer[DATA_FRAME_COORD_AREA_NUM_IDX + 1] == area_sum)
					{
						TcpTx("all_area_coord_have_received_succ",40);
						break;

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
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		else
		{
			AreaCoordInfor_2_tx_coordmsgFrame(main_anchor_ID,&area_num,result_arr,tx_coordmsg);
			int h3=10;
			while (h3--)
			{
				MessageSet(Slot_data.PanId, MODE_ANCHOR, main_anchor_ID-2, MAC_COOEST_AREACOORD, tx_coordmsg);
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
				}
				else
				{
					//TcpTx("发送失败！",10);
				}
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
				sleep_ms(800);
			}
			while(1)//循环接收并转发来自main_anchor_ID+2的area_coord_infor,直至收到来自area_sum*2-2的信息转发完成后退出
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
					if (CheckMessage(MAC_COOEST_AREACOORD))
					{
						memcpy(tx_coordmsg,rx_buffer,sizeof(rx_buffer));
						int h4 = 10;
						while (h4--)
						{
							MessageSet(Slot_data.PanId, MODE_ANCHOR, main_anchor_ID - 2, MAC_COOEST_AREACOORD, tx_coordmsg);
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
							}
							else
							{
								//TcpTx("发送失败！",10);
							}
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
							sleep_ms(800);
						}

						if (rx_buffer[DATA_FRAME_COORD_AREA_NUM_IDX + 1] == area_sum)
						{
							TcpTx("all_area_coord_have_received_succ", 40);
							break;

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


		while(1){}

	}


	else if((mode == MODE_ANCHOR)&&((mode_num ==(main_anchor_ID+1))||(mode_num ==(main_anchor_ID+2))||(mode_num ==(main_anchor_ID+3))))
	{
		int init_enable = 0;
		while(!init_enable)
		{
		init_enable = CoordEstablish_Resp();
		}
		if(init_enable)
		{
			int j;
			double dist;
		    for(j =main_anchor_ID+3; j>Slot_data.AncNum ; j--)
			{
			    dist = CoordEstablish_Init(MODE_ANCHOR,j);
			    sprintf(dist_str,"DIST%d%d:%f.4 m",Slot_data.AncNum,j,dist);
			    TcpTx(dist_str,15);
			    sleep_ms(10000);
			    int hh=10;
			    while(hh--)
			    {

                tx_msg[DATA_FRAME_DIST_DEST_ADDR_IDX] = MODE_ANCHOR;
                tx_msg[DATA_FRAME_DIST_DEST_ADDR_IDX + 1] = j;
                int tmp1 = (int)(dist*10000);
                tx_msg[DATA_FRAME_DIST_IDX] = (tmp1&0xff0000)>>16;
                tx_msg[DATA_FRAME_DIST_IDX+1] = (tmp1&0x00ff00)>>8;
                tx_msg[DATA_FRAME_DIST_IDX+2] = tmp1&0x0000ff;
                MessageSet(Slot_data.PanId,MODE_ANCHOR,main_anchor_ID,MAC_COOEST_DIST,tx_msg);
			    dwt_writetxdata(sizeof(tx_msg),tx_msg,0);
			    dwt_writetxfctrl(sizeof(tx_msg),0);
			    dwt_starttx(DWT_START_TX_IMMEDIATE);
			    while(!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS))
			    {}
			    if(status_reg&SYS_STATUS_TXFRS)
			    {
//			    	sprintf(dist_str,"%x%x%x",tx_msg[DATA_FRAME_DIST_IDX],tx_msg[DATA_FRAME_DIST_IDX+1],tx_msg[DATA_FRAME_DIST_IDX+2]);
//			    	TcpTx(dist_str,15);
//			    	Sleep(50);
			    }
			    else {
			    					    							//TcpTx("发送失败！",10);
			    }
			    dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_ALL_TX);
			    sleep_ms(800);
                }
			}
		}
		(*area_num)++;
	}
	else
	{
		(*area_num)++;
		sleep(60);
	}

}


