/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   main loop for the DecaRanging application
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
/* Includes */
#include "compiler.h"
#include <strings.h>
#include "instance.h"
#include "deca_types.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#include "appinit.h"
#include "flash_function.h"
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;




void Delay(uint32_t nCount)
{
  /* Capture the current local time */
  timingdelay = LocalTime + nCount;

  /* wait until the desired delay finish */
  while(timingdelay > LocalTime)
  {
  }
}

/**
  * @brief  Updates the system local time
  * @param  None
  * @retval None
  */
void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;
}

/**
  * @brief  Handles the periodic tasks of the system
  * @param  None
  * @retval None
  */
void System_Periodic_Handle(void)
{
  /* Update the LCD display and the LEDs status */
  /* Manage the IP address setting */
  Display_Periodic_Handle(LocalTime);

  /* LwIP periodic services are done here */
  LwIP_Periodic_Handle(LocalTime);
}

void sleep_ms(unsigned int time_ms)
{
    /* This assumes that the tick has a period of exactly one millisecond. See CLOCKS_PER_SEC define. */
    unsigned long end = portGetTickCount() + time_ms;
    while ((signed long)(portGetTickCount() - end) <= 0)
        ;
}

struct tcp_pcb *connect;

void TcpTx(unsigned char* tmpdata,int len)
{
	connect = Check_TCP_Connect();//婵犵妲呴崑鎾诲箯閻戣姤鍋傞柍鈺佸暞娴溿倝鏌ｉ幇顓熺稇濠㈣鎷�	if(connect != 0)
	{
	 TCP_Client_Send_Data(connect,tmpdata,len);	//闂備礁鎲＄喊宥夊垂鐠虹尨鑰块柛銉墮缁�鏌嶉妷銊︾彧闁绘縿鍊濋弻娑樷枎閹邦剛浼岄梺璺ㄥ枍缁瑥顕ｉ锔藉亱闁割偅绻勯妶锟�	}
	           	        		//Delay_s(0xfffff);											//闁诲海鍋ｉ崐娑㈠春閺嵮屾工闁跨噦鎷�           	        		System_Periodic_Handle();
}
}
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
#define AREASUM 2



static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    0,
    (129 + 8 - 8 + 50)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

Slot Slot_data =
{
	MODE_ANCHOR,  //Tag��Anchor��Monitor
	9,            //Anchor0��1��2��3����
	4,            //��С��Anchor��
	0xCADE,       //PanId
	0x00,         //��������ͨ��16λ��ַ����ǰ������ͨ�����ӵ������Զ����ã������ʼ��
};

//double CoordResult_Arr[6][3] = { { 0,1,2 },{33,-34,35},{-36,37,-38},{339,-310,-311} };
//double CoordResult_Arr[6][3] = { { 10,11,12 },{13,-41,51},{16,71,81},{19,120,131} };
//double CoordResult_Arr[6][3] = { { 90,81,172 },{83,-774,85},{-699,77,788},{-789,170,181} };
//double CoordResult_Arr[6][3] = { { 90,81,172 },{8,-74,-85},{-699,77,788},{9,-170,-181} };
double AreaCoord[(AREASUM+1)][9]={ { 0}};
//double AreaCoord[(AREASUM+1)][9]={ { 0},{ 0},{ 0,1,2,3,4,5,6,7,8},{ -1,-22,-44,-55,-66,-99,-67,-45,99},{1,3,-3,-19,90,45,133,-765,123}};
double CoordResult_Arr[6][3] = { { 0 } };//��С����masteranchorͨ�����յ��ľ�����������ĸ�С������ϵ�������
//double AreaCoord[][9]={ { 0 } };//A0��������A2,A4����������ϵ���ݣ���С����һ��ȫ�ǣ�0��0��0�����꣬�ʲ�����[1][2],�������Ե�һ��С���ĵڶ���anchor��Z���ꣻ
double SwitchAreaCoord[(AREASUM+1)][12]={ { 0 } };//A0ת��������꣬[2][3]�����ڶ���С������2��anchor��X���ꣻ
//double SwitchAreaCoord[(AREASUM+1)][12]={{0,1,2,3,4,5,6,7,8,9,10,11},{0,-1,-2,3,-4,-5,6,-7,-8,9,10,-11},{0,-1,-2,3,4,-5,-6,7,8,-9,-10,11}};
//double SwitchAreaCoord[(AREASUM+1)][12]={{0,1,2,3,4,5,6,7,8,9,10,11},{0,-1,-2,3,-4,-5,6,-7,-8,9,10,-11},{0,1,2,3,4,5,6,7,8,9,10,11},{0,-1,-2,3,-4,-5,6,-7,-8,9,10,-11},{10,111,222,-663,554,445,-336,787,-843,439,110,11}};

double UnderCellOneCoord[4][3]={ { 0 } };//��С��masteranchor�յ�����A0������ϵת�����֡�󣬴洢�ڱ��أ�[0][2]������С����һ��anchor��Z���ꣻ

//double AreaCoord[][9]={ { 0,1,2,3,4,5,6,7,8 } ,{ 10,11,21,31,24,35,567,77,88 }  };
//double AreaCoord[][9]={ { 0,1,2,3,4,5,6,7,8 } ,{ 10,11,21,31,24,35,567,77,88 } ,{ 90,17,62,53,44,55,63,77,88 } };
#pragma GCC optimize ("O3")
int main(void)
{
	peripherals_init();
	spi_peripheral_init();
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
	SPI_ChangeRate(SPI_BaudRatePrescaler_32);
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
	{
		while (1)
		{ };
	}
	SPI_ChangeRate(SPI_BaudRatePrescaler_4);

		    /* Configure DW1000. See NOTE 5 below. */
	dwt_configure(&config,DWT_LOADANTDLY);
	dwt_setleds(1);
	LwIP_Init();
	TCP_Client_Init(TCP_LOCAL_PORT,TCP_SERVER_PORT,TCP_SERVER_IP);
	connect = Check_TCP_Connect();
	sleep_ms(1500);
	TcpTx("Init succ",20);

	sleep_ms(1000);
	uint16 readFlash[2];
	getHalfWordData(0x0803f800, readFlash, 2);
	char str[40];
	sprintf(str,"READ_FLASH**: %X %X",readFlash[0],readFlash[1]);
	TcpTx(str,40);
	Slot_data.AncNum=readFlash[0];
	dwt_settxantennadelay(TX_ANT_DLY);
	dwt_setrxantennadelay(RX_ANT_DLY);
	//while(1){CoordEstablish_Resp();};
	int MainAnchorID=0;
	int AreaNum=1;
	int AreaSum=AREASUM;
	int flag=0;
	while(AreaNum<=AreaSum)
	{
		CoordEstablish(MainAnchorID,&AreaNum,CoordResult_Arr,&flag);
		MainAnchorID=MainAnchorID+2;
	}
	SingleCellCoordResultTransmit2A0(AreaSum,CoordResult_Arr,&flag,AreaCoord);
	if((Slot_data.AncNum)==0)
	{
		CoordSwitch2CellOne(AreaCoord, SwitchAreaCoord,AreaSum);
	}

	CoordSwitchResultTransmitFrom_A0_To_OtherCellMasteranchor(SwitchAreaCoord, &flag, AreaSum, UnderCellOneCoord);



}





