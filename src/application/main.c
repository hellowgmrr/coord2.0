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
	connect = Check_TCP_Connect();//濠电姷顣藉Σ鍛村磻閹捐绠柣鎴ｅГ閸嬪倿鏌嶉埡浣告殲濞存嚎鍊濋弻锝夊箛椤撶喓绋囨繝銏ｎ啇閹凤拷	if(connect != 0)
	{
	 TCP_Client_Send_Data(connect,tmpdata,len);	//闂傚倷绀侀幉锛勫枈瀹ュ鍨傞悹铏瑰皑閼板潡鏌涢妷顔煎缂侊拷顧侀弻宥夊Ψ閵婏妇褰ч梺缁樼缚閸婃繈寮诲☉妯锋瀻闁归偊鍓涙导宀勬⒑鐠恒劌鏋嶇紒顔界懃椤曪綁顢曢敂钘変罕闂佸壊鍋呯换鍕Χ閿燂拷	}
	           	        		//Delay_s(0xfffff);											//闂佽娴烽崑锝夊磹濞戙垹鏄ラ柡宓本宸ラ梺璺ㄥ櫐閹凤拷           	        		System_Periodic_Handle();
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
	MODE_ANCHOR,  //Tag、Anchor、Monitor
	2,            //Anchor0、1、2、3……
	4,            //单小区Anchor数
	0xCADE,       //PanId
	0x00,         //本地无线通信16位地址，当前代码中通过板子的身份自动配置，无需初始化
};

//double CoordResult_Arr[6][3] = { { 0,1,2 },{33,-34,35},{-36,37,-38},{339,-310,-311} };
//double CoordResult_Arr[6][3] = { { 10,11,12 },{13,-41,51},{16,71,81},{19,120,131} };
//double CoordResult_Arr[6][3] = { { 90,81,172 },{83,-774,85},{-699,77,788},{-789,170,181} };
//double CoordResult_Arr[6][3] = { { 90,81,172 },{8,-74,-85},{-699,77,788},{9,-170,-181} };
double AreaCoord[(AREASUM+1)][9]={ { 0}};
//double AreaCoord[(AREASUM+1)][9]={ { 0},{ 0},{ 0,1,2,3,4,5,6,7,8},{ -1,-22,-44,-55,-66,-99,-67,-45,99},{1,3,-3,-19,90,45,133,-765,123}};
double CoordResult_Arr[6][3] = { { 0 } };//单小区的masteranchor通过接收到的距离数据算出的该小区坐标系建立结果
//double AreaCoord[][9]={ { 0 } };//A0接收来自A2,A4……的坐标系数据，各小区第一个全是（0，0，0）坐标，故不传，[1][2],代表来自第一个小区的第二个anchor的Z坐标；
double SwitchAreaCoord[(AREASUM+1)][12]={ { 0 } };//A0转换后的坐标，[2][3]代表第二个小区，第2个anchor的X坐标；
//double SwitchAreaCoord[(AREASUM+1)][12]={{0,1,2,3,4,5,6,7,8,9,10,11},{0,-1,-2,3,-4,-5,6,-7,-8,9,10,-11},{0,-1,-2,3,4,-5,-6,7,8,-9,-10,11}};
//double SwitchAreaCoord[(AREASUM+1)][12]={{0,1,2,3,4,5,6,7,8,9,10,11},{0,-1,-2,3,-4,-5,6,-7,-8,9,10,-11},{0,1,2,3,4,5,6,7,8,9,10,11},{0,-1,-2,3,-4,-5,6,-7,-8,9,10,-11},{10,111,222,-663,554,445,-336,787,-843,439,110,11}};

double UnderCellOneCoord[4][3]={ { 0 } };//各小区masteranchor收到来自A0的坐标系转换结果帧后，存储在本地，[0][2]代表本小区第一个anchor的Z坐标；

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





