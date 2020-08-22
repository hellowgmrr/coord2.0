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
	connect = Check_TCP_Connect();//å©µçŠµî•½å¦²å‘´å´‘éŽ¾è¯²ç®¯é–»æˆ£å§¤é‹å‚žæŸéˆºä½¸æšžå¨´æº¿å€éŒï½‰å¹‡é¡“ç†ºç¨‡æ¿ ãˆ£î†éŽ·ï¿½	if(connect != 0)
	{
	 TCP_Client_Send_Data(connect,tmpdata,len);	//é—‚å‚™ç¤éŽ²ï¼„å–Šå®¥å¤Šåž‚é è™¹å°¨é‘°å—æŸ›éŠ‰î†¼å¢®ç¼ï¿½î™éŒå¶‰å¦·éŠŠï¸¾å½§é—ç»˜ç¸¿éŠæ¿‹å¼»å¨‘æ¨·æžŽé–¹é‚¦å‰›æµ¼å²„æ¢ºç’ºã„¥æžç¼î†½ç‘¥é¡•ï½‰î”•é””è—‰äº±é—å‰²å…ç»»å‹¯å¦¶é”Ÿï¿½	}
	           	        		//Delay_s(0xfffff);											//é—è¯²æµ·é‹ï½‰å´å¨‘ãˆ æ˜¥é–ºåµ®å±¾å·¥é—è·¨å™¦éŽ·ï¿½           	        		System_Periodic_Handle();
}
}
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436



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
	MODE_ANCHOR,  //Tag¡¢Anchor¡¢Monitor
	5,            //Anchor0¡¢1¡¢2¡¢3¡­¡­
	4,            //µ¥Ð¡ÇøAnchorÊý
	0xCADE,       //PanId
	0x00,         //±¾µØÎÞÏßÍ¨ÐÅ16Î»µØÖ·£¬µ±Ç°´úÂëÖÐÍ¨¹ý°å×ÓµÄÉí·Ý×Ô¶¯ÅäÖÃ£¬ÎÞÐè³õÊ¼»¯
};

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
	int AreaSum=2;
	while(AreaNum<=AreaSum){
	CoordEstablish(MainAnchorID,&AreaNum,AreaSum);
	MainAnchorID=MainAnchorID+2;
	}





}





