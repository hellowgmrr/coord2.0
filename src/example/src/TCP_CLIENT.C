#include "main.h"

unsigned char connect_flag = 0;
/*******************延时函数****************************************************/
/*******************************************************************************/
void Delay_s(unsigned long ulVal) /* 利用循环产生一定的延时 */
{
	while ( --ulVal != 0 );
}



err_t TCP_Client_Send_Data(struct tcp_pcb *cpcb,unsigned char *buff,unsigned int length)
{
	err_t err;

	err = tcp_write(cpcb,buff,length,TCP_WRITE_FLAG_COPY);	//发送数据
	tcp_output(cpcb);
	//tcp_close(cpcb);				//发送完数据关闭连接,根据具体情况选择使用	
	return err;					
}                  /***************TCP客户端发送数据函数************************************/

struct tcp_pcb *Check_TCP_Connect(void)
{
	struct tcp_pcb *cpcb = 0;
	connect_flag = 0;
	for(cpcb = tcp_active_pcbs;cpcb != NULL; cpcb = cpcb->next)
	{
		if(cpcb->local_port == TCP_LOCAL_PORT && cpcb->remote_port == TCP_SERVER_PORT)		//如果TCP_LOCAL_PORT端口指定的连接没有断开
//		if(cpcb -> state == ESTABLISHED)
		{
			connect_flag = 1;  						//连接标志
			break;							   	
		}
	}

	if(connect_flag == 0)  							// TCP_LOCAL_PORT指定的端口未连接或已断开
	{
		TCP_Client_Init(TCP_LOCAL_PORT,TCP_SERVER_PORT,TCP_SERVER_IP); //重新连接
		cpcb = 0;
	}
	return cpcb;
}                                                    /*****检查连接*****/

err_t TCP_Connected(void *arg,struct tcp_pcb *pcb,err_t err)
{
	//tcp_client_pcb = pcb;
	return ERR_OK;
}                   /*****这是一个回调函数，当TCP客户端请求的连接建立时被调用*****/


/*******************************************************************************/
err_t  TCP_Client_Recv(void *arg, struct tcp_pcb *pcb,struct pbuf *p,err_t err)
{

	if(p != NULL)
	{
		tcp_recved(pcb, p->tot_len);				//获取数据长度 tot_len：tcp数据块的长度
		tcp_write(pcb,p->payload,p->tot_len,TCP_WRITE_FLAG_COPY);
		tcp_output(pcb);			    
	}
	else	 										//如果服务器断开连接，则客户端也应断开
	{
		tcp_close(pcb);
	}
	pbuf_free(p); 				
	err = ERR_OK;

	return err;	
}               /***tcp客户端接收数据回调函数***这是一个回调函数，当TCP服务器发来数据时调用***/

 /****************tcp客户端初始化***************/
void TCP_Client_Init(u16_t local_port,u16_t remote_port,unsigned char a,unsigned char b,unsigned char c,unsigned char d)
{

	struct ip_addr ipaddr;
	struct tcp_pcb *tcp_client_pcb;
	err_t err;
	IP4_ADDR(&ipaddr,a,b,c,d);
	tcp_client_pcb = tcp_new();
	if (!tcp_client_pcb)
	{
		return ;
	}	
	err = tcp_bind(tcp_client_pcb,IP_ADDR_ANY,local_port);
    if(err != ERR_OK)
	{
		return ;
	}
	tcp_connect(tcp_client_pcb,&ipaddr,remote_port,TCP_Connected);
	tcp_recv(tcp_client_pcb,TCP_Client_Recv);
}



