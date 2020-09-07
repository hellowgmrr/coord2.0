
#ifndef _FLASH_FUNCTION_H_
#define _FLASH_FUNCTION_H_
#endif

#include "stm32f10x.h"
#include "deca_types.h"

#define SYSTEM_FLASH_START_ADDR			0x08000000
#define SYSTEM_FLASH_END_ADDR			0x0807FFFF
//�û���ʹ�õ�flash��ַ��Χ
#define	USER_FLASH_START_ADDR			0x08032000								//������ֻ�ܴ��⿪ʼ(200k)
#define USER_FLASH_END_ADDR				USER_FLASH_START_ADDR + 0xdfc4			//�����鲻�ܳ��������ַ(255.941k�������������Ծ�����Ҫ����ô��)

#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || defined (STM32F10X_CL) || defined (STM32F10X_XL)
    #define FLASH_PAGE_SIZE    ((u16)0x0800)				//2k
#else
    #define FLASH_PAGE_SIZE    ((u16)0x0400)				//1k
#endif



int putWordData(uint32 startAddr, uint32 *pBuf, uint32 len);
int putHalfWordData(uint32 startAddr, uint16 *pBuf, uint32 len);
void getWordData(uint32 startAddr, uint32* pBuf, uint32 len);
void getHalfWordData(uint32 startAddr, uint16* pBuf, uint32 len);
