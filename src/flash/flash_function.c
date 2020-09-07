#include "flash_function.h"
#include "deca_types.h"




static int putFlashData(uint32 startAddr, void *pBuf, uint32 len, uint8 size);
static int checkTargetAddr(uint32 startAddr, uint32 len);
static int computeForEraseCnt(uint32 startAddr, uint32 len, uint8 size);
static int putFlashHalfWord(uint32 startAddr, uint16 *pBuf, uint32 len);
static int putFlashWord(uint32 startAddr, uint32 *pBuf, uint32 len);

static int putFlashData(uint32 startAddr, void *pBuf, uint32 len, uint8 size)
{
	uint32 i;
	uint8 eraseCnt;

	if (!checkTargetAddr(startAddr, len))
	{
		return 0;
	}
	eraseCnt = computeForEraseCnt(startAddr, len, size);
	FLASH_Unlock();

	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_BSY | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

	for (i = 0; i < eraseCnt; i++)
	{
		if(FLASH_ErasePage(startAddr + FLASH_PAGE_SIZE * i) != FLASH_COMPLETE)	//å¯¹é½æ“¦é™¤ï¼Œå³ä½¿ä¸æ˜¯é¦–åœ°å€ä¹Ÿä¼šè½¬æˆå¯¹åº”çš„é¦–åœ°å€è¿›è¡Œæ“¦é™¤
		{
			FLASH_Lock();
			return 0;
		}
	}

	switch(size)
	{
		case 2:
			if (!putFlashHalfWord(startAddr, pBuf, len))
			{
				return 0;
			}
			break;
		case 4:
			if (!putFlashWord(startAddr, pBuf, len))
			{
				return 0;
			}
			break;
		default:
			break;
	}

	FLASH_Lock();

	return 1;
}
static int checkTargetAddr(uint32 startAddr, uint32 len)
{
	if (len <= 0)
	{
		return 0;
	}
	//if (startAddr < SYSTEM_FLASH_START_ADDR || startAddr >= USER_FLASH_END_ADDR)		//·ÀÖ¹Ð´ÈëµÄµØÖ·²»ÔÚ·¶Î§ÄÚ
	//{
	//	return 0;
	//}

	if (startAddr + len >= USER_FLASH_END_ADDR)
	{
		return 0;
	}
	if (startAddr % 2 == 1)		//·ÀÖ¹Ð´ÈëµÄµØÖ·ÎªÆæÊý
	{
		return 0;
	}
	return 1;
}

static int computeForEraseCnt(uint32 startAddr, uint32 len, uint8 size)
{
	uint32 end;
	uint8 eraseCnt;
	end = startAddr + len * size;

	if ((end - SYSTEM_FLASH_START_ADDR) / FLASH_PAGE_SIZE != (startAddr - SYSTEM_FLASH_START_ADDR) / FLASH_PAGE_SIZE)
	{
		eraseCnt = (end - startAddr) / FLASH_PAGE_SIZE + 2;
	}
	else
	{
		eraseCnt = (end - startAddr) / FLASH_PAGE_SIZE + 1;
	}
	return eraseCnt;
}
static int putFlashHalfWord(uint32 startAddr, uint16 *pBuf, uint32 len)
{
	int i;
	for (i = 0; i < len; i ++)
	{
		if (FLASH_ProgramHalfWord(startAddr + i * 2, pBuf[i]) != FLASH_COMPLETE)
		{
			FLASH_Lock();
			return 0;
		}
	}
	return 1;
}

static int putFlashWord(uint32 startAddr, uint32 *pBuf, uint32 len)
{
	int i;
	for (i = 0; i < len; i ++)
	{
		if (FLASH_ProgramWord(startAddr + i * 4, pBuf[i]) != FLASH_COMPLETE)
		{
			FLASH_Lock();
			return 0;
		}
	}
	return 1;
}
int putWordData(uint32 startAddr, uint32 *pBuf, uint32 len)
{
	return putFlashData(startAddr, pBuf, len, 4);
}
int putHalfWordData(uint32 startAddr, uint16 *pBuf, uint32 len)
{
	return putFlashData(startAddr, pBuf, len, 2);
}
void getWordData(uint32 startAddr, uint32* pBuf, uint32 len)
{
	uint32 i;
	for (i = 0; i < len; i++)
	{
		pBuf[i] = *((uint32*)startAddr + i);
	}
}
void getHalfWordData(uint32 startAddr, uint16* pBuf, uint32 len)
{
	uint32 i;
	for (i = 0; i < len; i++)
	{
		pBuf[i] = *((uint16*)startAddr + i);
	}
}
