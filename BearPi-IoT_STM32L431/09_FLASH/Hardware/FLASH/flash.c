/******************************************************************************	
██╗ ██████╗ ████████╗ ██████╗██╗     ██╗   ██╗██████╗ 
██║██╔═══██╗╚══██╔══╝██╔════╝██║     ██║   ██║██╔══██╗
██║██║   ██║   ██║   ██║     ██║     ██║   ██║██████╔╝
██║██║   ██║   ██║   ██║     ██║     ██║   ██║██╔══██╗
██║╚██████╔╝   ██║   ╚██████╗███████╗╚██████╔╝██████╔╝
╚═╝ ╚═════╝    ╚═╝    ╚═════╝╚══════╝ ╚═════╝ ╚═════╝ 
*******************************************************************************	
* 文件名称: flash.c
* 作    者: 物联网俱乐部
* 版    本: V2.0
* 编写日期: 2018-5-1
* 功    能: flash读写驱动
*
* 说    明: 本例程配套物联网俱乐部BearPi开发板使用
*
* 淘    宝: https://iot-club.taobao.com
* 论    坛: http://www.iot-club.cn或http://www.iotclub.net
*******************************************************************************/
#include "flash.h"
#include "usart.h"
#include "stdio.h"
/*************下面所有的数都必须是4字节对齐的******************/

/***************************************************************
* 函数名称: Flash_GetBank
* 说    明: 获得传入地址所在Bank
* 参    数: faddr,Flash地址
* 返 回 值: Bank值
***************************************************************/
uint32_t Flash_GetBank(uint32_t faddr) 
{
	if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_MEM_MODE_Pos) == 0) 
	{
		if (faddr < (FLASH_BASE + FLASH_BANK_SIZE)) 
			return FLASH_BANK_1;
		else 
			return FLASH_BANK_1;
	} 
	else 
	{
		if (faddr < (FLASH_BASE + FLASH_BANK_SIZE)) // bank swap
			return FLASH_BANK_1;
		else 
			return FLASH_BANK_1;
	}
}

/***************************************************************
* 函数名称: Flash_GetPage
* 说    明: 获得传入地址所在Page
* 参    数: faddr,Flash地址
* 返 回 值: Bank值
***************************************************************/
uint32_t Flash_GetPage(uint32_t faddr) 
{
	if (faddr < (FLASH_BASE + FLASH_BANK_SIZE)) 
		return (faddr - FLASH_BASE) / FLASH_PAGE_SIZE;
	else 
		return (faddr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
}

/***************************************************************
* 函数名称: Flash_ReadOneWord
* 说    明: 从指定地址中读取一个32位的数据
* 参    数: faddr,Flash地址
* 返 回 值: Flash地址内的一个32位的数据
***************************************************************/
uint32_t Flash_ReadOneWord(uint32_t faddr)
{
	return *(__IO uint32_t *)faddr;
}

/***************************************************************
* 函数名称: Flash_ErasePages
* 说    明: 擦除页
* 参    数: faddr,起始地址
*						fdataNum,要擦除的字数，实际上也是整页擦除
* 返 回 值: 无
***************************************************************/
void Flash_ErasePages(uint32_t faddr,uint32_t fdataNum)
{
	if(fdataNum == 0||faddr>FLASH_ADDR_MAX)
	{
		return;
	}
	uint32_t PageError = 0;
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef FLASH_EraseInitSturcture;
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	
	FLASH_EraseInitSturcture.TypeErase = FLASH_TYPEERASE_PAGES;
	FLASH_EraseInitSturcture.Banks = Flash_GetBank(faddr);
	FLASH_EraseInitSturcture.Page = Flash_GetPage(faddr);
	FLASH_EraseInitSturcture.NbPages = Flash_GetPage(faddr + 4 * fdataNum - 1) - FLASH_EraseInitSturcture.Page + 1;
	
	if(HAL_FLASHEx_Erase(&FLASH_EraseInitSturcture,&PageError) != HAL_OK) 
	{
		HAL_FLASH_Lock();
		return;
	}
}

/***************************************************************
* 函数名称: Flash_WriteOneWord
* 说    明: 向指定地址中写入一个32位的数据
* 参    数: faddr,Flash地址
*						fdata32,Flash地址内的一个32位的数据
* 返 回 值: 写FLASH返回的状态
***************************************************************/
uint8_t Flash_WriteOneWord(uint32_t faddr,uint32_t fdata32)
{
	if(faddr<FLASH_BASE)										//非法地址
		return HAL_ERROR;	
	uint8_t FLASH_STATUS;
	uint64_t fdata64;
	HAL_FLASH_Unlock();
	if(faddr<FLASH_ADDR_MAX)										//写入数据地址小于STM32L431RC的flash地址最大值
	{
		fdata64 = fdata32|0xffffffff00000000uL;
		FLASH_STATUS = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,faddr,fdata64);//写入数据
	} 
	HAL_FLASH_Lock();
	return FLASH_STATUS;
}

/***************************************************************
* 函数名称: Flash_WriteDoubleWord
* 说    明: 向指定地址中写入一个双字（64位）数据
* 参    数: faddr,Flash地址
*						fdata64,Flash地址内的一个双字（64位）数据
* 返 回 值: 写FLASH返回的状态
***************************************************************/
uint8_t Flash_WriteDoubleWord(uint32_t faddr,uint64_t fdata64)
{
	if(faddr<FLASH_BASE)										//非法地址
		return HAL_ERROR;
	uint8_t FLASH_STATUS;
	HAL_FLASH_Unlock();											//解锁FLASH
	if(faddr<FLASH_ADDR_MAX)								//写入数据地址小于STM32L431RC的flash地址最大值
	{
		FLASH_STATUS = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,faddr,fdata64);//写入数据
	} 
	HAL_FLASH_Lock();												//上锁
	return FLASH_STATUS;
}

/***************************************************************
* 函数名称: Flash_Write32
* 说    明: 向指定地址中写入fdataNum个32位的数据
* 参    数: faddr,Flash起始地址
*						fdata,Flash地址内的32位的数据指针
*						fdata32Num,要写入的字（32位）数的数量
* 返 回 值: 无
***************************************************************/
void Flash_Write32(uint32_t faddr,uint32_t *fdata32, uint32_t fdata32Num)
{
	uint64_t fdata64;
	HAL_FLASH_Unlock();																		//解锁FLASH
	if(faddr<FLASH_ADDR_MAX)																	//写入数据地址小于STM32L431RC的flash地址最大值
	{
		for (uint32_t i = 0; i < fdata32Num / 2; i++) 
		{
			fdata64 = *(uint64_t*)fdata32;
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, faddr, fdata64) != HAL_OK) 			
			{																									//如果出现错误
				HAL_FLASH_Lock(); 															//锁定FLASH
				return;
			}
			faddr += 8;
			fdata32 += 2;
		}
		if ((fdata32Num & 0x01) == 1) 
		{
			uint64_t fdata64 = *(uint64_t*)faddr;
			fdata64 = (fdata64 & 0xffffffff00000000uL) | (*fdata32);
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, faddr, fdata64) != HAL_OK) 			
			{																									//如果出现错误
				HAL_FLASH_Lock(); 															//锁定FLASH
				return;
			}
		}
	}
	HAL_FLASH_Lock();
}
/***************************************************************
* 函数名称: Flash_Write64
* 说    明: 向指定地址中写入fdataNum个64位的数据
* 参    数: faddr,Flash起始地址
*						fdata,Flash地址内的64位的数据指针
*						fdata64Num，要写入的字（64位）数的数量
* 返 回 值: 无
***************************************************************/
void Flash_Write64(uint32_t faddr,uint64_t *fdata64, uint32_t fdata64Num)
{
	HAL_FLASH_Unlock();															//解锁FLASH
	if(faddr<FLASH_ADDR_MAX)														//写入数据地址小于STM32L431RC的flash地址最大值
	{
		for (uint32_t i = 0; i < fdata64Num; i++) 
		{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, faddr, *fdata64) != HAL_OK) 
			{																						//如果写入异常
				HAL_FLASH_Lock(); 												//上锁FLASH
				return;
			}
			faddr += 8;
			fdata64 += 1;
		}
	}
	HAL_FLASH_Lock();															//上锁FLASH
}

/***************************************************************
* 函数名称: Flash_Read32
* 说    明: 从指定地址开始读出指定长度的数据
* 参    数: ReadAddr,起始地址
*						*pBuffer,数据指针
*						fdata32Num，字（32位）数
* 返 回 值: 无
***************************************************************/
void Flash_Read32(uint32_t faddr,uint32_t *fdata32,uint32_t fdata32Num)
{
	uint32_t i;
	if(faddr>FLASH_ADDR_MAX)										//地址小于STM32L431RC的flash地址最大值
		return;
	for(i=0;i<fdata32Num;i++)
	{
		fdata32[i] = Flash_ReadOneWord(faddr);	//读取4个字节
		faddr+=4;
	}
}

/***************************************************************
* 函数名称: FlashRead
* 说    明: 从指定地址开始读出指定长度的数据
* 参    数: ReadAddr,起始地址
*						*pBuffer，数据指针
*						fdata8Num,字（8位）数
* 返 回 值: 无
***************************************************************/
void Flash_ReadBytes(uint32_t faddr,uint8_t *fdata8,uint32_t fdata8Num)
{
	uint32_t i;
	if(faddr>FLASH_ADDR_MAX)										//地址小于STM32L431RC的flash地址最大值
		return;
	for(i=0;i<fdata8Num;i++)
	{
		fdata8[i] = *(__IO uint8_t *)faddr;		//读取1个字节
		faddr+=1;
	}
}

/***************************************************************
* 函数名称: Flash_WriteReadTest
*	说		明: Flash数据写入、读取测试
* 参    数: fAdress,flash地址
*						*wData,向flash地址里写入的数据
*						wDataLen,写入数据的长度
*						*rData,从flash地址里读出的数据
* 返 回 值: ERROR,失败
*						SUCCESS,成功
***************************************************************/
uint8_t Flash_WriteReadTest(uint32_t fAdress, uint8_t *wData, uint32_t wDataLen, uint8_t *rData)
{
	if(*wData == NULL)
	{
		printf("写入数据为空……\r\n");
		return ERROR;
	}
	//获得要写入的数据的长度，因为写入的数据为32位的，因此四个wData为一组，不满四个自动会补0x00。
	uint16_t strWriteLen = (wDataLen%4)==0?(wDataLen/4):(wDataLen/4 + 1);
	printf("Flash_Write32:%s\r\n", wData);
	Flash_ErasePages(fAdress, 1);																		//擦除flash	
	Flash_Write32(fAdress, (uint32_t *)wData, strWriteLen);					//写入数据
	Flash_ReadBytes(fAdress, rData, wDataLen);											//读出数据，这里写入多少就读出多少
	printf("Flash_ReadBytes:%s\r\n",rData);
	return SUCCESS;
}
/**********************END OF FILE**********************/
