/******************************************************************************	
██╗ ██████╗ ████████╗ ██████╗██╗     ██╗   ██╗██████╗ 
██║██╔═══██╗╚══██╔══╝██╔════╝██║     ██║   ██║██╔══██╗
██║██║   ██║   ██║   ██║     ██║     ██║   ██║██████╔╝
██║██║   ██║   ██║   ██║     ██║     ██║   ██║██╔══██╗
██║╚██████╔╝   ██║   ╚██████╗███████╗╚██████╔╝██████╔╝
╚═╝ ╚═════╝    ╚═╝    ╚═════╝╚══════╝ ╚═════╝ ╚═════╝ 
*******************************************************************************	
* 文件名称: flash.h
* 作    者: 物联网俱乐部
* 版    本: V2.0
* 编写日期: 2018-5-1
* 功    能: flash读写驱动头文件
*
* 说    明: 本例程配套物联网俱乐部BearPi开发板使用
*
* 淘    宝: https://iot-club.taobao.com
* 论    坛: http://www.iot-club.cn或http://www.iotclub.net
*******************************************************************************/
#ifndef __FLASH_H_
#define	__FLASH_H_

#include "stm32l4xx_hal.h"

//------------------------宏定义-----------------------------// 
#define FLASH_OP_ADDR 0x0803f800//;0x08007000			//STM32 FLASH操作的起始地址
#define FLASH_ADDR_MAX 0x08040000			//STM32 FLASH操作地址的最大值

/**************************************************************/
/**************         FLASH操作接口         *****************/
/**************************************************************/
uint32_t Flash_ReadOneWord(uint32_t faddr);
void Flash_ErasePages(uint32_t faddr,uint32_t fdataNum);
uint8_t Flash_WriteOneWord(uint32_t faddr,uint32_t fdata32);
uint8_t Flash_WriteDoubleWord(uint32_t faddr,uint64_t fdata64);
void Flash_Write32(uint32_t faddr,uint32_t *fdata32, uint32_t fdata32Num);
void Flash_Write64(uint32_t faddr,uint64_t *fdata64, uint32_t fdata64Num);
void Flash_Read32(uint32_t faddr,uint32_t *fdata32,uint32_t fdata32Num);
void Flash_ReadBytes(uint32_t faddr,uint8_t *fdata8,uint32_t fdata8Num);
uint8_t Flash_WriteReadTest(uint32_t fAdress, uint8_t *wData, uint32_t wDataLen, uint8_t *rData);
#endif
/**************************END OF FILE*************************/
