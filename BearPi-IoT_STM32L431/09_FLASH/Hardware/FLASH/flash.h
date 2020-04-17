/******************************************************************************	
�����[ �������������[ �����������������[ �������������[�����[     �����[   �����[�������������[ 
�����U�����X�T�T�T�����[�^�T�T�����X�T�T�a�����X�T�T�T�T�a�����U     �����U   �����U�����X�T�T�����[
�����U�����U   �����U   �����U   �����U     �����U     �����U   �����U�������������X�a
�����U�����U   �����U   �����U   �����U     �����U     �����U   �����U�����X�T�T�����[
�����U�^�������������X�a   �����U   �^�������������[���������������[�^�������������X�a�������������X�a
�^�T�a �^�T�T�T�T�T�a    �^�T�a    �^�T�T�T�T�T�a�^�T�T�T�T�T�T�a �^�T�T�T�T�T�a �^�T�T�T�T�T�a 
*******************************************************************************	
* �ļ�����: flash.h
* ��    ��: ���������ֲ�
* ��    ��: V2.0
* ��д����: 2018-5-1
* ��    ��: flash��д����ͷ�ļ�
*
* ˵    ��: �������������������ֲ�BearPi������ʹ��
*
* ��    ��: https://iot-club.taobao.com
* ��    ̳: http://www.iot-club.cn��http://www.iotclub.net
*******************************************************************************/
#ifndef __FLASH_H_
#define	__FLASH_H_

#include "stm32l4xx_hal.h"

//------------------------�궨��-----------------------------// 
#define FLASH_OP_ADDR 0x0803f800//;0x08007000			//STM32 FLASH��������ʼ��ַ
#define FLASH_ADDR_MAX 0x08040000			//STM32 FLASH������ַ�����ֵ

/**************************************************************/
/**************         FLASH�����ӿ�         *****************/
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
