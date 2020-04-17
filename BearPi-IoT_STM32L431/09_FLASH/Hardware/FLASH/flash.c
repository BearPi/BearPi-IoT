/******************************************************************************	
�����[ �������������[ �����������������[ �������������[�����[     �����[   �����[�������������[ 
�����U�����X�T�T�T�����[�^�T�T�����X�T�T�a�����X�T�T�T�T�a�����U     �����U   �����U�����X�T�T�����[
�����U�����U   �����U   �����U   �����U     �����U     �����U   �����U�������������X�a
�����U�����U   �����U   �����U   �����U     �����U     �����U   �����U�����X�T�T�����[
�����U�^�������������X�a   �����U   �^�������������[���������������[�^�������������X�a�������������X�a
�^�T�a �^�T�T�T�T�T�a    �^�T�a    �^�T�T�T�T�T�a�^�T�T�T�T�T�T�a �^�T�T�T�T�T�a �^�T�T�T�T�T�a 
*******************************************************************************	
* �ļ�����: flash.c
* ��    ��: ���������ֲ�
* ��    ��: V2.0
* ��д����: 2018-5-1
* ��    ��: flash��д����
*
* ˵    ��: �������������������ֲ�BearPi������ʹ��
*
* ��    ��: https://iot-club.taobao.com
* ��    ̳: http://www.iot-club.cn��http://www.iotclub.net
*******************************************************************************/
#include "flash.h"
#include "usart.h"
#include "stdio.h"
/*************�������е�����������4�ֽڶ����******************/

/***************************************************************
* ��������: Flash_GetBank
* ˵    ��: ��ô����ַ����Bank
* ��    ��: faddr,Flash��ַ
* �� �� ֵ: Bankֵ
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
* ��������: Flash_GetPage
* ˵    ��: ��ô����ַ����Page
* ��    ��: faddr,Flash��ַ
* �� �� ֵ: Bankֵ
***************************************************************/
uint32_t Flash_GetPage(uint32_t faddr) 
{
	if (faddr < (FLASH_BASE + FLASH_BANK_SIZE)) 
		return (faddr - FLASH_BASE) / FLASH_PAGE_SIZE;
	else 
		return (faddr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
}

/***************************************************************
* ��������: Flash_ReadOneWord
* ˵    ��: ��ָ����ַ�ж�ȡһ��32λ������
* ��    ��: faddr,Flash��ַ
* �� �� ֵ: Flash��ַ�ڵ�һ��32λ������
***************************************************************/
uint32_t Flash_ReadOneWord(uint32_t faddr)
{
	return *(__IO uint32_t *)faddr;
}

/***************************************************************
* ��������: Flash_ErasePages
* ˵    ��: ����ҳ
* ��    ��: faddr,��ʼ��ַ
*						fdataNum,Ҫ������������ʵ����Ҳ����ҳ����
* �� �� ֵ: ��
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
* ��������: Flash_WriteOneWord
* ˵    ��: ��ָ����ַ��д��һ��32λ������
* ��    ��: faddr,Flash��ַ
*						fdata32,Flash��ַ�ڵ�һ��32λ������
* �� �� ֵ: дFLASH���ص�״̬
***************************************************************/
uint8_t Flash_WriteOneWord(uint32_t faddr,uint32_t fdata32)
{
	if(faddr<FLASH_BASE)										//�Ƿ���ַ
		return HAL_ERROR;	
	uint8_t FLASH_STATUS;
	uint64_t fdata64;
	HAL_FLASH_Unlock();
	if(faddr<FLASH_ADDR_MAX)										//д�����ݵ�ַС��STM32L431RC��flash��ַ���ֵ
	{
		fdata64 = fdata32|0xffffffff00000000uL;
		FLASH_STATUS = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,faddr,fdata64);//д������
	} 
	HAL_FLASH_Lock();
	return FLASH_STATUS;
}

/***************************************************************
* ��������: Flash_WriteDoubleWord
* ˵    ��: ��ָ����ַ��д��һ��˫�֣�64λ������
* ��    ��: faddr,Flash��ַ
*						fdata64,Flash��ַ�ڵ�һ��˫�֣�64λ������
* �� �� ֵ: дFLASH���ص�״̬
***************************************************************/
uint8_t Flash_WriteDoubleWord(uint32_t faddr,uint64_t fdata64)
{
	if(faddr<FLASH_BASE)										//�Ƿ���ַ
		return HAL_ERROR;
	uint8_t FLASH_STATUS;
	HAL_FLASH_Unlock();											//����FLASH
	if(faddr<FLASH_ADDR_MAX)								//д�����ݵ�ַС��STM32L431RC��flash��ַ���ֵ
	{
		FLASH_STATUS = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,faddr,fdata64);//д������
	} 
	HAL_FLASH_Lock();												//����
	return FLASH_STATUS;
}

/***************************************************************
* ��������: Flash_Write32
* ˵    ��: ��ָ����ַ��д��fdataNum��32λ������
* ��    ��: faddr,Flash��ʼ��ַ
*						fdata,Flash��ַ�ڵ�32λ������ָ��
*						fdata32Num,Ҫд����֣�32λ����������
* �� �� ֵ: ��
***************************************************************/
void Flash_Write32(uint32_t faddr,uint32_t *fdata32, uint32_t fdata32Num)
{
	uint64_t fdata64;
	HAL_FLASH_Unlock();																		//����FLASH
	if(faddr<FLASH_ADDR_MAX)																	//д�����ݵ�ַС��STM32L431RC��flash��ַ���ֵ
	{
		for (uint32_t i = 0; i < fdata32Num / 2; i++) 
		{
			fdata64 = *(uint64_t*)fdata32;
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, faddr, fdata64) != HAL_OK) 			
			{																									//������ִ���
				HAL_FLASH_Lock(); 															//����FLASH
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
			{																									//������ִ���
				HAL_FLASH_Lock(); 															//����FLASH
				return;
			}
		}
	}
	HAL_FLASH_Lock();
}
/***************************************************************
* ��������: Flash_Write64
* ˵    ��: ��ָ����ַ��д��fdataNum��64λ������
* ��    ��: faddr,Flash��ʼ��ַ
*						fdata,Flash��ַ�ڵ�64λ������ָ��
*						fdata64Num��Ҫд����֣�64λ����������
* �� �� ֵ: ��
***************************************************************/
void Flash_Write64(uint32_t faddr,uint64_t *fdata64, uint32_t fdata64Num)
{
	HAL_FLASH_Unlock();															//����FLASH
	if(faddr<FLASH_ADDR_MAX)														//д�����ݵ�ַС��STM32L431RC��flash��ַ���ֵ
	{
		for (uint32_t i = 0; i < fdata64Num; i++) 
		{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, faddr, *fdata64) != HAL_OK) 
			{																						//���д���쳣
				HAL_FLASH_Lock(); 												//����FLASH
				return;
			}
			faddr += 8;
			fdata64 += 1;
		}
	}
	HAL_FLASH_Lock();															//����FLASH
}

/***************************************************************
* ��������: Flash_Read32
* ˵    ��: ��ָ����ַ��ʼ����ָ�����ȵ�����
* ��    ��: ReadAddr,��ʼ��ַ
*						*pBuffer,����ָ��
*						fdata32Num���֣�32λ����
* �� �� ֵ: ��
***************************************************************/
void Flash_Read32(uint32_t faddr,uint32_t *fdata32,uint32_t fdata32Num)
{
	uint32_t i;
	if(faddr>FLASH_ADDR_MAX)										//��ַС��STM32L431RC��flash��ַ���ֵ
		return;
	for(i=0;i<fdata32Num;i++)
	{
		fdata32[i] = Flash_ReadOneWord(faddr);	//��ȡ4���ֽ�
		faddr+=4;
	}
}

/***************************************************************
* ��������: FlashRead
* ˵    ��: ��ָ����ַ��ʼ����ָ�����ȵ�����
* ��    ��: ReadAddr,��ʼ��ַ
*						*pBuffer������ָ��
*						fdata8Num,�֣�8λ����
* �� �� ֵ: ��
***************************************************************/
void Flash_ReadBytes(uint32_t faddr,uint8_t *fdata8,uint32_t fdata8Num)
{
	uint32_t i;
	if(faddr>FLASH_ADDR_MAX)										//��ַС��STM32L431RC��flash��ַ���ֵ
		return;
	for(i=0;i<fdata8Num;i++)
	{
		fdata8[i] = *(__IO uint8_t *)faddr;		//��ȡ1���ֽ�
		faddr+=1;
	}
}

/***************************************************************
* ��������: Flash_WriteReadTest
*	˵		��: Flash����д�롢��ȡ����
* ��    ��: fAdress,flash��ַ
*						*wData,��flash��ַ��д�������
*						wDataLen,д�����ݵĳ���
*						*rData,��flash��ַ�����������
* �� �� ֵ: ERROR,ʧ��
*						SUCCESS,�ɹ�
***************************************************************/
uint8_t Flash_WriteReadTest(uint32_t fAdress, uint8_t *wData, uint32_t wDataLen, uint8_t *rData)
{
	if(*wData == NULL)
	{
		printf("д������Ϊ�ա���\r\n");
		return ERROR;
	}
	//���Ҫд������ݵĳ��ȣ���Ϊд�������Ϊ32λ�ģ�����ĸ�wDataΪһ�飬�����ĸ��Զ��Ჹ0x00��
	uint16_t strWriteLen = (wDataLen%4)==0?(wDataLen/4):(wDataLen/4 + 1);
	printf("Flash_Write32:%s\r\n", wData);
	Flash_ErasePages(fAdress, 1);																		//����flash	
	Flash_Write32(fAdress, (uint32_t *)wData, strWriteLen);					//д������
	Flash_ReadBytes(fAdress, rData, wDataLen);											//�������ݣ�����д����پͶ�������
	printf("Flash_ReadBytes:%s\r\n",rData);
	return SUCCESS;
}
/**********************END OF FILE**********************/
