/** \file max30102.cpp ******************************************************
*
* Project: MAXREFDES117#
* Filename: max30102.cpp
* Description: This module is an embedded controller driver for the MAX30102
*
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*  Modified original MAXIM source code on: 13.01.2019
*		Author: Mateusz Salamon
*		www.msalamon.pl
*		mateusz@msalamon.pl
*	Code is modified to work with STM32 HAL libraries.
*
*	Website: https://msalamon.pl/palec-mi-pulsuje-pulsometr-max30102-pod-kontrola-stm32/
*	GitHub:  https://github.com/lamik/MAX30102_STM32_HAL
*
*/
#include "main.h"


#include "MAX30102/MAX30102.h"
#include "MAX30102/algorithm.h"

#define I2C_TIMEOUT	1

I2C_HandleTypeDef *i2c_max30102;

volatile uint32_t IrBuffer[MAX30102_BUFFER_LENGTH]; //IR LED sensor data
volatile uint32_t RedBuffer[MAX30102_BUFFER_LENGTH];    //Red LED sensor data
volatile uint32_t BufferHead;
volatile uint32_t BufferTail;
volatile uint32_t CollectedSamples;
volatile uint8_t IsFingerOnScreen;
int32_t Sp02Value;
int8_t Sp02IsValid;
int32_t HeartRate;
int8_t IsHrValid;

typedef enum
{
	MAX30102_STATE_BEGIN,
	MAX30102_STATE_CALIBRATE,
	MAX30102_STATE_CALCULATE_HR,
	MAX30102_STATE_COLLECT_NEXT_PORTION
}MAX30102_STATE;

MAX30102_STATE StateMachine;

MAX30102_STATUS Max30102_WriteReg(uint8_t uch_addr, uint8_t uch_data)
{
	if(HAL_I2C_Mem_Write(i2c_max30102, MAX30102_ADDRESS, uch_addr, 1, &uch_data, 1, I2C_TIMEOUT) == HAL_OK)
		return MAX30102_OK;
	return MAX30102_ERROR;
}

MAX30102_STATUS Max30102_ReadReg(uint8_t uch_addr, uint8_t *puch_data)
{
	if(HAL_I2C_Mem_Read(i2c_max30102, MAX30102_ADDRESS, uch_addr, 1, puch_data, 1, I2C_TIMEOUT) == HAL_OK)
		return MAX30102_OK;
	return MAX30102_ERROR;
}

MAX30102_STATUS Max30102_WriteRegisterBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
//2021. 06. 10 CHANG
// Set 1  specific bit of Register

	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(Register, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(1<<Bit);
	tmp |= (Value&0x01)<<Bit;
	if(MAX30102_OK != Max30102_WriteReg(Register, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}
MAX30102_STATUS Max30102_ReadFifo_MODIFY(volatile uint32_t *pun_red_led, volatile uint32_t *pun_ir_led, volatile uint8_t LED_PW)
{
	uint32_t un_temp;
	*pun_red_led=0;
	*pun_ir_led=0;
	uint8_t ach_i2c_data[6];
	uint8_t word=0;

	if(HAL_I2C_Mem_Read(i2c_max30102, MAX30102_ADDRESS, REG_FIFO_DATA, 1, ach_i2c_data, 6, I2C_TIMEOUT) != HAL_OK)
	{
		return MAX30102_ERROR;
	}
	un_temp=(unsigned char) ach_i2c_data[0];
	un_temp<<=16;
	*pun_red_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[1];
	un_temp<<=8;
	*pun_red_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[2];
	*pun_red_led+=un_temp;

	un_temp=(unsigned char) ach_i2c_data[3];
	un_temp<<=16;
	*pun_ir_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[4];
	un_temp<<=8;
	*pun_ir_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[5];
	*pun_ir_led+=un_temp;

	if(LED_PW==SPO2_PULSE_WIDTH_69)
	{
		*pun_red_led=*pun_red_led>>3;
		*pun_red_led&=0x007FFF;
	}
	else if(LED_PW==SPO2_PULSE_WIDTH_118)
	{
		*pun_red_led=*pun_red_led>>2;
		*pun_red_led&=0x00FFFF;
	}
	else if(LED_PW==SPO2_PULSE_WIDTH_215)
	{
		*pun_red_led=*pun_red_led>>1;
		*pun_red_led&=0x01FFFF;
	}
	else
	{
		*pun_red_led&=0x03FFFF;
	}
	/** SETTING **/
	//SPO2_PULSE_WIDTH_118 ">>2"
	//SPO2_PULSE_WIDTH_215 ">>1"
	//SPO2_PULSE_WIDTH_411 NONE
	//*pun_red_led=*pun_red_led>>2;
	//*pun_red_led&=0x00FFFF;  //Mask MSB [23:18]
	//*pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]


	return MAX30102_OK;
}

MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_red_led, volatile uint32_t *pun_ir_led)
{
	uint32_t un_temp;
	*pun_red_led=0;
	*pun_ir_led=0;
	uint8_t ach_i2c_data[6];
	uint8_t word=0;

	if(HAL_I2C_Mem_Read(i2c_max30102, MAX30102_ADDRESS, REG_FIFO_DATA, 1, ach_i2c_data, 6, I2C_TIMEOUT) != HAL_OK)
	{
		return MAX30102_ERROR;
	}
	un_temp=(unsigned char) ach_i2c_data[0];
	un_temp<<=16;
	*pun_red_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[1];
	un_temp<<=8;
	*pun_red_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[2];
	*pun_red_led+=un_temp;

	un_temp=(unsigned char) ach_i2c_data[3];
	un_temp<<=16;
	*pun_ir_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[4];
	un_temp<<=8;
	*pun_ir_led+=un_temp;
	un_temp=(unsigned char) ach_i2c_data[5];
	*pun_ir_led+=un_temp;
	/** SETTING **/
	//SPO2_PULSE_WIDTH_118 ">>2"
	//SPO2_PULSE_WIDTH_215 ">>1"
	//SPO2_PULSE_WIDTH_411 NONE
	*pun_red_led=*pun_red_led>>2;
	*pun_red_led&=0x00FFFF;  //Mask MSB [23:18]
	*pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]


	return MAX30102_OK;
}

//
//	Interrupts
//
MAX30102_STATUS Max30102_SetIntAlmostFullEnabled(uint8_t Enable)
{
	return Max30102_WriteRegisterBit(REG_INTR_ENABLE_1, INT_A_FULL_BIT, Enable);
}

MAX30102_STATUS Max30102_SetIntFifoDataReadyEnabled(uint8_t Enable)
{

	return Max30102_WriteRegisterBit(REG_INTR_ENABLE_1, INT_PPG_RDY_BIT, Enable);
}

MAX30102_STATUS Max30102_SetIntAmbientLightCancelationOvfEnabled(uint8_t Enable)
{

	return Max30102_WriteRegisterBit(REG_INTR_ENABLE_2, INT_ALC_OVF_BIT, Enable);
}
#ifdef MAX30102_USE_INTERNAL_TEMPERATURE
MAX30102_STATUS Max30102_SetIntInternalTemperatureReadyEnabled(uint8_t Enable)
{

	return Max30102_WriteRegisterBit(REG_INTR_ENABLE_2, INT_DIE_TEMP_RDY_BIT, Enable);
}
#endif
MAX30102_STATUS Max30102_ReadInterruptStatus(uint8_t *Status)
{
	uint8_t tmp;
	*Status = 0;

	if(MAX30102_OK != Max30102_ReadReg(REG_INTR_STATUS_1, &tmp))
		return MAX30102_ERROR;
	*Status |= tmp & 0xE1; // 3 highest bits
#ifdef MAX30102_USE_INTERNAL_TEMPERATURE
	if(MAX30102_OK != Max30102_ReadReg(REG_INTR_STATUS_2, &tmp))
		return MAX30102_ERROR;
	*Status |= tmp & 0x02;
#endif
	return MAX30102_OK;
}

void Max30102_InterruptCallback(void)
{
	uint8_t Status;
	while(MAX30102_OK != Max30102_ReadInterruptStatus(&Status));

	// Almost Full FIFO Interrupt handle
	if(Status & (1<<INT_A_FULL_BIT))
	{
// 2021.06.10 CHANG
		for(uint8_t i = 0; i < MAX30102_FIFO_ALMOST_FULL_SAMPLES; i++)
//		for(uint8_t i = 0; i < (MAX30102_FIFO_MAX -MAX30102_FIFO_ALMOST_FULL_SAMPLES); i++)
		{
			while(MAX30102_OK != Max30102_ReadFifo((RedBuffer+BufferHead), (IrBuffer+BufferHead)));

			if(IsFingerOnScreen)
			{
				if(IrBuffer[BufferHead] < MAX30102_IR_VALUE_FINGER_OUT_SENSOR) IsFingerOnScreen = 0;
			}
			else
			{
				if(IrBuffer[BufferHead] > MAX30102_IR_VALUE_FINGER_ON_SENSOR) IsFingerOnScreen = 1;
			}
			BufferHead = (BufferHead + 1) % MAX30102_BUFFER_LENGTH;
			CollectedSamples++;
		}
	}

	// New FIFO Data Ready Interrupt handle
	if(Status & (1<<INT_PPG_RDY_BIT))
	{
		while(MAX30102_OK != Max30102_ReadFifo((RedBuffer+BufferHead), (IrBuffer+BufferHead)));
		if(IsFingerOnScreen)
		{
			if(IrBuffer[BufferHead] < MAX30102_IR_VALUE_FINGER_OUT_SENSOR) IsFingerOnScreen = 0;
		}
		else
		{
			if(IrBuffer[BufferHead] > MAX30102_IR_VALUE_FINGER_ON_SENSOR) IsFingerOnScreen = 1;
		}
		BufferHead = (BufferHead + 1) % MAX30102_BUFFER_LENGTH;
		CollectedSamples++;
	}

	//  Ambient Light Cancellation Overflow Interrupt handle
	if(Status & (1<<INT_ALC_OVF_BIT))
	{

	}

	// Power Ready Interrupt handle
	if(Status & (1<<INT_PWR_RDY_BIT))
	{
	}
#ifdef MAX30102_USE_INTERNAL_TEMPERATURE
	// Internal Temperature Ready Interrupt handle
	if(Status & (1<<INT_DIE_TEMP_RDY_BIT))
	{

	}
#endif
}

//
//	FIFO Configuration
//
MAX30102_STATUS Max30102_FifoWritePointer(uint8_t Address)
{
	if(MAX30102_OK != Max30102_WriteReg(REG_FIFO_WR_PTR,(Address & 0x1F)))  //FIFO_WR_PTR[4:0] : B0 ~B4
			return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoOverflowCounter(uint8_t Address)
{
	if(MAX30102_OK != Max30102_WriteReg(REG_OVF_COUNTER,(Address & 0x1F)))  //OVF_COUNTER[4:0]
			return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoReadPointer(uint8_t Address)
{
	if(MAX30102_OK != Max30102_WriteReg(REG_FIFO_RD_PTR,(Address & 0x1F)))  //FIFO_RD_PTR[4:0]
			return MAX30102_ERROR;
	return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoSampleAveraging(uint8_t Value)
{
// FIFO Configuration(0x08)
// SMP_AVE[2:0] --> B7, B6, B5  (0xE0)

	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_FIFO_CONFIG, &tmp))
		return MAX30102_ERROR;

	tmp &= ~(0xE0);
	tmp |= (Value&0x07)<<5;
//	tmp &= ~(0x07);
//	tmp |= (Value&0x07)<<5;
	if(MAX30102_OK != Max30102_WriteReg(REG_FIFO_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

MAX30102_STATUS Max30102_FifoRolloverEnable(uint8_t Enable)
{
	return Max30102_WriteRegisterBit(REG_FIFO_CONFIG, FIFO_CONF_FIFO_ROLLOVER_EN_BIT, (Enable & 0x01));
}

MAX30102_STATUS Max30102_FifoAlmostFullValue(uint8_t Value) 	// 17
{
// FIFO Configuration(0x08)
// FIFO_A_FULL[3:0] --> B3, B2, B1, B0  (0x0F)

//2021.06.10 CHANG
	if(Value < 17) Value = 17;
	if(Value > 32) Value = 32;
	Value = 32 - Value;
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_FIFO_CONFIG, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(0x0F);
	tmp |= (Value & 0x0F);
	if(MAX30102_OK != Max30102_WriteReg(REG_FIFO_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}
//
//	Mode Configuration
//
MAX30102_STATUS Max30102_ShutdownMode(uint8_t Enable)
{
	return Max30102_WriteRegisterBit(REG_MODE_CONFIG, MODE_SHDN_BIT, (Enable & 0x01));
}

MAX30102_STATUS Max30102_Reset(void)
{
	uint8_t tmp = 0xFF;
    if(MAX30102_OK != Max30102_WriteReg(REG_MODE_CONFIG,0x40))		//0x09 레지스터에 0x40을 write함.
        return MAX30102_ERROR;
    do
    {
    	if(MAX30102_OK != Max30102_ReadReg(REG_MODE_CONFIG, &tmp))
    		return MAX30102_ERROR;
    } while(tmp & (1<<6));											// 0x09 레지스터의 B6값이 자동으로 0으로 될때까지 대기 (즉 RESET bit는 자동으로 시스템 리셋 이후 0으로 clear될 때까지 대기)

    return MAX30102_OK;
}

MAX30102_STATUS Max30102_SetMode(uint8_t Mode)	//OK
{
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_MODE_CONFIG, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(0x07);
	tmp |= (Mode & 0x07);
	if(MAX30102_OK != Max30102_WriteReg(REG_MODE_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}
//
//	SpO2 Configuration
//
MAX30102_STATUS Max30102_SpO2AdcRange(uint8_t Value)	//OK
{
//2021.06.10 CHANG
// SPO2 Config. Reg(0x0A)
// SPO2_ADC_RGE[1:0] ---> B6, B5
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_SPO2_CONFIG, &tmp))
		return MAX30102_ERROR;
//	tmp &= ~(0x03);
//	tmp |= ((Value & 0x03) << 5);
	tmp &= ~(0x60);
	tmp |= ((Value & 0x03) <<5);
	if(MAX30102_OK != Max30102_WriteReg(REG_SPO2_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

MAX30102_STATUS Max30102_SpO2SampleRate(uint8_t Value)	//OK
{
//2021.06.10 CHANG
// SPO2 Configuration register (0x0A)
// SPO2_SR[2:0] ---> B4, B3, B2
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_SPO2_CONFIG, &tmp))
		return MAX30102_ERROR;
//	tmp &= ~(0x07);
//	tmp |= ((Value & 0x07) << 2);
	tmp &= ~ (0x1C);
	tmp |= ((Value & 0x07)<<2);
	if(MAX30102_OK != Max30102_WriteReg(REG_SPO2_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

MAX30102_STATUS Max30102_SpO2LedPulseWidth(uint8_t Value)	// OK
{
	uint8_t tmp;
	if(MAX30102_OK != Max30102_ReadReg(REG_SPO2_CONFIG, &tmp))
		return MAX30102_ERROR;
	tmp &= ~(0x03);
	tmp |= (Value & 0x03);
	if(MAX30102_OK != Max30102_WriteReg(REG_SPO2_CONFIG, tmp))
		return MAX30102_ERROR;

	return MAX30102_OK;
}

//
//	LEDs Pulse Amplitute Configuration
//	LED Current = Value * 0.2 mA
//  Led1 means RED (Register address = 0x0C)
MAX30102_STATUS Max30102_Led1PulseAmplitude(uint8_t Value)	// OK
{
	if(MAX30102_OK != Max30102_WriteReg(REG_LED1_PA, Value))
		return MAX30102_ERROR;
	return MAX30102_OK;
}

////  Led2 means IR (Register address = 0x0D)
MAX30102_STATUS Max30102_Led2PulseAmplitude(uint8_t Value)	// OK
{
	if(MAX30102_OK != Max30102_WriteReg(REG_LED2_PA, Value))
		return MAX30102_ERROR;
	return MAX30102_OK;
}

//
//	Usage functions
//
MAX30102_STATUS Max30102_IsFingerOnSensor(void)
{
	return IsFingerOnScreen;
}

int32_t Max30102_GetHeartRate(void)
{
	return HeartRate;
}

int32_t Max30102_GetSpO2Value(void)
{
	return Sp02Value;
}

void Max30102_Task(void)
{
	switch(StateMachine)
	{
		case MAX30102_STATE_BEGIN:
			HeartRate = 0;
			Sp02Value = 0;
			if(IsFingerOnScreen)
			{
				CollectedSamples = 0;
				BufferTail = BufferHead;
				Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_HIGH);		// RED current=7.2mA Register Address = 0x0C, RED
				Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_HIGH);		// IR current=7.2mA Register Address = 0x0D, IR
				StateMachine = MAX30102_STATE_CALIBRATE;
			}
			break;

		case MAX30102_STATE_CALIBRATE:
				if(IsFingerOnScreen)
				{
					if(CollectedSamples > (MAX30102_BUFFER_LENGTH-MAX30102_SAMPLES_PER_SECOND))		// integrated sample cnt = 5 sec * 100sps.
					{
						StateMachine = MAX30102_STATE_CALCULATE_HR;
					}
				}
				else
				{
					Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_LOW);
					Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW);
					StateMachine = MAX30102_STATE_BEGIN;
				}
			break;

		case MAX30102_STATE_CALCULATE_HR:
			if(IsFingerOnScreen)
			{
				maxim_heart_rate_and_oxygen_saturation(IrBuffer, RedBuffer, MAX30102_BUFFER_LENGTH-MAX30102_SAMPLES_PER_SECOND, BufferTail, &Sp02Value, &Sp02IsValid, &HeartRate, &IsHrValid);
				BufferTail = (BufferTail + MAX30102_SAMPLES_PER_SECOND) % MAX30102_BUFFER_LENGTH;
				CollectedSamples = 0;
				StateMachine = MAX30102_STATE_COLLECT_NEXT_PORTION;
			}
			else
			{
				Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_LOW);
				Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW);
				StateMachine = MAX30102_STATE_BEGIN;
			}
			break;

		case MAX30102_STATE_COLLECT_NEXT_PORTION:
			if(IsFingerOnScreen)
			{
				if(CollectedSamples > MAX30102_SAMPLES_PER_SECOND)
				{
					StateMachine = MAX30102_STATE_CALCULATE_HR;
				}
			}
			else
			{
				Max30102_Led1PulseAmplitude(MAX30102_RED_LED_CURRENT_LOW);
				Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW);
				StateMachine = MAX30102_STATE_BEGIN;
			}
			break;
	}
}

//
//	Initialization
//
MAX30102_STATUS Max30102_Init(I2C_HandleTypeDef *i2c)
{
	uint8_t uch_dummy;
	uint8_t s2;
	i2c_max30102 = i2c;


	if(MAX30102_OK != Max30102_Reset()) 												//resets the MAX30102
		return MAX30102_ERROR;

	if(MAX30102_OK != Max30102_ReadReg(0,&uch_dummy))									// 0x00를 읽어 uch_dummy에 저장
		return MAX30102_ERROR;															// 0x00 : Interrupt 레지스터(full, ready, ovf,)
	if(MAX30102_OK != Max30102_FifoWritePointer(0x00))									// Initialize FifoWritePTR
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_FifoOverflowCounter(0x00))								// Initialize OverflowCNT
		return MAX30102_ERROR;															// Over flow count 상태 clear
	if(MAX30102_OK != Max30102_FifoReadPointer(0x00))									// Initialize FifoReadPTR
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_FifoSampleAveraging(FIFO_SMP_AVE_4))						// No Average mode
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_FifoRolloverEnable(0))									// Disable overall writing
		return MAX30102_ERROR;															// full일 때 더이상 쓰여지지 않음.
	if(MAX30102_OK != Max30102_FifoAlmostFullValue(MAX30102_FIFO_ALMOST_FULL_SAMPLES))	// when unreaded data is 17, interrupt will be issue.
		return MAX30102_ERROR;
	//if(MAX30102_OK != Max30102_SetMode(MODE_SPO2_MODE))									// SPO2/HR/ Multi-LED Mode
	if(MAX30102_OK != Max30102_SetMode(MODE_HEART_RATE_MODE))							// HearRate Mode
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_SpO2AdcRange(SPO2_ADC_RGE_4096))							// FULL SCALE  4096nA (2^12)
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_SpO2SampleRate(SPO2_SAMPLE_RATE_1600))						// SPO2 Sample rate : 100 SPS
		return MAX30102_ERROR;
//===========Done
	if(MAX30102_OK != Max30102_SpO2LedPulseWidth(SPO2_PULSE_WIDTH_118))					// LED Pulse width : 411uSec
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_Led1PulseAmplitude(0x1f))		// Set RED LEd current = 0 (=OFF)
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_Led2PulseAmplitude(MAX30102_IR_LED_CURRENT_LOW))			// Set IR LEd current = 0 (=OFF)
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_SetIntAlmostFullEnabled(0))								// Interrupt A_FULL_EN Enable
		return MAX30102_ERROR;
	if(MAX30102_OK != Max30102_SetIntFifoDataReadyEnabled(1))							// Ready data Enable
		return MAX30102_ERROR;
	HAL_Delay(10);
	Max30102_ReadReg(0x0A, s2);
		//printf("SMPAVE: 3 FIFOROLL: 1 FIFO_A_FULL: 4 %d\n",s2);
	//if(MAX30102_OK != Max30102_WriteReg(REG_PILOT_PA,0x3f))   // Choose value for ~ 25mA for Pilot LED
	//	return MAX30102_ERROR;
	//StateMachine = MAX30102_STATE_BEGIN;
	return MAX30102_OK;
}
MAX30102_STATUS Max30102_State_Change(void)
{
	IsFingerOnScreen=1;
	return MAX30102_OK;
}
uint32_t* Max30102_RED_LED_BUFFER()
{
	return RedBuffer;
}
