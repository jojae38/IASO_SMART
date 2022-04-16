/*
 * IASO_SMART_Function.c
 *
 *  Created on: Sep 7, 2021
 *      Author: jojae
 */
#include "IASO_SMART_Function.h"

uint8_t s2;
void PORT_INIT(IASO_SMART_STATE *STA)
{
	ON_SYSTEM_SET;
	DDS_PWR_OFF;
	LD_PWR_OFF;
	PPG_PWR_EN_OFF;
	SOUND_PWR_OFF;
	BEEP_OFF;
	LED_MD1_OFF;
	LED_MD2_OFF;
	LED_MD3_OFF;
	LED_COLOR_RD_OFF;
	LED_COLOR_BL_OFF;
	LED_COLOR_GR_OFF;
	LD650_OFF;
	LD830_OFF;
	HAL_Delay(10);
}
void STATE_INIT(IASO_SMART_STATE *STA,SPI_HandleTypeDef *hspi1,I2C_HandleTypeDef *hi2c3,ADC_HandleTypeDef *hadc1)
{
	/**Mode**/
	STA->System_Mode=OFF;
	STA->Mode_Change=0;
	STA->Mode_Change_Done=1;
	/**Button**/
	STA->Prev_Key=NO_BTN;
	STA->Curr_Key=NO_BTN;
	STA->Chattering=OFF;
	STA->Chattering_Time=0;
	STA->Chattering_Time_Val=200;
	STA->Button_Press_Time=0;
	STA->Key_Off_time=2000;
	STA->Key_On_time=1000;
	/**Buzzer**/
	STA->Beep_On=OFF;
	STA->Beep_Type=ON;
	STA->Beep_Done=ON;
	STA->Sound_Tick=0;
	STA->Sound_Tick_Cnt=100;
	/**DDS**/
	STA->DDS_First=OFF;
	STA->DDS_DutyRate=5;
	STA->DDS_STATE=OFF;
	STA->Frequency=960000;
	STA->pSPI=hspi1;
	/**LED_STATE**/
	STA->LED_STATE=OFF;
	STA->LED_MD1=OFF;
	STA->LED_MD2=OFF;
	STA->LED_MD3=OFF;
	/**LD_STATE**/
	STA->LD_650=OFF;
	STA->LD_830=OFF;
	/**PPG_STATE**/
	STA->PPG_First=OFF;
	STA->PPG_STATE=OFF;
	STA->SMP_AVG=FIFO_SMP_AVE_4;
	STA->ADC_RGE=SPO2_ADC_RGE_4096;
	STA->LED_PW=SPO2_PULSE_WIDTH_118;
	STA->SPO2_RATE=SPO2_SAMPLE_RATE_1600;
	STA->LED_AMP=0x1F;
	STA->pI2C=hi2c3;
	/**TIME**/
	STA->Time_1ms=0;
	STA->Time_1sec=0;
	STA->Time_Mode_1sec=0;
	STA->Battery_SafeMode_Time=0;
	STA->Battery_SafeMode_Time_Val=180;
	/**TEMP**/
	STA->Temperature=25;
	STA->analog_Voltage_Temp_PCB=1311;
	/**Analog_Vol**/
	STA->battery_voltage=3.9;
	STA->analog_VSENSE_USB=3105;
	STA->analog_VSENSE_BATT=2422;
	STA->CHG_ST=OFF;
	STA->pADC=hadc1;
	/**PWR_STATE**/
	STA->MCU_PWR=OFF;
	STA->LD_PWR=OFF;
	STA->DDS_PWR=OFF;
	STA->ULTRA_SOUND_PWR=OFF;
	STA->PPG_PWR=OFF;
}
void PWR_UPDATE(IASO_SMART_STATE *STA)
{
	if(STA->MCU_PWR==ON)
	{
		ON_SYSTEM_SET;
	}
	else if(STA->MCU_PWR==OFF)
	{
		ON_SYSTEM_RESET;
	}
	if(STA->LD_PWR==ON)
	{
		LD_PWR_ON;
		LD830_ON;
		LD650_ON;
	}
	else
	{
		LD830_OFF;
		LD650_OFF;
		LD_PWR_OFF;
	}
	if(STA->DDS_PWR==ON)
	{
		DDS_PWR_ON;
		DDS_Write(STA->Frequency, *(STA->pSPI));
	}
	else
	{
		DDS_PWR_OFF;
	}
	if(STA->ULTRA_SOUND_PWR==ON)
	{
		if(STA->DDS_First==OFF)
		{
			STA->DDS_First=ON;
		}
		if(STA->DDS_STATE==ON)
		{
			SOUND_PWR_ON;
		}
		else
		{
			SOUND_PWR_OFF;
		}
	}
	else
	{
		DDS_PWR_OFF;
		SOUND_PWR_OFF;
		STA->DDS_First=OFF;
	}
	if(STA->PPG_PWR==ON)
	{
		PPG_PWR_EN_ON;
		if(STA->PPG_STATE==OFF)
			{
				Max30102_Init(STA->pI2C);
				PPG_Init(STA->SMP_AVG, STA->ADC_RGE, STA->LED_PW, STA->SPO2_RATE, STA->LED_AMP);
				STA->PPG_STATE=ON;
			}
	}
	else
	{
		PPG_PWR_EN_OFF;
	}
}
void Get_Key(IASO_SMART_STATE*STA)
{
	if(STA->Chattering==0)
	{
		STA->Chattering=1;
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)==RESET)
		{
			STA->Curr_Key=SYS_BTN;
		}
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)==RESET)
		{
			STA->Curr_Key=DIAGO_BTN;
		}
		else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)==RESET)
		{
			STA->Curr_Key=MODE_BTN;
		}
		else
		{
			STA->Curr_Key=NO_BTN;
			STA->Chattering=0;
			STA->Button_Press_Time=0;
		}
	}

}
void Time_Update(IASO_SMART_STATE *STA)
{
	STA->Time_1ms++;

	if(STA->Curr_Key!=NO_BTN)
	{
		STA->Button_Press_Time++;
	}
	if(STA->Chattering==1)
	{
		STA->Chattering_Time++;
	}
	if(STA->Beep_On==ON)
	{
		STA->Sound_Tick++;
	}
	if(STA->ULTRA_SOUND_PWR==ON)
	{
		STA->DDS_Duty_time++;
	}
}
void TIME(IASO_SMART_STATE*STA)
{

	if(STA->Chattering_Time>200)
	{
		STA->Chattering=0;
		STA->Chattering_Time=0;
	}
	if(STA->DDS_Duty_time>=11)
	{
		STA->DDS_Duty_time=0;
	}
	if(STA->Time_1ms>=1000)
	{
		if(STA->System_Mode>1)
		{
			STA->Time_Mode_1sec++;
		}
		if(STA->Battery_SafeMode_Time>180)
		{
			STA->System_Mode=OFF;
			Beep_Control(STA, OFF);
		}
		STA->Time_1sec++;
		STA->Time_1ms=0;
		if(STA->System_Mode==ON&&STA->Curr_Key==NO_BTN)
		{
			STA->Battery_SafeMode_Time++;
		}
		else
		{
			STA->Battery_SafeMode_Time=0;
		}
	}
}
void Sequence(IASO_SMART_STATE*STA)
{
	switch(STA->System_Mode)
	{
	case OFF:
		OFF_Mode(STA);
		STA->LED_STATE=OFF;
		break;
	case ON:
		ON_Mode(STA);
		STA->LED_STATE=GREEN_LED;
		break;
	case DIAGO:
		DIAGO_Mode(STA);
		STA->LED_STATE=BLUE_LED;
		break;
	case CHARGING:
		Charge_Mode(STA);
		STA->LED_STATE=OFF;
		break;
	case MODE_1:
		MODE1(STA);
		STA->LED_STATE=RED_LED;
		break;
	case MODE_2:
		MODE2(STA);
		STA->LED_STATE=PURPLE_LED;
		break;
	case MODE_3:
		MODE3(STA);
		STA->LED_STATE=TURQUOISE_LED;
		break;
	case MODE_4_1:
		MODE4_1(STA);
		STA->LED_STATE=PURPLE_LED;
		break;
	case MODE_4_2:
		MODE4_2(STA);
		STA->LED_STATE=YELLOW_LED;
		break;
	}
}
void LED_STATE(IASO_SMART_STATE*STA)
{
	if(STA->System_Mode==OFF)
	{
		LED_MD1_OFF;
		LED_MD2_OFF;
		LED_MD3_OFF;
	}
	else
	{
		if(STA->LED_MD1==ON)
			{
				LED_MD1_ON;
			}
			else
			{
				LED_MD1_OFF;
			}
			if(STA->LED_MD2==ON)
			{
				LED_MD2_ON;
			}
			else
			{
				LED_MD2_OFF;
			}
			if(STA->LED_MD3==ON)
			{
				LED_MD3_ON;
			}
			else
			{
				LED_MD3_OFF;
			}
	}
	switch(STA->LED_STATE)
	{
	case OFF:
		LED_COLOR_GR_OFF;
		LED_COLOR_BL_OFF;
		LED_COLOR_RD_OFF;
		break;
	case RED_LED:
		LED_COLOR_GR_OFF;
		LED_COLOR_BL_OFF;
		LED_COLOR_RD_ON;
		break;
	case GREEN_LED:
		LED_COLOR_GR_ON;
		LED_COLOR_BL_OFF;
		LED_COLOR_RD_OFF;
		break;
	case BLUE_LED:
		LED_COLOR_GR_OFF;
		LED_COLOR_BL_ON;
		LED_COLOR_RD_OFF;
		break;
	case TURQUOISE_LED:
		LED_COLOR_GR_ON;
		LED_COLOR_BL_ON;
		LED_COLOR_RD_OFF;
		break;
	case YELLOW_LED:
		LED_COLOR_GR_ON;
		LED_COLOR_BL_OFF;
		LED_COLOR_RD_ON;
		break;
	case PURPLE_LED:
		LED_COLOR_GR_OFF;
		LED_COLOR_BL_ON;
		LED_COLOR_RD_ON;
		break;
	case WHITE_LED:
		LED_COLOR_GR_ON;
		LED_COLOR_BL_ON;
		LED_COLOR_RD_ON;
		break;
	}
}
void Beep_Control(IASO_SMART_STATE*STA,char Type)
{
	STA->Beep_On=ON;
	STA->Beep_Type=Type;
	STA->Beep_Done=OFF;
}
void Mode_Pwr_Control(IASO_SMART_STATE*STA,char MCU,char LD,char DDS,char SOUND,char PPG)
{
	STA->MCU_PWR=MCU;
	STA->LD_PWR=LD;
	STA->DDS_PWR=DDS;
	STA->ULTRA_SOUND_PWR=SOUND;
	STA->PPG_PWR=PPG;
}
void ON_Mode(IASO_SMART_STATE*STA)
{
	Mode_Pwr_Control(STA, ON, OFF, OFF,	OFF, OFF);
	if(STA->Curr_Key==SYS_BTN&&STA->Button_Press_Time>STA->Key_Off_time)
	{
		STA->System_Mode=OFF;
		Beep_Control(STA, OFF);
		STA->Button_Press_Time=0;
	}
	else if(STA->Curr_Key==DIAGO_BTN)
	{
		STA->System_Mode=DIAGO;
		STA->Time_Mode_1sec=0;
		Beep_Control(STA, MODE_CHANGE);
		STA->PPG_STATE=OFF;
	}
	else if(STA->Curr_Key==MODE_BTN)
	{
		STA->System_Mode=MODE_1;
		STA->Time_Mode_1sec=0;
		STA->Curr_Key=NO_BTN;
		Beep_Control(STA, MODE_CHANGE);
	}
}
void OFF_Mode(IASO_SMART_STATE*STA)
{
	Mode_Pwr_Control(STA, OFF, OFF, OFF, OFF, OFF);
	if(STA->Curr_Key==SYS_BTN&&STA->Button_Press_Time>STA->Key_On_time)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, ON);
		STA->Button_Press_Time=0;
	}
}
void DIAGO_Mode(IASO_SMART_STATE*STA)
{
	if(STA->Curr_Key==SYS_BTN)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}
	if(STA->Time_Mode_1sec>300)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}

	Mode_Pwr_Control(STA, ON, OFF, OFF, OFF, ON);
}
void Charge_Mode(IASO_SMART_STATE*STA)
{
	calc_Battery(STA);
}
void MODE1(IASO_SMART_STATE*STA)
{
	Mode_Pwr_Control(STA, ON, ON, OFF, OFF, OFF);
	if(STA->Curr_Key==SYS_BTN)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}
	else if(STA->Curr_Key==MODE_BTN)
	{
		STA->System_Mode=MODE_2;
		Beep_Control(STA, MODE_CHANGE);
		STA->Curr_Key=NO_BTN;
	}
	if(STA->Time_Mode_1sec>1800)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}
}
void MODE2(IASO_SMART_STATE*STA)
{
	Mode_Pwr_Control(STA, ON, OFF, ON, OFF, OFF);
	if(STA->Curr_Key==SYS_BTN)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}
	else if(STA->Curr_Key==MODE_BTN)
	{
		STA->System_Mode=MODE_3;
		Beep_Control(STA, MODE_CHANGE);
		STA->Curr_Key=NO_BTN;
	}
	if(STA->Time_Mode_1sec>1800)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}
}
void MODE3(IASO_SMART_STATE*STA)
{
	Mode_Pwr_Control(STA, ON, ON, ON, ON, OFF);
	if(STA->Curr_Key==SYS_BTN)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}
	else if(STA->Curr_Key==MODE_BTN)
	{
		STA->System_Mode=MODE_4_1;
		Beep_Control(STA, MODE_CHANGE);
		STA->Curr_Key=NO_BTN;
		STA->PPG_STATE=OFF;
	}
	if(STA->Time_Mode_1sec>1800)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}
}
void MODE4_1(IASO_SMART_STATE*STA) //PPG+LD+ULTRASOUND
{
	Mode_Pwr_Control(STA, ON, ON, ON, ON, ON);
	if(STA->Curr_Key==SYS_BTN)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}
	else if(STA->Curr_Key==MODE_BTN)
	{
		STA->System_Mode=MODE_1;
		Beep_Control(STA, MODE_CHANGE);
		STA->Curr_Key=NO_BTN;
	}
	if(STA->Time_Mode_1sec>300)
	{
		STA->System_Mode=MODE_4_2;
	}
	else if(STA->Time_Mode_1sec>1800)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}
}
void MODE4_2(IASO_SMART_STATE*STA) //LD+ULTRASOUND
{
	Mode_Pwr_Control(STA, ON, ON, ON, ON, OFF);
	if(STA->Curr_Key==SYS_BTN)
	{
		STA->System_Mode=ON;
		Beep_Control(STA, MODE_CHANGE);
	}
	else if(STA->Curr_Key==MODE_BTN)
	{
		STA->System_Mode=MODE_1;
		Beep_Control(STA, MODE_CHANGE);
		STA->Curr_Key=NO_BTN;
	}
	if(STA->Time_Mode_1sec>1500)
	{
		STA->System_Mode=MODE_4_1;
		STA->PPG_STATE=OFF;
	}
}

void calc_Temp(IASO_SMART_STATE *STA)
{

}
void calc_Battery(IASO_SMART_STATE *STA)
{	Calc_ADC(STA);
	if(STA->System_Mode!=OFF)
	{
		if(STA->analog_VSENSE_BATT<2269)
				  {
					  STA->LED_MD3=OFF;
					  STA->LED_MD2=OFF;
					  STA->LED_MD1=ON;
				  }
				  else if(STA->analog_VSENSE_BATT<2392)
				  {
					  STA->LED_MD3=OFF;
					  STA->LED_MD2=ON;
					  STA->LED_MD1=ON;
				  }
				  else if(STA->analog_VSENSE_BATT<2515)
				  {
					  STA->LED_MD3=ON;
					  STA->LED_MD2=ON;
					  STA->LED_MD1=ON;
				  }
	}
}
void PPG_Init(uint8_t FIFO_SMP_AVE,uint8_t SPO2_ADC_RGE,uint8_t LED_PW,uint8_t SPO2_RATE,uint8_t LED_AMP)
{
	HAL_Delay(10);
	printf("%d\n",Max30102_FifoSampleAveraging(FIFO_SMP_AVE));
	HAL_Delay(10);
	printf("%d\n",Max30102_SetIntAlmostFullEnabled(0));
	HAL_Delay(10);
	printf("%d\n",Max30102_FifoRolloverEnable(0));
	HAL_Delay(10);
	printf("%d\n",Max30102_SetMode(MODE_HEART_RATE_MODE));
	HAL_Delay(10);
	printf("%d\n",Max30102_SpO2AdcRange(SPO2_ADC_RGE));
	HAL_Delay(10);
	printf("%d\n",Max30102_SpO2LedPulseWidth(LED_PW));
	HAL_Delay(10);
	printf("%d\n",Max30102_SpO2SampleRate(SPO2_RATE));
	HAL_Delay(10);
	printf("%d\n",Max30102_Led1PulseAmplitude(LED_AMP));

//	Max30102_ReadReg(0x0A, &s2);
//	printf("SMPAVE: 3 FIFOROLL: 1 FIFO_A_FULL: 4 %d\n",s2);
//	HAL_Delay(10);
//	Max30102_ReadReg(0x09, &s2);
//	printf("SHDN: 1 RESET: 1 BLANK: 3 MODE: 3 %d\n",s2);
//	HAL_Delay(10);
//	Max30102_ReadReg(0x0A, &s2);
//	printf("BLANK: 1 ADC_RGE: 2 SPO2_SR: 3 LED_PW: 2 %d\n",s2);
//	HAL_Delay(10);
//	Max30102_ReadReg(0x0C, &s2);
//	printf("RED_LED_AMPL %d\n",s2);
//	HAL_Delay(10);
//	DATA_SAMPLING - 8bit
//	Max30102_FifoSampleAveraging(0x03);
//	HAL_Delay(10);
//	SET_MODE - HR_MODE
//	Max30102_SetMode(0x02);
//	HAL_Delay(10);
//	SET ADC_RANGE -
//	Max30102_SpO2AdcRange(0x01);
//	HAL_Delay(10);
//	SET RED LED SAMPLERATE - 800sample/sec
//	Max30102_SpO2SampleRate(0x04);
//	HAL_Delay(10);
//	SET RED LED PULSE WIDTH - 16bit resolution 118us
//	Max30102_SpO2LedPulseWidth(0x01);
//	HAL_Delay(10);
//	SET RED LED CURRENT to 6.4mA
//	Max30102_Led1PulseAmplitude(0x1F);
//	HAL_Delay(10);
}
void PPG_RESET()
{
	Max30102_Reset();
}


void Calc_ADC(IASO_SMART_STATE*STA)
{
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, 100);
//	STA->analog_VSENSE_USB=HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Start(STA->pADC);
	HAL_ADC_PollForConversion(STA->pADC, 100);
	STA->analog_VSENSE_BATT=HAL_ADC_GetValue(STA->pADC);
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, 100);
//	STA->analog_Voltage_Temp_PCB=HAL_ADC_GetValue(&hadc1);
}
void DDS_Write(uint32_t freq,SPI_HandleTypeDef hspi1)	// Write DDS data
{
	float test=0;
	uint16_t AD9837_CtrlReg=0;
	uint16_t FreqRegMSB =0;
	uint16_t FreqRegLSB =0;
	uint16_t PhaseReg = 0;

	AD9837_CtrlReg = 0x2100;			// ADC9837 Control Register REST

	test = (float)(freq)*16.777216;		// 16.777216 = 2^28/16MHz Value
	FreqRegLSB = (uint16_t)( (int)(test) & 0x3FFF);
	FreqRegLSB |= 0x4000;

	FreqRegMSB = (uint16_t)( (int)(test)>>14 );
	FreqRegMSB |= 0x4000;

	PhaseReg = 0xC000;

//	AD9837_CtrlReg = 0x2100;			// 0010  0001  0000  0000
//	FreqRegLSB = 0x50C7;				// 0101  0000  1100  0111
//	FreqRegMSB = 0x4CF0;				// 0100  0000  0000  0000
//	PhaseReg = 0xC000;					// 1100  0000  0000  0000
////	AD9837_CtrlReg = 0x2000;		// 0010  0000  0000  0000
	SPI_NSS_ON;
	HAL_SPI_Transmit(&hspi1, &AD9837_CtrlReg, 1, 100);		// AD9837 Control Register
	HAL_SPI_Transmit(&hspi1, &FreqRegLSB, 1, 100);			// Frequency Register Low
	HAL_SPI_Transmit(&hspi1, &FreqRegMSB, 1, 100);			// Frequency Register High
	HAL_SPI_Transmit(&hspi1, &PhaseReg, 1, 100);			// Phase Register
	//AD9837_CtrlReg = 0x2000;	// Run Sine Wave
//	AD9837_CtrlReg = 0x2002;	// Run Triangle Wave
//	AD9837_CtrlReg = 0x2020;	// Run MSB/2 --> Setting Freq/2
	AD9837_CtrlReg = 0x2028;	// Run MSB   --> Setting Freq. ( Rectangular wave)
	HAL_SPI_Transmit(&hspi1, &AD9837_CtrlReg, 1, 100);		// Run DDS
	SPI_NSS_OFF;
}

