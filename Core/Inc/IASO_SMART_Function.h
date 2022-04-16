/*
 * IASO_SMART_Function.h
 *
 *  Created on: Sep 7, 2021
 *      Author: jojae
 */

#ifndef INC_IASO_SMART_FUNCTION_H_
#define INC_IASO_SMART_FUNCTION_H_
#include "main.h"
#include "MAX30102.h"
/**Variable**/
enum Simple_Switch {OFF=0,ON};
enum System_Mode {DIAGO=2,CHARGING,MODE_1,MODE_2,MODE_3,MODE_4_1,MODE_4_2};
enum key {NO_BTN=0,SYS_BTN,DIAGO_BTN,MODE_BTN};
enum RGB {RED_LED=1,GREEN_LED,BLUE_LED,TURQUOISE_LED,YELLOW_LED,PURPLE_LED,WHITE_LED};
enum Beep_Type {MODE_CHANGE=2};

//MCU_SETTING
char UartBuffer[32];
/**Variable Container**/

typedef struct{
/**MODE**/
	volatile uint8_t System_Mode:4;					// 0: OFF, 1: ON, 2: DIAGO, 3: CHARGING,  4: MODE_1, 5: MODE_2, 6: MODE_3, 7: MODE_4_1, 8:MODE_4_2
	volatile uint8_t Mode_Change:1;					// 0: OFF, 1: ON
	volatile uint8_t Mode_Change_Done:1;			// 0: OFF, 1: ON
/**Button**/
	volatile uint8_t Prev_Key:2;					//0: NO_BTN 1: SYS_BTN 2: DIAG_BTN 3: MODE_BTN
	volatile uint8_t Curr_Key:2; 					//0: NO_BTN 1: SYS_BTN 2: DIAG_BTN 3: MODE_BTN
	volatile uint8_t Chattering:1;					// 0: OFF, 1: ON
	volatile uint16_t Chattering_Time:10;			//200ms
	volatile uint16_t Chattering_Time_Val:10;		//200ms
	volatile uint16_t Button_Press_Time:11;
	volatile uint16_t Key_On_time:10;				// 1000msec
	volatile uint16_t Key_Off_time:11;				// 2000msec
/**Buzzer**/
	volatile uint8_t Beep_On:1;						//0: OFF, 1: ON
	volatile uint8_t Beep_Type:3;					//0: OFF 1: ON 2: BATTERY_LOW 3: MODE_CHANGE 4: MODE_STOP
	volatile uint8_t Beep_Done:1;					//0: ON, 1: OFF
	volatile uint16_t Sound_Tick:10;
	volatile uint8_t Sound_Tick_Cnt;				//100;
/**DDS**/
	volatile uint8_t DDS_First:1;					//0: OFF, 1: ON
	volatile uint8_t DDS_STATE:1;					//0: OFF, 1: ON
	volatile uint8_t DDS_Duty_time:4;				//0~10ms
	volatile uint8_t DDS_DutyRate;					//10ms 5ms OFF 5ms ON
	volatile uint32_t Frequency;					//FRE=960000
	SPI_HandleTypeDef *pSPI;						//SPI_HandleTypeDef *hspi1;
/**LED_STATE**/
	volatile uint8_t LED_STATE:3; 					//0: OFF, 1: RED, 2: GREEN, 3: BLUE, 4:TURQUOISE ,5: YELLOW, 6: SCARLET, 7: WHITE
	volatile uint8_t LED_MD1:1;   					//0: OFF, 1: ON
	volatile uint8_t LED_MD2:1;	 					//0: OFF, 1: ON
	volatile uint8_t LED_MD3:1;   					//0: OFF, 1: ON
/**LD_STATE**/
	volatile uint8_t LD_650:1;    					//0: OFF, 1: ON
	volatile uint8_t LD_830:1;    					//0: OFF, 1: ON
/**PPG_STATE**/
	volatile uint8_t PPG_First:1;					//0: OFF, 1: ON
	volatile uint8_t PPG_STATE:1; 					//0: OFF, 1: ON
	volatile uint8_t SMP_AVG:4;						//
	volatile uint8_t ADC_RGE:4;						//
	volatile uint8_t LED_PW:4;						//
	volatile uint8_t SPO2_RATE:4;					//
	volatile uint8_t LED_AMP;						//
	volatile uint32_t RED_BUFFER;	 				//RED_LED_DATA
	uint32_t IR_BUFFER;								//IR_LED_DATA
	I2C_HandleTypeDef *pI2C;						//I2C_HandleTypeDef *hi2c3;
/**TIME**/
	volatile uint16_t Time_1ms:10;					//1024
	volatile uint16_t Time_1sec:12;					//4096
	volatile uint16_t Time_Mode_1sec:11;			//2048
	volatile uint8_t Battery_SafeMode_Time;			//180sec
	volatile uint8_t Battery_SafeMode_Time_Val;
/**TEMP**/
	volatile uint8_t Temperature;					//25
	volatile uint16_t analog_Voltage_Temp_PCB; 		//25C 1.055V 1311
/**Analog_Vol**/
	volatile float battery_voltage;
	volatile uint16_t analog_VSENSE_USB;			//3416 -5.5V 1677 -2.7V 3105 -5V
	volatile uint16_t analog_VSENSE_BATT;			//4.1V Full_CHG 2547 3.9V Middle_CHG 2422 3.7V LOW_CHG 2298
	volatile uint8_t CHG_ST:1;						//0: OFF, 1: ON
	ADC_HandleTypeDef *pADC;
/**PWR_STATE**/
	volatile uint8_t MCU_PWR:1;						//0: OFF, 1: ON
	volatile uint8_t LD_PWR:1;						//0: OFF, 1: ON
	volatile uint8_t DDS_PWR:1;						//0: OFF, 1: ON
	volatile uint8_t ULTRA_SOUND_PWR:1;				//0: OFF, 1: ON
	volatile uint8_t PPG_PWR:1;						//0: OFF, 1: ON
}IASO_SMART_STATE;

/**Function**/

/**POWER**/
#define ON_SYSTEM_SET HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,SET);
#define ON_SYSTEM_RESET HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,RESET);

/**LED**/
#define LED_MD1_ON HAL_GPIO_WritePin(LED_MD1_GPIO_Port,LED_MD1_Pin,RESET)
#define LED_MD1_OFF HAL_GPIO_WritePin(LED_MD1_GPIO_Port,LED_MD1_Pin,SET)
#define LED_MD2_ON HAL_GPIO_WritePin(LED_MD2_GPIO_Port,LED_MD2_Pin,RESET)
#define LED_MD2_OFF HAL_GPIO_WritePin(LED_MD2_GPIO_Port,LED_MD2_Pin,SET)
#define LED_MD3_ON HAL_GPIO_WritePin(LED_MD3_GPIO_Port,LED_MD3_Pin,RESET)
#define LED_MD3_OFF HAL_GPIO_WritePin(LED_MD3_GPIO_Port,LED_MD3_Pin,SET)

/**RGB_LED**/
#define LED_COLOR_GR_ON HAL_GPIO_WritePin(LED_GR_GPIO_Port,LED_GR_Pin,RESET)
#define LED_COLOR_GR_OFF HAL_GPIO_WritePin(LED_GR_GPIO_Port,LED_GR_Pin,SET)
#define LED_COLOR_RD_ON HAL_GPIO_WritePin(LED_RD_GPIO_Port,LED_RD_Pin,RESET)
#define LED_COLOR_RD_OFF HAL_GPIO_WritePin(LED_RD_GPIO_Port,LED_RD_Pin,SET)
#define LED_COLOR_BL_ON HAL_GPIO_WritePin(LED_BL_GPIO_Port,LED_BL_Pin,RESET)
#define LED_COLOR_BL_OFF HAL_GPIO_WritePin(LED_BL_GPIO_Port,LED_BL_Pin,SET)

/**BUZZER**/
#define BEEP_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET)
#define BEEP_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, RESET)
#define BEEP_ON_CHANGE HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0)

/**LD**/
#define LD_PWR_ON HAL_GPIO_WritePin(ON_LD_PWR_GPIO_Port,ON_LD_PWR_Pin, SET)
#define LD_PWR_OFF HAL_GPIO_WritePin(ON_LD_PWR_GPIO_Port,ON_LD_PWR_Pin, RESET)
#define LD650_ON HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,SET)
#define LD650_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,RESET)
#define LD830_ON HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,SET)
#define LD830_OFF HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,RESET)

/**ULTRA_SOUND**/
#define SPI_NSS_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,RESET)
#define SPI_NSS_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,SET)
#define SOUND_PWR_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,SET)
#define SOUND_PWR_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,RESET)

/**ULTRA_SOUND_SHORT_CUT**/
#define DDS_PWR_ON HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,SET)
#define DDS_PWR_OFF HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,RESET)

/**PPG SENSOR**/
#define PPG_PWR_EN_ON HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, SET)
#define PPG_PWR_EN_OFF HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET)


/** INIT **/
void PORT_INIT(IASO_SMART_STATE *STA);
void STATE_INIT(IASO_SMART_STATE *STA,SPI_HandleTypeDef *hspi1,I2C_HandleTypeDef *hi2c3,ADC_HandleTypeDef *hadc1);
void PWR_UPDATE(IASO_SMART_STATE *STA);
/** KEY **/
void Get_Key(IASO_SMART_STATE*STA);
/** TIME **/
void Time_Update(IASO_SMART_STATE *STA);
void TIME(IASO_SMART_STATE*STA);
/** SEQUENCE **/
void Sequence(IASO_SMART_STATE*STA);
/** LED **/
void LED_STATE(IASO_SMART_STATE*STA);
/** MODE **/
void Mode_Pwr_Control(IASO_SMART_STATE*STA,char MCU,char LD,char DDS,char SOUND,char PPG);
void ON_Mode(IASO_SMART_STATE*STA);
void OFF_Mode(IASO_SMART_STATE*STA);
void DIAGO_Mode(IASO_SMART_STATE*STA);
void Charge_Mode(IASO_SMART_STATE*STA);
void MODE1(IASO_SMART_STATE*STA);
void MODE2(IASO_SMART_STATE*STA);
void MODE3(IASO_SMART_STATE*STA);
void MODE4_1(IASO_SMART_STATE*STA);
void MODE4_2(IASO_SMART_STATE*STA);
/** ADC **/
void Calc_ADC(IASO_SMART_STATE*STA);
void calc_Temp(IASO_SMART_STATE *STA);
void calc_Battery(IASO_SMART_STATE *STA);
/** Function **/
void PPG_Init(uint8_t FIFO_SMP_AVE,uint8_t SPO2_ADC_RGE,uint8_t LED_PW,uint8_t SPO2_RATE,uint8_t LED_AMP);
void PPG_RESET();
void DDS_Write(uint32_t freq,SPI_HandleTypeDef hspi1);
void Beep_Control(IASO_SMART_STATE*STA,char Type);
#endif /* INC_IASO_SMART_FUNCTION_H_ */
