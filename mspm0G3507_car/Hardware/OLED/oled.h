#ifndef __OLED_H
#define __OLED_H			  	 
#include "main.h"


//----------------------------------------------------------------------------------
//OLED SSD1306 I2C 时钟SCL
#define		OLED_SCL_Set()			    (DL_GPIO_setPins(OLED_PORT, OLED_SCL_PIN))
#define		OLED_SCL_Clr()				(DL_GPIO_clearPins(OLED_PORT, OLED_SCL_PIN))

//----------------------------------------------------------------------------------
//OLED SSD1306 I2C 数据SDA
#define		OLED_SDA_Set()				(DL_GPIO_setPins(OLED_PORT, OLED_SDA_PIN))
#define		OLED_SDA_Clr()			    (DL_GPIO_clearPins(OLED_PORT, OLED_SDA_PIN))
#define 	OLED_READ_SDA()				((DL_GPIO_readPins(OLED_PORT, OLED_SDA_PIN)==OLED_SDA_PIN)?1:0)
#define 	OLED_SDA_OUT()				{DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX);DL_GPIO_enableOutput(OLED_PORT, OLED_SDA_PIN);}
#define 	OLED_SDA_IN()				{DL_GPIO_disableOutput(OLED_PORT, OLED_SDA_PIN);DL_GPIO_initDigitalInputFeatures(OLED_SDA_IOMUX,DL_GPIO_INVERSION_DISABLE,DL_GPIO_RESISTOR_PULL_UP,DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);}				   


#define OLED_CMD  0	//写命�?
#define OLED_DATA 1	//写数�?
//操作函数
void OLED_ClearPoint(uint8_t x,uint8_t y);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat,uint8_t mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);
void OLED_Clear(void);
//画形�?
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);//画点
void OLED_DrawLine(uint8_t X0, uint8_t Y0, uint8_t X1, uint8_t Y1);//画线
void OLED_DrawRectangle(uint8_t X,uint8_t Y,uint8_t Width, uint8_t Height,uint8_t IsFilled);//画矩�?
void OLED_DrawCircle(uint8_t X, uint8_t Y, uint8_t Radius, uint8_t IsFilled);//画圆�?
void OLED_DrawEllipse(uint8_t X, uint8_t Y, uint8_t A, uint8_t B, uint8_t IsFilled);//画椭�?
//显示内�??
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1,uint8_t mode);//显示字�??
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1,uint8_t mode);//显示字�?�串
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode);//显示正整�?
void OLED_ShowSignedNum(uint8_t x,uint8_t y,int32_t num,uint8_t len,uint8_t size1,uint8_t mode);//显示有�?�号整数
void OLED_ShowHexNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode);//显示十六进制数字
void OLED_ShowBinNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode);//显示二进制数�?
void OLED_ShowFloatNum(uint8_t x,uint8_t y,double num,uint8_t intlen,uint8_t fralen,uint8_t size1,uint8_t mode);//显示�?点型数据


void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1,uint8_t mode);
void OLED_ScrollDisplay(uint8_t num,uint8_t space,uint8_t mode);
void OLED_ShowPicture(uint8_t x,uint8_t y,uint8_t sizex,uint8_t sizey,uint8_t BMP[],uint8_t mode);
void OLED_Printf(uint8_t X, uint8_t Y, uint8_t FontSize, char *format, ...);


void OLED_SetCursor(uint8_t Page, uint8_t Y);
void OLED_UpdateArea(uint8_t X, uint8_t Y, uint8_t Width, uint8_t Height);



void OLED_Init(void);


void OLED_Update(void);
void OLED_Show_String(uint8_t han,uint8_t lie,uint8_t *string);
void OLED_DisplayLine(uint8_t han,uint8_t lie,uint8_t *temp);

#endif  
	 



