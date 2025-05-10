#include "oled.h"
#include "oledfont.h" 
#include "ti_msp_dl_config.h" 	 
#ifdef __CC_ARM
#pragma O0
#elif defined(__GNUC__)
#pragma GCC optimize ("O0")
#elif defined(__clang__)
#pragma clang optimize off
#else
//adding ur own compiler controlling pragmas
#endif
//OLED鐨勬樉瀛�
//瀛樻斁鏍煎紡濡備笅.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 			   
void delay_ms(unsigned long ms) 
{
    while(ms--)
	    delay_cycles(CPUCLK_FREQ/1000);
}

//鍙嶆樉鍑芥暟
void OLED_ColorTurn(uint8_t i)
{
    if(i==0)
    {
        OLED_WR_Byte(0xA6,OLED_CMD);//姝ｅ父鏄剧ず
    }
    if(i==1)
    {
        OLED_WR_Byte(0xA7,OLED_CMD);//鍙嶈壊鏄剧ず
    }
}

//灞忓箷鏃嬭浆180搴�
void OLED_DisplayTurn(uint8_t i)
{
if(i==0)
    {
        OLED_WR_Byte(0xC8,OLED_CMD);//姝ｅ父鏄剧ず
        OLED_WR_Byte(0xA1,OLED_CMD);
    }
    if(i==1)
    {
        OLED_WR_Byte(0xC0,OLED_CMD);//鍙嶈浆鏄剧ず
        OLED_WR_Byte(0xA0,OLED_CMD);
    }
}

//璧峰�嬩俊鍙�
void I2C_Start(void)
{
    OLED_SDA_Set();
    OLED_SCL_Set();

    OLED_SDA_Clr();
    OLED_SCL_Clr();
}

//缁撴潫淇″彿
void I2C_Stop(void)
{
    OLED_SDA_Clr();
    OLED_SCL_Set();

    OLED_SDA_Set();
}

//绛夊緟淇″彿鍝嶅簲
void I2C_WaitAck(void) //娴嬫暟鎹�淇″彿鐨勭數骞�
{
    OLED_SDA_Set();

    OLED_SCL_Set();

    OLED_SCL_Clr();
}

//鍐欏叆涓€涓�瀛楄妭
void Send_Byte(uint8_t dat)
{
    uint8_t i;
    for(i=0;i<8;i++)
    {
        OLED_SCL_Clr();//灏嗘椂閽熶俊鍙疯�剧疆涓轰綆鐢靛钩
        if(dat&0x80)//灏哾at鐨�8浣嶄粠鏈€楂樹綅渚濇�″啓鍏�
        {
            OLED_SDA_Set();
        }
        else
        {
            OLED_SDA_Clr();
        }

        OLED_SCL_Set();

        OLED_SCL_Clr();
        dat<<=1;
    }
}

//鍙戦€佷竴涓�瀛楄妭
//鍚慡SD1306鍐欏叆涓€涓�瀛楄妭銆�
//mode:鏁版嵁/鍛戒护鏍囧織 0,琛ㄧず鍛戒护;1,琛ㄧず鏁版嵁;
void OLED_WR_Byte(uint8_t dat,uint8_t mode)
{
    I2C_Start();
    Send_Byte(0x78);
    I2C_WaitAck();
    if(mode){Send_Byte(0x40);}
    else{Send_Byte(0x00);}
    I2C_WaitAck();
    Send_Byte(dat);
    I2C_WaitAck();
    I2C_Stop();
}


//鍧愭爣璁剧疆

void OLED_Set_Pos(uint8_t x, uint8_t y) 
{ 
    OLED_WR_Byte(0xb0+y,OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
    OLED_WR_Byte((x&0x0f),OLED_CMD);
}   	  
//寮€鍚疧LED鏄剧ず    
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC鍛戒护
    OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//鍏抽棴OLED鏄剧ず     
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC鍛戒护
    OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   			 
//娓呭睆鍑芥暟,娓呭畬灞�,鏁翠釜灞忓箷鏄�榛戣壊鐨�!鍜屾病鐐逛寒涓€鏍�!!!	  
void OLED_Clear(void)  
{  
    uint8_t i,n;		    
    for(i=0;i<8;i++)  
    {  
        OLED_WR_Byte (0xb0+i,OLED_CMD);    //璁剧疆椤靛湴鍧€锛�0~7锛�
        OLED_WR_Byte (0x00,OLED_CMD);      //璁剧疆鏄剧ず浣嶇疆鈥斿垪浣庡湴鍧€
        OLED_WR_Byte (0x10,OLED_CMD);      //璁剧疆鏄剧ず浣嶇疆鈥斿垪楂樺湴鍧€   
        for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
    } //鏇存柊鏄剧ず
}

//鍦ㄦ寚瀹氫綅缃�鏄剧ず涓€涓�瀛楃��,鍖呮嫭閮ㄥ垎瀛楃��
//x:0~127
//y:0~63				 
//sizey:閫夋嫨瀛椾綋 6x8  8x16
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t sizey)
{      	
    uint8_t c=0,sizex=sizey/2;
    uint16_t i=0,size1;
    if(sizey==8)size1=6;
    else size1=(sizey/8+((sizey%8)?1:0))*(sizey/2);
    c=chr-' ';//寰楀埌鍋忕Щ鍚庣殑鍊�
    OLED_Set_Pos(x,y);
    for(i=0;i<size1;i++)
    {
        if(i%sizex==0&&sizey!=8) OLED_Set_Pos(x,y++);
        if(sizey==8) OLED_WR_Byte(asc2_0806[c][i],OLED_DATA);//6X8瀛楀彿
        else if(sizey==16) OLED_WR_Byte(asc2_1608[c][i],OLED_DATA);//8x16瀛楀彿
        //		else if(sizey==xx) OLED_WR_Byte(asc2_xxxx[c][i],OLED_DATA);//鐢ㄦ埛娣诲姞瀛楀彿
        else return;
    }
}
//m^n鍑芥暟
uint32_t oled_pow(uint8_t m,uint8_t n)
{
    uint32_t result=1;	 
    while(n--)result*=m;    
    return result;
}				  
//鏄剧ず鏁板瓧
//x,y :璧风偣鍧愭爣
//num:瑕佹樉绀虹殑鏁板瓧
//len :鏁板瓧鐨勪綅鏁�
//sizey:瀛椾綋澶у皬		  
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t sizey)
{         	
    uint8_t t,temp,m=0;
    uint8_t enshow=0;
    if(sizey==8)m=2;
    for(t=0;t<len;t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+(sizey/2+m)*t,y,' ',sizey);
                continue;
            }else enshow=1;
        }
        OLED_ShowChar(x+(sizey/2+m)*t,y,temp+'0',sizey);
    }
}
//鏄剧ず涓€涓�瀛楃�﹀彿涓�
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t sizey)
{
    uint8_t j=0;
    while (chr[j]!='\0')
    {		
        OLED_ShowChar(x,y,chr[j++],sizey);
        if(sizey==8)x+=6;
        else x+=sizey/2;
    }
}
//鏄剧ず姹夊瓧
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t no,uint8_t sizey)
{
    uint16_t i,size1=(sizey/8+((sizey%8)?1:0))*sizey;
    for(i=0;i<size1;i++)
    {
        if(i%sizey==0) OLED_Set_Pos(x,y++);
        if(sizey==16) OLED_WR_Byte(Hzk[no][i],OLED_DATA);//16x16瀛楀彿
        //		else if(sizey==xx) OLED_WR_Byte(xxx[c][i],OLED_DATA);//鐢ㄦ埛娣诲姞瀛楀彿
        else return;
    }				
}


//鏄剧ず鍥剧墖
//x,y鏄剧ず鍧愭爣
//sizex,sizey,鍥剧墖闀垮��
//BMP锛氳�佹樉绀虹殑鍥剧墖
void OLED_DrawBMP(uint8_t x,uint8_t y,uint8_t sizex, uint8_t sizey,uint8_t BMP[])
{ 	
    uint16_t j=0;
    uint8_t i,m;
    sizey=sizey/8+((sizey%8)?1:0);
    for(i=0;i<sizey;i++)
    {
        OLED_Set_Pos(x,i+y);
        for(m=0;m<sizex;m++)
        {      
            OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
        }
    }
} 



//鍒濆�嬪寲SSD1306					    
void OLED_Init(void)
{
    delay_ms(200);

    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0宸﹀彸鍙嶇疆 0xa1姝ｅ父
    OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0涓婁笅鍙嶇疆 0xc8姝ｅ父
    OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00,OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02,OLED_CMD);//
    OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
    OLED_Clear();
    OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
}  

