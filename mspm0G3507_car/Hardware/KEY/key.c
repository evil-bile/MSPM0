/**
    该按键代码可以直接在KEY.h文件里更改按键宏定义
    #define KEY_PORT                                            Your KEY_PORT

    #define KEY1_PIN                                            your keynum1
    #define KEY2_PIN                                            your keynum2

    注意不要和我的宏定义重复，例如
    #define KEY_PORT                                            KEY_PORT

*/


#include "key.h"
#include "delay.h"

uint8_t Key1_Monitor(void)
{
    static uint8_t count = 0;

    if(count>=4)
        count = 0;
    if(DL_GPIO_readPins(KEY_PORT,KEY1_PIN) == 0)
    {
        delay_ms(20);
        if(DL_GPIO_readPins(KEY_PORT,KEY1_PIN) == 0)
        {
            while(DL_GPIO_readPins(KEY_PORT,KEY1_PIN) == 0);
            count++;
        }
    }
    return count;
}


uint8_t Key2_Monitor(void)
{
    static uint8_t flag = 0;

    if(DL_GPIO_readPins(KEY_PORT,KEY2_PIN) == 0)
    {
        delay_ms(20);
        if(DL_GPIO_readPins(KEY_PORT,KEY2_PIN == 0))
        {
            while(DL_GPIO_readPins(KEY_PORT,KEY2_PIN == 0));
            flag = 1;
        }
    }
    else
        flag=0;

    return flag;
}


