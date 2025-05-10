#include "menu.h"
#include "key.h"
#include "main.h"



// 菜单处理函数
void menuHandler(const Menu* menu) { 
    OLED_Clear();
    OLED_ShowString(30, 1, (unsigned char *)"MSP CAR", 8);
    while (1) {
        u8 key1_value = Key1_Monitor();
        u8 key_start_flag = Key2_Monitor();
        if (key1_value > 0 && key1_value <= menu->itemCount) {
            OLED_ShowString(20, 2, (unsigned char *)menu->items[key1_value - 1].name, 8);
            if(key_start_flag == 1){
                 menu->items[key1_value - 1].action();
            }
           
        }
    }
}

void show_statu(void){}
void set_Threshold(void){}
void hand_control(void){}
void auto_control(void){}

MenuItem mainMenuItems[] = {
    {"1.moer", show_statu},
    {"2.set", set_Threshold},
    {"3.hand", hand_control},
    {"4.atuo", auto_control}
};

Menu mainMenu = {
    mainMenuItems,
    sizeof(mainMenuItems) / sizeof(mainMenuItems[0])
};


void Menu_Car(void) {
    menuHandler(&mainMenu);
}
