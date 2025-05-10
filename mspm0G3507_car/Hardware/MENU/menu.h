#ifndef __MENU_H
#define __MENU_H

#include "main.h"
#include "key.h"
#include "oled.h"


// 定义菜单项结构体
typedef struct {
	//菜单项的名字
    const char* name;
	//菜单项的动作（或者说是菜单项背后执行的函数）
    void (*action)();
} MenuItem;

// 定义菜单结构体
typedef struct {
	//菜单项的数组
    const MenuItem* items;
	//选项的数量
    int itemCount;
} Menu;

void Menu_Car(void);


#endif

