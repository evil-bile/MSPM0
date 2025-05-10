// /*
//  * Copyright (c) 2021, Texas Instruments Incorporated
//  * All rights reserved.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions
//  * are met:
//  *
//  * *  Redistributions of source code must retain the above copyright
//  *    notice, this list of conditions and the following disclaimer.
//  *
//  * *  Redistributions in binary form must reproduce the above copyright
//  *    notice, this list of conditions and the following disclaimer in the
//  *    documentation and/or other materials provided with the distribution.
//  *
//  * *  Neither the name of Texas Instruments Incorporated nor the names of
//  *    its contributors may be used to endorse or promote products derived
//  *    from this software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//  * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
//  * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
//  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
//  * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
//  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
//  * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  */

#include "ti_msp_dl_config.h"
#include "mpu6050.h"
#include "delay.h"
#include "main.h"

char str = 0;
char buffer[100] = {0};

void uart_sendstring(char *str)
{
    //DL_UART_Main_transmitDataBlocking(UART_0_INST,'O');
    while(*str)
    {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, *str++);
        //DL_UART_Main_transmitDataBlocking(UART_0_INST,'K');
    }  
}


void mpu6050_send_data(void)
{
    sprintf(buffer, "Pitch:  %4.2f/n", mpu6050.Pitch);
    uart_sendstring(buffer);
    sprintf(buffer, "Yaw:  %4.2f/n", mpu6050.Yaw);
    uart_sendstring(buffer);
    sprintf(buffer, "Roll:  %4.2f/n", mpu6050.Roll);
    uart_sendstring(buffer);

}


int main(void)
{
    SYSCFG_DL_init();

    mpu6050_init();

    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_0_INST);

    
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    while (1) {
        mpu6050_send_data();
        delay_ms(1000);
    }
}


void TIMER_0_INST_IRQHandler(void)
{
    AHRS_Geteuler();
}




void UART_0_INST_IRQHandler(void)
{
   str = DL_UART_Main_receiveData(UART_0_INST);
   DL_UART_Main_transmitDataBlocking(UART_0_INST, str);
    // uart_sendstring(&str);
}


// #include "ti_msp_dl_config.h"

// volatile uint8_t gEchoData = 0;

// int main(void)
// {
//     SYSCFG_DL_init();

//     NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
//     NVIC_EnableIRQ(UART_0_INST_INT_IRQN);


//     while (1) {
        
//     }
// }

// void UART_0_INST_IRQHandler(void)
// {
//     switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
//         case DL_UART_MAIN_IIDX_RX:
//             gEchoData = DL_UART_Main_receiveData(UART_0_INST);
//             DL_UART_Main_transmitData(UART_0_INST, gEchoData);
//             break;
//         default:
//             break;
//     }
// }
