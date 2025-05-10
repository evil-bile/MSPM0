/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



#include "ti_msp_dl_config.h"
#include "stdio.h"

uint16_t adc_data[20] = {0};


void uart_sendstring(char *str)
{
   // DL_UART_Main_transmitDataBlocking(UART_0_INST,'O');
    while(*str)
    {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, *str++);
        //DL_UART_Main_transmitDataBlocking(UART_0_INST,'K');
    }  
}


int main(void)
{
    char buffer[50] = {0};
    SYSCFG_DL_init();

    //设置DMA的目的地址
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)adc_data);

    // //设置DMA的源地址(ADC通道0的内存地址(开始地址))
    // DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, DL_ADC12_getMemResultAddress(ADC12_0_INST, DL_ADC12_MEM_IDX_0));

    //设置DMA的源地址（FIFO的内存地址）
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, DL_ADC12_getFIFOAddress(ADC12_0_INST));
    //开启DMA通道搬运
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    //触发ADC采样转换
    DL_ADC12_startConversion(ADC12_0_INST);

    uart_sendstring("adc_demo:");


    while (1) {
        for(int i = 0;i < 20;i++){
            sprintf(buffer," adc_data[%d]:%d\n", i, adc_data[i]);
            uart_sendstring(buffer);
        }

        delay_cycles(CPUCLK_FREQ);
    }
}
