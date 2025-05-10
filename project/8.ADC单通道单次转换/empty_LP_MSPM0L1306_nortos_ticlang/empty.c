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


#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "stdio.h"
uint16_t adc_data = 0;


void uart_sendstring(char *str)
{
   // DL_UART_Main_transmitDataBlocking(UART_0_INST,'O');
    while(*str)
    {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, *str++);
        //DL_UART_Main_transmitDataBlocking(UART_0_INST,'K');
    }  
}

void adc_readValue(void)
{
    //开启ADC转换
    DL_ADC12_startConversion(ADC12_0_INST);

    //如果ADC正忙
    while(DL_ADC12_getStatus(ADC12_0_INST) != DL_ADC12_STATUS_CONVERSION_IDLE);

    //暂停ADC转换
    DL_ADC12_stopConversion(ADC12_0_INST);

    adc_data = DL_ADC12_getMemResult(ADC12_0_INST,DL_ADC12_MEM_IDX_0);

    //单次转换会自动停止，下次启动需要手动开启
    DL_ADC12_enableConversions(ADC12_0_INST);
}

int main(void)
{
    char buffer[50] = {0};
    SYSCFG_DL_init();

    uart_sendstring("adc_demo:");

    while (1) {
        adc_readValue();
        sprintf(buffer,"adc_getvalue:%d\n", adc_data);
        uart_sendstring(buffer);

        delay_cycles(CPUCLK_FREQ/2);
    }
}
