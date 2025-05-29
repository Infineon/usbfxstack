/***************************************************************************//**
* \file cy_fault_handlers.c
* \version 1.0
*
* Provides general purpose CPU fault handlers for the FX device family.
*
*******************************************************************************
* \copyright
* (c) (2025), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "cy_syslib.h"
#if FREERTOS_ENABLE
#include "FreeRTOS.h"
#include "task.h"
#endif /* FREERTOS_ENABLE */
#include "cy_sysclk.h"
#include "cy_wdt.h"
#include "cy_debug.h"

#if defined(__cplusplus)
extern "C" {
#endif

static uint16_t wdt_rounded_timeout_ms = 0;
static uint32_t wdt_ignore_bits = 0;

typedef struct {
    uint16_t min_period_ms; // Minimum period in milliseconds that can be represented with this many ignored bits
    uint16_t round_threshold_ms; // Timeout threshold in milliseconds from which to round up to the minimum period
} wdt_ignore_bits_data_t;

static const wdt_ignore_bits_data_t wdt_ignore_data[] = {
    {4000, 3001}, //  0 bit(s): min period: 4000ms, max period: 6000ms, round up from 3001+ms
    {2000, 1501}, //  1 bit(s): min period: 2000ms, max period: 3000ms, round up from 1501+ms
    {1000,  751}, //  2 bit(s): min period: 1000ms, max period: 1500ms, round up from 751+ms
    { 500,  376}, //  3 bit(s): min period:  500ms, max period:  750ms, round up from 376+ms
    { 250,  188}, //  4 bit(s): min period:  250ms, max period:  375ms, round up from 188+ms
    { 125,   94}, //  5 bit(s): min period:  125ms, max period:  187ms, round up from 94+ms
    {  63,   47}, //  6 bit(s): min period:   63ms, max period:   93ms, round up from 47+ms
    {  32,   24}, //  7 bit(s): min period:   32ms, max period:   46ms, round up from 24+ms
    {  16,   12}, //  8 bit(s): min period:   16ms, max period:   23ms, round up from 12+ms
    {   8,    6}, //  9 bit(s): min period:    8ms, max period:   11ms, round up from 6+ms
    {   4,    3}, // 10 bit(s): min period:    4ms, max period:    5ms, round up from 3+ms
    {   2,    2}, // 11 bit(s): min period:    2ms, max period:    2ms, round up from 2+ms
    {   1,    1}, // 12 bit(s): min period:    1ms, max period:    1ms, round up from 1+ms
};

#if 0
static const _cyhal_wdt_ignore_bits_data_t _cyhal_wdt_ignore_data[] = {
    {3277, 2458}, //  0 bit(s): min period: 3277ms, max period: 4915ms, round up from 2458+ms
    {1639, 1229}, //  1 bit(s): min period: 1639ms, max period: 2457ms, round up from 1229+ms
    { 820,  615}, //  2 bit(s): min period:  820ms, max period: 1228ms, round up from 615+ms
    { 410,  308}, //  3 bit(s): min period:  410ms, max period:  614ms, round up from 308+ms
    { 205,  154}, //  4 bit(s): min period:  205ms, max period:  307ms, round up from 154+ms
    { 103,   77}, //  5 bit(s): min period:  103ms, max period:  153ms, round up from 77+ms
    {  52,   39}, //  6 bit(s): min period:   52ms, max period:   76ms, round up from 39+ms
    {  26,   20}, //  7 bit(s): min period:   26ms, max period:   38ms, round up from 20+ms
    {  13,   10}, //  8 bit(s): min period:   13ms, max period:   19ms, round up from 10+ms
    {   7,    5}, //  9 bit(s): min period:    7ms, max period:    9ms, round up from 5+ms
    {   4,    3}, // 10 bit(s): min period:    4ms, max period:    4ms, round up from 3+ms
    {   2,    2}, // 11 bit(s): min period:    2ms, max period:    2ms, round up from 2+ms
    {   1,    1}, // 12 bit(s): min period:    1ms, max period:    1ms, round up from 1+ms
};
#endif

/* Fault Simulation Functions that can be used for testing */
#if 0
__asm void usage_fault(void)
{
    DCI  0xF123                         // DCI is not an ARM assmebly instruction
    DCI  0x4567
    BX   LR
}

__asm void mem_fault(void)
{
    B 0xFFFFFFFF
}

void bus_fault(void)
{
    volatile unsigned int* p;
    unsigned int n;
    p = (unsigned int*)0xCCCCCCCC;
    n = *p;
}
#endif

#if FREERTOS_ENABLE
/** Stack Overflow callback function from FreeRTOS **/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    Cy_Debug_SetPrintNow(true);
    DBG_APP_INFO("Stack Overflow in %s task\r\n", pcTaskName);

    while(true) {}
}
#endif /* FREERTOS_ENABLE */

void Cy_SysLib_ProcessingFault(void)
{
#if (CY_CPU_CORTEX_M4)
    cy_stc_fault_shcsr_t shcsrVal = cy_faultFrame.shcsr.shcsrBits;

#if DEBUG_INFRA_EN
    /* Since a fault has been detected, enable immediate flush of messages. */
    Cy_Debug_SetPrintNow(true);
#endif /* DEBUG_INFRA_EN */

    if(shcsrVal.memFaultAct == true)
    {
        DBG_APP_INFO("Memory Manage Fault\r\n");
        if(cy_faultFrame.cfsr.cfsrBits.mmarValid == true)
        {            
            DBG_APP_INFO("Fault Addr: %x\r\n", cy_faultFrame.mmfar);
            cy_faultFrame.cfsr.cfsrBits.mmarValid = 0;
        }
    }
    else if(shcsrVal.busFaultAct == true)
    {
        DBG_APP_INFO("Bus Fault\r\n");
        if(cy_faultFrame.cfsr.cfsrBits.bfarValid == true)
        {            
            DBG_APP_INFO("Fault Addr: %x\r\n", cy_faultFrame.bfar);
            cy_faultFrame.cfsr.cfsrBits.bfarValid = 0;
        }
    }
    else if(shcsrVal.usgFaultAct == true)
    {
        DBG_APP_INFO("Usage Fault\r\n");
    }
    else
    {
        DBG_APP_INFO("Hard Fault\r\n");
    }
    
    DBG_APP_INFO("*** Stack Frame ***\r\n");
    Cy_SysLib_Delay(1);
    DBG_APP_INFO("CFSR: %x\r\n", cy_faultFrame.cfsr.cfsrReg);
    Cy_SysLib_Delay(1);
    DBG_APP_INFO("HFSR: %x\r\n", cy_faultFrame.hfsr.hfsrReg);
    Cy_SysLib_Delay(1);
    DBG_APP_INFO("SHCSR: %x\r\n", cy_faultFrame.shcsr.shcsrReg);
    Cy_SysLib_Delay(1);
#else
    DBG_APP_INFO("*** Stack Frame ***\r\n");
    Cy_SysLib_Delay(1);
#endif /* CY_CPU_CORTEX_M4 */
    DBG_APP_INFO("r0: %x\r\n", cy_faultFrame.r0);
    Cy_SysLib_Delay(1);
    DBG_APP_INFO("r1: %x\r\n", cy_faultFrame.r1);
    Cy_SysLib_Delay(1);
    DBG_APP_INFO("r2: %x\r\n", cy_faultFrame.r2);
    Cy_SysLib_Delay(1);
    DBG_APP_INFO("r3: %x\r\n", cy_faultFrame.r3);
    Cy_SysLib_Delay(1);
    DBG_APP_INFO("r12: %x\r\n", cy_faultFrame.r12);
    Cy_SysLib_Delay(1);
    DBG_APP_INFO("lr: %x\r\n", cy_faultFrame.lr);
    Cy_SysLib_Delay(1);
    DBG_APP_INFO("pc: %x\r\n", cy_faultFrame.pc);
    Cy_SysLib_Delay(1);
    DBG_APP_INFO("psr: %x\r\n", cy_faultFrame.psr);
    
    while(true) {}
}

static uint32_t wdt_timeout_to_ignore_bits(uint32_t *timeout_ms)
{
    for (uint32_t i = 0; i <= WDT_MAX_IGNORE_BITS; i++)
    {
        if (*timeout_ms >= wdt_ignore_data[i].round_threshold_ms)
        {
            if (*timeout_ms < wdt_ignore_data[i].min_period_ms)
            {
                *timeout_ms = wdt_ignore_data[i].min_period_ms;
            }
            return i;
        }
    }
    return WDT_MAX_IGNORE_BITS; // Ideally should never reach this
}

static uint16_t wdt_timeout_to_match(uint16_t timeout_ms, uint16_t ignore_bits)
{
    uint32_t timeout = ((uint32_t)timeout_ms * CY_SYSCLK_ILO_FREQ) / 1000;
    return (uint16_t)(timeout - (1UL << (17 - ignore_bits)) + Cy_WDT_GetCount());
}

void InitializeWDT (uint32_t timeout_ms)
{
    wdt_ignore_bits = wdt_timeout_to_ignore_bits(&timeout_ms);
    wdt_rounded_timeout_ms = timeout_ms;

    Cy_WDT_Unlock();
    Cy_WDT_Disable();
    Cy_WDT_MaskInterrupt();
    Cy_WDT_SetIgnoreBits(wdt_ignore_bits);

    Cy_WDT_SetMatch(wdt_timeout_to_match(wdt_rounded_timeout_ms, wdt_ignore_bits));

    Cy_WDT_Unlock();
    Cy_WDT_Enable();
    Cy_WDT_Lock();
}

void KickWDT(void)
{
    Cy_WDT_ClearWatchdog(); /* Clear to prevent reset from WDT */
    Cy_WDT_Unlock();
    Cy_WDT_SetMatch(wdt_timeout_to_match(wdt_rounded_timeout_ms, wdt_ignore_bits));
    Cy_WDT_Lock();
}

#if defined(__cplusplus)
}
#endif

/* End of file */

