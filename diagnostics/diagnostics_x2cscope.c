/**
 * diagnostics.c
 * 
 * Diagnostics code
 * 
 * Component: diagnostics
 */

/*******************************************************************************
* Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
*
* SOFTWARE LICENSE AGREEMENT:
* 
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
* THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
* SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/

#include "library/x2cscope/X2CScope.h"
#include "uart1.h"
#include <stdint.h>

#define X2C_DATA __attribute__((section("x2cscope_data_buf")))
#define X2C_BAUDRATE_DIVIDER 54
#define X2C_BUFFER_SIZE 4900
X2C_DATA static uint8_t X2C_BUFFER[X2C_BUFFER_SIZE];
    /*
     * baud rate = 100MHz/16/(1+baudrate_divider) for highspeed = false
     * baud rate = 100MHz/4/(1+baudrate_divider) for highspeed = true
     * 
     * 6250kbaud => 0 (for DIAG_BAUDRATE_DIVIDER with highspeed = false)
     * 3125kbaud => 1
     * 2083kbaud => 2
     * 1562kbaud => 3
     *  1250kbaud => 4
     *  1041kbaud => 5
     *  892kbaud => 6
     *  781kbaud => 7
     *  694kbaud => 8
     *  625kbaud => 9
     *  568kbaud => 10
     *  520.8kbaud => 11
     *  480.76kbaud => 12
     *  446.4kbaud => 13
     *  416.6kbaud => 14
     *  390.62kbaud => 15
     *  367.6kbaud => 16
     *  347.2kbaud => 17
     *  328.9kbaud => 18
     *  115.7kbaud => 54
     *   57.87kbaud => 107
     */

void X2CScope_Init(void);

void DiagnosticsInit(void)
{
    UART1_InterruptReceiveDisable();
    UART1_InterruptReceiveFlagClear();
    UART1_InterruptTransmitDisable();
    UART1_InterruptTransmitFlagClear();
    UART1_Initialize();
    UART1_BaudRateDividerSet(X2C_BAUDRATE_DIVIDER);
    UART1_SpeedModeStandard();
    UART1_ModuleEnable();  
    
    X2CScope_Init();
}

void DiagnosticsStepMain(void)
{
    X2CScope_Communicate();
}

void DiagnosticsStepIsr(void)
{
    X2CScope_Update();
}

/* ---------- communication primitives used by X2CScope library ---------- */

static void X2CScope_sendSerial(uint8_t data)
{
    UART1_DataWrite(data);
}

static uint8_t X2CScope_receiveSerial()
{
    return UART1_DataRead();
}

static uint8_t X2CScope_isReceiveDataAvailable()
{
    return UART1_IsReceiveBufferDataReady();
}

static uint8_t X2CScope_isSendReady()
{
    return !UART1_StatusBufferFullTransmitGet();
}

void X2CScope_Init(void)
{
    X2CScope_HookUARTFunctions(
        X2CScope_sendSerial,
        X2CScope_receiveSerial,
        X2CScope_isReceiveDataAvailable,
        X2CScope_isSendReady);
    X2CScope_Initialise(X2C_BUFFER,sizeof(X2C_BUFFER));
}