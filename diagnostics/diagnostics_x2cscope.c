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

#include "X2CScope.h"
#include "hal/uart1.h"
#include <stdint.h>

#define X2C_BAUDRATE_DIVIDER 54
    /*
     * baud rate = 100MHz/16/(1+baudrate_divider) for highspeed = false
     * 
     * 115.1kbaud => 54 (for DIAG_BAUDRATE_DIVIDER with highspeed = false)
     */

    /*
     * baud rate = 70MHz/16/(1+baudrate_divider) for highspeed = false
     * baud rate = 70MHz/4/(1+baudrate_divider) for highspeed = true
     * 
     * 4375kbaud => 0 (for DIAG_BAUDRATE_DIVIDER with highspeed = false)
     * 2188kbaud => 1
     * 1458kbaud => 2
     * 1094kbaud => 3
     *  875kbaud => 4
     *  729kbaud => 5
     *  625kbaud => 6
     *  547kbaud => 7
     *  486kbaud => 8
     *  437kbaud => 9
     *  397kbaud => 10
     *  364.6kbaud => 11
     *  336.5kbaud => 12
     *  312.5kbaud => 13
     *  291.7kbaud => 14
     *  273.4kbaud => 15
     *  257.3kbaud => 16
     *  243.1kbaud => 17
     *  230.2kbaud => 18
     *  115.1kbaud => 37
     *   57.6kbaud => 75
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
    const uint16_t error_mask = _U1STA_OERR_MASK 
                              | _U1STA_FERR_MASK
                              | _U1STA_PERR_MASK;
    if (UART1_StatusGet() & error_mask)
    {
        UART1_ReceiveBufferOverrunErrorFlagClear();
        return 0;
    }
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
    X2CScope_Initialise();
}