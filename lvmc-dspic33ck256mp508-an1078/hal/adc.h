/*******************************************************************************
  ADC Configuration Routine Header File

  File Name:
    adc.h

  Summary:
    This header file lists ADC Configuration related functions and definitions

  Description:
    Definitions in the file are for dsPIC33CK256MP508

*******************************************************************************/
/*******************************************************************************
* Copyright (c) 2020 released Microchip Technology Inc.  All rights reserved.
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
#ifndef _ADC_H
#define _ADC_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>

#include "userparms.h"
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// ADC MODULE Related Definitions
#define ADCBUF_IPHASE1    -ADCBUF0
#define ADCBUF_IPHASE2    -ADCBUF1
#define ADCBUF_IBUS       ADCBUF4  
        
#define ADCBUF_SPEED_REF      ADCBUF11
#define ADCBUF_VBUS           ADCBUF15
#define ADCBUF_MOSFET_TEMP    ADCBUF12
        

#ifdef SINGLE_SHUNT       
 #define EnableADCInterrupt()   _ADCAN4IE = 1
 #define DisableADCInterrupt()  _ADCAN4IE = 0
 #define ClearADCIF()           _ADCAN4IF = 0
 #define ClearADCIF_ReadADCBUF() ADCBUF4        
 #define _ADCInterrupt _ADCAN4Interrupt  
#else
 #define EnableADCInterrupt()   _ADCAN11IE = 1
 #define DisableADCInterrupt()  _ADCAN11IE = 0
 #define ClearADCIF()           _ADCAN11IF = 0
 #define ClearADCIF_ReadADCBUF() ADCBUF11        
 #define _ADCInterrupt _ADCAN11Interrupt  
#endif      
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitializeADCs(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of ADC_H

