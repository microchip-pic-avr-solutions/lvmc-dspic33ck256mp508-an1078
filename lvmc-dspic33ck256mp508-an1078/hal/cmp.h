// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * cmp.h
 *
 * This header file lists interface functions - configuring and enabling the 
 * Comparator Modules and its output
 * 
 * Definitions in this file are for dsPIC33CK256MP508.
 * 
 * Component: HAL - CMP
 * 
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">
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
// </editor-fold>

#ifndef __CMP_H
#define __CMP_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
    
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__

#include <stdint.h>
#include <stdbool.h>

// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
                
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
            
void CMP_Initialize(void);

void CMP1_ModuleEnable(bool);
void CMP1_ReferenceSet(uint16_t );

// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __CMP_H
    