/*******************************************************************************
  Header file for Motor Control library inline definitions of internal functions  

  Company:
    Microchip Technology Inc.

  File Name:
    motor_control_inline_internal.h

  Summary:
    This header file contains implementation details of the Motor Control Library 
	that are not part of its public interface and are subject to change.
    Please use the functions and type definitions contained in other header files
	of the Motor Control Library instead.

  Description:
    This header file is automatically included when the library interfaces header file
    is included in the project.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/* *********************************************************************
 *
 * (c) 2017 Microchip Technology Inc. and its subsidiaries. You may use
 * this software and any derivatives exclusively with Microchip products.
 *
 * This software and any accompanying information is for suggestion only.
 * It does not modify Microchip's standard warranty for its products.
 * You agree that you are solely responsible for testing the software and
 * determining its suitability.  Microchip has no obligation to modify,
 * test, certify, or support the software.
 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH
 * MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
 * APPLICATION.
 
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL,
 * PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF
 * ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF
 * MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE
 * FORESEEABLE.  TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL
 * LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT
 * EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO
 * MICROCHIP FOR THIS SOFTWARE.
 
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
 * THESE TERMS.
 *
 * *****************************************************************************/
// DOM-IGNORE-END

#ifndef _MOTOR_CONTROL_INLINE_INTERNAL_H_
#define _MOTOR_CONTROL_INLINE_INTERNAL_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.
*/
#include <stdint.h>


/**
 * Compute limit(x + ofs_out, min, max)
 * This optimized code works ONLY if ofs_out >= 0
 */
static inline int16_t MC_adjust_zero_sequence(int16_t x, int16_t ofs_out, int16_t min, int16_t max)
{
    int16_t w;
    asm volatile (
        "    add     %[ofs_out], %[x], %[w]\n" /* overflow is only positive */
        "    cpslt   %[min], %[w]\n"
        "    mov     %[min], %[w]\n"    /* if (w < min) { w = min; } */
        "    btss    SR, #2\n"          /* if ofs_out > 0, OV = SR<2> indicates we need positive clipping */
        "    cpslt   %[w], %[max]\n"
        "    mov     %[max], %[w]\n"
        : [w]"=&r"(w)
        : [x]"r"(x),
          [ofs_out]"r"(ofs_out),
          [min]"r"(min),
          [max]"r"(max)
    );
    return w;
}

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

#endif // _MOTOR_CONTROL_INLINE_INTERNAL_H_

