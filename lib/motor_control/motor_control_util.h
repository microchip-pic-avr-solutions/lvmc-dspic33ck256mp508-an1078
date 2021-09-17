/* 
 * File:   motor_control_util.h
 * 
 * Utility routines for computation used only in Motor control library
 * 
 */

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

#ifndef _MOTOR_CONTROL_UTIL_H_
#define	_MOTOR_CONTROL_UTIL_H_

#include <stdint.h>

// Declarations for DSP registers
#include "motor_control_dsp.h"    

/*
 * Access to accumulator registers is required
 * only for XC16 1.25 and earlier; this causes
 * a dependency on <xc.h> which is unnecessary
 * for newer versions of the compiler.
 */
#if __XC16_VERSION__ < 1026
#include <xc.h>
#endif


#ifdef	__cplusplus
extern "C" {
#endif


/**
 * Helper function to multiply an unsigned 16-bit quantity
 * and a signed 16-bit quantity and shift-right by 16.
 * (one of the inputs should be a Q16 fixed-point values,
 * and the other one and the output have identical binary points,
 * e.g. Q12 = Q16 * Q12, or Q12 = Q12 * Q16)
 *
 * @param a first input (unsigned)
 * @param b second input (signed)
 * @return (a*b)>>16
 */
inline static int16_t MC_UTIL_mulus16(uint16_t a, int16_t b)
{
    return __builtin_mulus(a,b) >> 16;
}

/**
 * Compute the average of two int16_t values
 * @param a first value
 * @param b second value
 * @return (a+b)/2
 */

inline static int16_t MC_UTIL_AverageS16(int16_t a, int16_t b)
{
    return (int16_t)((((int32_t)a) + b) >> 1);
}

/**
 * Compute the minimum and maximum of a set of three int16_t values
 * @param a first value
 * @param b second value
 * @param c third value
 * @return struct containing minimum and maximum value --
 *   this is fairly unusual but it permits the compiler to
 *   optimize by placing in an appropriate pair
 *   of adjacent working registers.
 */
inline static MC_minmax16_t MC_UTIL_MinMax3_S16(int16_t a, int16_t b, int16_t c)
{
    /* Sort a,b,c */
    asm (
        "    cpslt   %[a], %[b]\n"
        "    exch    %[a], %[b]\n"
        "    cpslt   %[a], %[c]\n"
        "    exch    %[a], %[c]\n"
        "    cpslt   %[b], %[c]\n"
        "    exch    %[b], %[c]\n"
        : [a]"+r"(a),
          [b]"+r"(b),
          [c]"+r"(c)
    );
    /* Now a <= b <= c */

    MC_minmax16_t result;
    result.min = a;
    result.max = c;
    return result;
}

/**  Read accumulator A */
inline static int32_t MC_UTIL_readAccA32()
{
#if __XC16_VERSION__ >= 1030
    return __builtin_sacd(a_Reg, 0);
#elif __XC16_VERSION__ >= 1026
    const int32_t tmp = __builtin_sacd(a_Reg, 0);
    /* Prevent optimization from re-ordering/ignoring this sequence of operations */
    asm volatile ("");
    return tmp;
#else
    int32_t result;
    /* Prevent optimization from re-ordering/ignoring this sequence of operations */
    asm volatile ("" : "+w"(a_Reg):); 
    result = ACCAH;
    result <<= 16;
    result |= ACCAL; 
    return result;
#endif
}

/**  Write accumulator B */
inline static void MC_UTIL_writeAccB32(int32_t input)
{
#if __XC16_VERSION__ >= 1030
    b_Reg = __builtin_lacd(input, 0);
#elif __XC16_VERSION__ >= 1026
    const int32_t tmp = input;
    asm volatile ("" :: "r"(tmp)); 
    b_Reg = __builtin_lacd(tmp, 0);
#else
    uint32_t temp_dword;
    uint16_t temp_word;
    temp_dword = 0xFFFF0000 & input;
    temp_dword = temp_dword >> 16;
    temp_word = (uint16_t)temp_dword;
    b_Reg = __builtin_lac(temp_word, 0);
    /* Prevent optimization from re-ordering/ignoring this sequence of operations */
    asm volatile ("" : "+w"(b_Reg):); 
    temp_word = (uint16_t)(0xFFFF & input);
    ACCBL = temp_word;
#endif

}



#ifdef	__cplusplus
}
#endif

#endif	/* _MOTOR_CONTROL_UTIL_H_ */

