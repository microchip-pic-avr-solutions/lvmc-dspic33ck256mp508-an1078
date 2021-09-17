/*******************************************************************************
  Motor Control library inline definitions header file

  Company:
    Microchip Technology Inc.

  File Name:
    motor_control_inline_dspic.h

  Summary:
    This header file hosts inline definitions of certain library functions included 
    in the Motor Control library.

  Description:
    This header file hosts inline definitions of certain library functions included 
    in the Motor Control library. This header file is automatically included when the
    library interfaces header file is included in the project.
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

#ifndef _MOTOR_CONTROL_INLINE_DSPIC_H_
#define _MOTOR_CONTROL_INLINE_DSPIC_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
/*  This section lists the other files that are included in this file.
*/
#include <xc.h>
#include <stdint.h>
#include "motor_control_util.h"
#include "motor_control_inline_internal.h"

// Declarations for DSP registers
#include "motor_control_dsp.h"

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
#ifdef __dsPIC33F__
__psv__ extern uint16_t MC_SineTableInFlash[] __attribute__((space(psv)));
#elif defined(__dsPIC33E__) || defined(__dsPIC33C__)
__eds__ extern uint16_t MC_SineTableInFlash[] __attribute__((space(psv)));
#else
#error The selected device is not compatible with the Motor Control library!
#endif

extern uint16_t       MC_SineTableInRam[];

enum {
   /** CORCON bit definitions  */
   MC_LATENCY_CONTROL     = 0x8000,
   MC_UNSIGNED_MULTIPLY   = 0x1000,
   MC_MIXED_MULTPLY       = 0x2000,
   MC_DO_lOOP_TERMINATION = 0x800,
   MC_DO_LOOP_7           = 0x700,
   MC_DO_LOOP_6           = 0x600,
   MC_DO_LOOP_5           = 0x500,
   MC_DO_LOOP_4           = 0x400,
   MC_DO_LOOP_3           = 0x300,
   MC_DO_LOOP_2           = 0x200,
   MC_DO_LOOP_1           = 0x100, 
   MC_SAT_ACCA_ENABLE     = 0x80,
   MC_SAT_ACCB_ENABLE     = 0x40,
   MC_SAT_DATASPACE       = 0x20,
   MC_SAT_SUPER_MODE      = 0x10,
   MC_CPU_HIGH_PRIORITY   = 0x8,
   MC_STACK_FRAME_ACTIVE  = 0x4,
   MC_ROUND_MODE_BIASED   = 0x2,
   MC_INTEGER_MODE        = 0x1,
};

enum {
   /** saturation enabled for ACCA and ACCB, DSP to data space write enabled, Biased rounding mode */
   MC_CORECONTROL = ( MC_SAT_ACCA_ENABLE | MC_SAT_ACCB_ENABLE | MC_SAT_DATASPACE | MC_ROUND_MODE_BIASED ) 
};
// *****************************************************************************
// *****************************************************************************
// Section: Inline function definitions
// *****************************************************************************
// *****************************************************************************
/*  This section lists inline implementation of the library functions.
*/

static inline uint16_t MC_TransformPark_InlineC( const MC_ALPHABETA_T *alphabeta, const MC_SINCOS_T *sincos, MC_DQ_T *dq)
{
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    /* Id =  Ialpha*cos(Angle) + Ibeta*sin(Angle) */
    a_Reg = __builtin_mpy(alphabeta->alpha, sincos->cos,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, alphabeta->beta, sincos->sin,0,0,0,0,0,0,0,0);
    dq->d = __builtin_sacr(a_Reg,0);

    /* Iq = - Ialpha*sin(Angle) + Ibeta*cos(Angle) */
    a_Reg = __builtin_mpy(alphabeta->beta, sincos->cos,0,0,0,0,0,0);
    a_Reg = __builtin_msc(a_Reg, alphabeta->alpha, sincos->sin,0,0,0,0,0,0,0,0);
    dq->q = __builtin_sacr(a_Reg,0);

    CORCON = mcCorconSave;
    return 1;
}

       
static inline uint16_t MC_ControllerPIUpdate_InlineC(int16_t in_Ref, int16_t in_Meas, MC_PISTATE_T *state, int16_t *out)
{
    int16_t error;
    int16_t out_Buffer;
    int16_t output;

    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    /* Calculate error */
    a_Reg = __builtin_lac(in_Ref,0);
    b_Reg = __builtin_lac(in_Meas,0);
    a_Reg = __builtin_subab(a_Reg,b_Reg);
    error = __builtin_sacr(a_Reg,0);

    /* Read state->integrator into B */
    MC_UTIL_writeAccB32(state->integrator);

    /* Calculate (Kp * error * 2^4), store in A and out_Buffer */
    a_Reg = __builtin_mpy(error, state->kp,0,0,0,0,0,0);
    a_Reg = __builtin_sftac(a_Reg,-4);
    a_Reg = __builtin_addab(a_Reg,b_Reg);
    out_Buffer = __builtin_sacr(a_Reg,0);

    /* Limit the output */
    if(out_Buffer > state->outMax)
    {
        output = state->outMax;
    }
    else if(out_Buffer < state->outMin)
    {
        output = state->outMin;
    }
    else
    {
        output = out_Buffer;
    }
    *out = output;    

    /* Calculate (error * Ki) and store in A */
    a_Reg = __builtin_mpy(error, state->ki,0,0,0,0,0,0);

    /* Calculate (excess * Kc), subtract from (error * Ki) and store in A */
    error = out_Buffer - output;
    a_Reg = __builtin_msc(a_Reg, error, state->kc,0,0,0,0,0,0,0,0);

    /* Add (error * Ki)-(excess * Kc) to the integrator value in B */
    a_Reg = __builtin_addab(a_Reg,b_Reg);
    asm volatile ("" : "+w"(a_Reg):); // Prevent optimization from re-ordering/ignoring this sequence of operations

    /* Store the integrator result */
    state->integrator = MC_UTIL_readAccA32();
    CORCON = mcCorconSave;
    
    return 1;
}


static inline uint16_t MC_TransformClarke_InlineC( const MC_ABC_T *abc, MC_ALPHABETA_T *alphabeta)
{
    const uint16_t MC_ONEBYSQ3 = 18919u;
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    /* alpha = a */
    alphabeta->alpha = abc->a;

    /* beta = a/sqrt(3) + 2*b/sqrt(3) */
    a_Reg = __builtin_mpy(abc->a, MC_ONEBYSQ3,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, MC_ONEBYSQ3, abc->b,0,0,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, MC_ONEBYSQ3, abc->b,0,0,0,0,0,0,0,0);
    alphabeta->beta = __builtin_sacr(a_Reg,0);

    CORCON = mcCorconSave;
    return 1;
}


static inline void MC_TransformClarkeABC_InlineC(const MC_ABC_T *abc, MC_ALPHABETA_T *alphabeta)
{
    const int16_t tan30Q16      = 37837U;  // 1/sqrt(3) Q16
    const int16_t one_thirdQ16  = 21845U;  // 1/3 Q16
    const int16_t two_thirdsQ16 = 43690U;  // 2/3 Q16 
    /* note that 43691 would be more accurate but this maintains equal gain among the phases */
    
    /* alpha = a*2/3 - b/3- c/3 */
    alphabeta->alpha = MC_UTIL_mulus16(two_thirdsQ16, abc->a)
                     - MC_UTIL_mulus16(one_thirdQ16, abc->b)
                     - MC_UTIL_mulus16(one_thirdQ16, abc->c);
    
    /* beta = b/sqrt(3) - c/sqrt(3) */   
    alphabeta->beta  = MC_UTIL_mulus16(tan30Q16, abc->b)
                     - MC_UTIL_mulus16(tan30Q16, abc->c);
}

static inline uint16_t MC_TransformParkInverse_InlineC(const MC_DQ_T *dq, const MC_SINCOS_T *sincos, MC_ALPHABETA_T *alphabeta)
{
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    /* alphabeta->alpha = (dq->d * sincos->cos) - (dq->q * sincos->sin) */
    a_Reg = __builtin_mpy(dq->d, sincos->cos,0,0,0,0,0,0);
    a_Reg = __builtin_msc(a_Reg, dq->q, sincos->sin,0,0,0,0,0,0,0,0);
    alphabeta->alpha = __builtin_sacr(a_Reg,0);

    /* alphabeta->beta = (dq->d * sincos->sin) + (dq->q * sincos->cos) */
    a_Reg = __builtin_mpy(dq->d, sincos->sin,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, dq->q, sincos->cos,0,0,0,0,0,0,0,0);
    alphabeta->beta = __builtin_sacr(a_Reg,0);

    CORCON = mcCorconSave;
    return 1;
}


static inline uint16_t MC_TransformClarkeInverseSwappedInput_InlineC( const MC_ALPHABETA_T *alphabeta, MC_ABC_T *abc)
{
    const int16_t MC_SQ3OV2 = 28378u;  
    const int16_t MC_POINT5 = 0x4000;
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    /* a = beta */
    abc->a = alphabeta->beta;

    /* b = (-beta/2) + (Sqrt(3)/2)*alpha */
    a_Reg = __builtin_clr();
    a_Reg = __builtin_msc(a_Reg, alphabeta->beta, MC_POINT5,0,0,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, alphabeta->alpha, MC_SQ3OV2,0,0,0,0,0,0,0,0);
    abc->b = __builtin_sacr(a_Reg,0);

    /* c = (-beta/2) - (Sqrt(3)/2)*alpha */
    a_Reg = __builtin_clr();
    a_Reg = __builtin_msc(a_Reg, alphabeta->beta, MC_POINT5,0,0,0,0,0,0,0,0);
    a_Reg = __builtin_msc(a_Reg, alphabeta->alpha, MC_SQ3OV2,0,0,0,0,0,0,0,0);
    abc->c = __builtin_sacr(a_Reg,0);

    CORCON = mcCorconSave;
    return 1;
}

static inline uint16_t MC_TransformClarkeInverse_InlineC( const MC_ALPHABETA_T *alphabeta, MC_ABC_T *abc)
{
    const int16_t MC_SQ3OV2 = 28378u; 
    const int16_t MC_NEGPOINT5 = 0xC000; 
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    /* a = alpha */
    abc->a = alphabeta->alpha;
    
    /* b = (-alpha/2) + (Sqrt(3)/2)*beta */
    a_Reg = __builtin_mpy(alphabeta->alpha, MC_NEGPOINT5,0,0,0,0,0,0);
    a_Reg = __builtin_mac(a_Reg, alphabeta->beta, MC_SQ3OV2,0,0,0,0,0,0,0,0);
    abc->b = __builtin_sacr(a_Reg,0);

    /* c = (-alpha/2) - (Sqrt(3)/2)*beta */
    a_Reg = __builtin_mpy(alphabeta->alpha, MC_NEGPOINT5,0,0,0,0,0,0);
    a_Reg = __builtin_msc(a_Reg, alphabeta->beta, MC_SQ3OV2,0,0,0,0,0,0,0,0);
    abc->c = __builtin_sacr(a_Reg,0);

    CORCON = mcCorconSave;
    return 1;
}

static inline void MC_TransformClarkeInverseNoAccum_InlineC( const MC_ALPHABETA_T *alphabeta, MC_ABC_T *abc)
{
    /* cos 30 deg = sqrt(3)/2 */
    const uint16_t cos30Q16 = 56756u;

	const int16_t alpha_sin30 = alphabeta->alpha >> 1;
	const int16_t beta_cos30 = MC_UTIL_mulus16(cos30Q16, alphabeta->beta);

    /* a = alpha */
    abc->a = alphabeta->alpha;	
    /* b = -(alpha/2) + (Sqrt(3)/2)*beta */
    abc->b = -alpha_sin30 + beta_cos30;
    /* c = -(alpha/2) - (Sqrt(3)/2)*beta */
    abc->c = -alpha_sin30 - beta_cos30;
}

static inline uint16_t MC_CalculateSpaceVectorPhaseShifted_InlineC( const MC_ABC_T *abc, uint16_t period, MC_DUTYCYCLEOUT_T *pdcout)
{
    int16_t T1, T2, Ta, Tb, Tc;
    
    uint16_t mcCorconSave = CORCON;
    CORCON = MC_CORECONTROL;

    if (abc->a >= 0)
    {
        // (xx1)
        if (abc->b >= 0)
        {
            // (x11)
            // Must be Sector 3 since Sector 7 not allowed
            // Sector 3: (0,1,1)  0-60 degrees
            T1 = abc->a;
            T2 = abc->b;
                /* T1 = period * T1 */
                a_Reg = __builtin_mulus(period, T1);
                T1 = __builtin_sacr(a_Reg,0);
                /* T2 = period * T2 */
                a_Reg = __builtin_mulus(period, T2);
                T2 = __builtin_sacr(a_Reg,0);
                Tc = period-T1-T2;
                Tc = Tc >> 1;
                Tb = Tc + T1;
                Ta = Tb + T2;
            pdcout->dutycycle1 = Ta;
            pdcout->dutycycle2 = Tb;
            pdcout->dutycycle3 = Tc;
        }
        else
        {
            // (x01)
            if (abc->c >= 0)
            {
                // Sector 5: (1,0,1)  120-180 degrees
                T1 = abc->c;
                T2 = abc->a;
                    /* T1 = period * T1 */
                    a_Reg = __builtin_mulus(period, T1);
                    T1 = __builtin_sacr(a_Reg,0);
                    /* T2 = period * T2 */
                    a_Reg = __builtin_mulus(period, T2);
                    T2 = __builtin_sacr(a_Reg,0);
                    Tc = period-T1-T2;
                    Tc = Tc >> 1;
                    Tb = Tc + T1;
                    Ta = Tb + T2;
                pdcout->dutycycle1 = Tc;
                pdcout->dutycycle2 = Ta;
                pdcout->dutycycle3 = Tb;
            }
            else
            {
                // Sector 1: (0,0,1)  60-120 degrees
                T1 = -abc->c;
                T2 = -abc->b;
                    /* T1 = period * T1 */
                    a_Reg = __builtin_mulus(period, T1);
                    T1 = __builtin_sacr(a_Reg,0);
                    /* T2 = period * T2 */
                    a_Reg = __builtin_mulus(period, T2);
                    T2 = __builtin_sacr(a_Reg,0);
                    Tc = period-T1-T2;
                    Tc = Tc >> 1;
                    Tb = Tc + T1;
                    Ta = Tb + T2;
                pdcout->dutycycle1 = Tb;
                pdcout->dutycycle2 = Ta;
                pdcout->dutycycle3 = Tc;
            }
        }
    }
    else
    {
        // (xx0)
        if (abc->b >= 0)
        {
            // (x10)
            if (abc->c >= 0)
            {
                // Sector 6: (1,1,0)  240-300 degrees
                T1 = abc->b;
                T2 = abc->c;
                    /* T1 = period * T1 */
                    a_Reg = __builtin_mulus(period, T1);
                    T1 = __builtin_sacr(a_Reg,0);
                    /* T2 = period * T2 */
                    a_Reg = __builtin_mulus(period, T2);
                    T2 = __builtin_sacr(a_Reg,0);
                    Tc = period-T1-T2;
                    Tc = Tc >> 1;
                    Tb = Tc + T1;
                    Ta = Tb + T2;
                pdcout->dutycycle1 = Tb;
                pdcout->dutycycle2 = Tc;
                pdcout->dutycycle3 = Ta;
            }
            else
            {
                // Sector 2: (0,1,0)  300-0 degrees
                T1 = -abc->a;
                T2 = -abc->c;
                    /* T1 = period * T1 */
                    a_Reg = __builtin_mulus(period, T1);
                    T1 = __builtin_sacr(a_Reg,0);
                    /* T2 = period * T2 */
                    a_Reg = __builtin_mulus(period, T2);
                    T2 = __builtin_sacr(a_Reg,0);
                    Tc = period-T1-T2;
                    Tc = Tc >> 1;
                    Tb = Tc + T1;
                    Ta = Tb + T2;
                pdcout->dutycycle1 = Ta;
                pdcout->dutycycle2 = Tc;
                pdcout->dutycycle3 = Tb;
            }
        }
        else
        {
            // (x00)
            // Must be Sector 4 since Sector 0 not allowed
            // Sector 4: (1,0,0)  180-240 degrees
            T1 = -abc->b;
            T2 = -abc->a;
                /* T1 = period * T1 */
                a_Reg = __builtin_mulus(period, T1);
                T1 = __builtin_sacr(a_Reg,0);
                /* T2 = period * T2 */
                a_Reg = __builtin_mulus(period, T2);
                T2 = __builtin_sacr(a_Reg,0);
                Tc = period-T1-T2;
                Tc = Tc >> 1;
                Tb = Tc + T1;
                Ta = Tb + T2;
            pdcout->dutycycle1 = Tc;
            pdcout->dutycycle2 = Tb;
            pdcout->dutycycle3 = Ta;
        }
    }

    CORCON = mcCorconSave;
    return 1;
}

static inline void MC_CalculateZeroSequenceModulation_InlineC(const MC_ABC_T *pabc_in, MC_ABC_T *pabc_out, int16_t min, int16_t max)
{
    const int16_t center_out = MC_UTIL_AverageS16(min, max);    
    const MC_minmax16_t minmax_in = MC_UTIL_MinMax3_S16(pabc_in->a, pabc_in->b, pabc_in->c);
    const int16_t center_in = MC_UTIL_AverageS16(minmax_in.min, minmax_in.max);
    pabc_out->a = MC_adjust_zero_sequence(pabc_in->a - center_in, center_out, min, max);
    pabc_out->b = MC_adjust_zero_sequence(pabc_in->b - center_in, center_out, min, max);
    pabc_out->c = MC_adjust_zero_sequence(pabc_in->c - center_in, center_out, min, max);
}

static inline uint16_t MC_CalculateSineCosine_InlineC_Ram( int16_t angle, MC_SINCOS_T *sincos )
{
    uint16_t remainder, index, y0, y1, delta, return_value;
    uint32_t result;

    return_value = 0;
    
    /* Index = (Angle*128)/65536 */
    result = __builtin_muluu(128,angle);
    index = result >> 16;
    remainder = (uint16_t) result ;

    /* Check if interpolation is required or not */
    if(remainder == 0)
    {
        /* No interpolation required, use index only */
        sincos->sin = MC_SineTableInRam[index];
        index = index+32;
        if (index > 127)
        {
            index = index - 128;
        }    
        sincos->cos = MC_SineTableInRam[index];
        return_value = 1;
    }
    else
    {
        /* Interpolation required. Determine the delta between indexed value
         * and the next value from the mcSineTableInRam and scale the remainder 
         * with delta to get the linear interpolated value. */

        y0 = MC_SineTableInRam[index];
        index = index+1;
        if (index > 127)
        {
            index = index - 128;
        }
        y1 = MC_SineTableInRam[index];
        delta = y1 - y0;
        result = __builtin_mulus(remainder,delta);
        sincos->sin = y0 + ( result >>16 );

        /* Increment by 32 for cosine index. Increment by 31 here
         * since index has already been incremented once. */
        index = index+31;
        if (index > 127)
        {
            index = index - 128;
        }
        
        y0 = MC_SineTableInRam[index];
        index = index+1;
        if (index > 127)
        {
            index = index - 128;
        }
        y1 = MC_SineTableInRam[index];
        delta = y1 - y0;
        result = __builtin_mulus(remainder,delta);
        sincos->cos = y0 + ( result >>16);
        return_value = 2;
    }
    return  return_value ;
}

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif

#endif // _MOTOR_CONTROL_INLINE_DSPIC_H_




