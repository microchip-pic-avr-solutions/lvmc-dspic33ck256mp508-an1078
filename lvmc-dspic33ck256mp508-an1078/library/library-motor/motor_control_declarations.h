/*******************************************************************************
  Motor Control Library Function Declaration Header File

  File Name:
    motor_control_declarations.h

  Summary:
    This header file lists all the function declarations used by the Motor Control library.

  Description:
    This header file lists all the function declarations used by the Motor Control library.
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
/*******************************************************************************
Note:
* Some of the function declarations have a MC_ATTRB prefix. This prefix has been
  provided as a placeholder for adding attributes for supporting future versions
  of the compiler.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _MOTOR_CONTROL_DECLARATIONS_H_    // Guards against multiple inclusion
#define _MOTOR_CONTROL_DECLARATIONS_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include "motor_control_types.h"

#if __XC16_VERSION__ > 1011     // Place-holder function attribute prefix
#define MC_ATTRB
#else
#define MC_ATTRB 
#endif

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    uint16_t MC_CalculateSineCosine_Assembly_Ram(int16_t angle, MC_SINCOS_T *pSinCos)

  Summary:    
    This function calculates the Sine and Cosine values for a specified angle input.
    
  Description:
    This function calculates the Sine and Cosine values for specified angle input using 
    linear interpolation on a sine table of 128 words. This routine works the same for 
    both integer input and 1.15 scaling input.

  Precondition:
    None.

  Parameters:
    angle           - Input - This parameter is the input angle which will be used to calculate 
                      the Sine and Cosine components.
    pSinCos         - Output - This parameter is a pointer to a MC_SINCOS_T type structure to
                      which the Sine and Cosine components of the angle are written.

  Returns:
    Unsigned integer value '1' for direct look up and '2' for interpolation.

  Example:
    <code>
    uint16_t temp;
    int16_t angle;
    MC_SINCOS_T mcSinCos;
    temp = MC_CalculateSineCosine_Assembly_Ram(angle, &mcSinCos);    
    </code>  
    
  Remarks:
    For integer scaling the Angle is scaled such that 0 <= Angle < 2*pi corresponds
    to 0 <= Ang < 0xFFFF. The resulting Sine and Cosine values are returned scaled to
    -32769 -> 32767 i.e. (0x8000 -> 0x7FFF).
    For 1.15 scaling the Angle is scaled such that -pi <= Angle < pi corresponds to
    -1 -> 0.9999 i.e. (0x8000 <= Ang < 0x7FFF). The resulting Sine and Cosine values are
    returned scaled to -1 -> 0.9999 i.e. (0x8000 -> 0x7FFF).   
 *******************************************************************************/ 
uint16_t MC_ATTRB MC_CalculateSineCosine_Assembly_Ram(int16_t angle,
                                                      MC_SINCOS_T *pSinCos);

/*******************************************************************************
  Function:
    uint16_t MC_TransformParkInverse_Assembly( const MC_DQ_T *pDQ, const MC_SINCOS_T *pSinCos,
                                                                    MC_ALPHABETA_T *pAlphaBeta)

  Summary:
    This function calculates the inverse Park transformation.

  Description:
    This function calculates the inverse Park transform on a pair of stationary reference frame inputs.
    Inverse park transformation is performed as described by the equation:
    <code>
    alpha = d*cos - q*sin
    beta = d*sin + q*cos
    </code>

  Precondition:
    None.

  Parameters:
    pDQ             - Input - This parameter is a pointer to a MC_DQ_T type structure.
    pSinCos         - Input - This parameter is a pointer to a MC_SINCOS_T type structure.
    pAlphaBeta      - Output - This parameter is a pointer to a MC_ALPHABETA_T type structure.

  Returns:
    Unsigned integer value '1'.

  Example:
    <code>
    MC_DQ_T mcVDQ;    
    MC_SINCOS_T mcSinCos;    
    MC_ALPHABETA_T mcVAlphaBeta;   
    temp = MC_TransformParkInverse_Assembly(&mcVDQ, &mcSinCos, &mcVAlphaBeta);
    </code>

  Remarks:
    This routine works for any q format so long as it is consistent across input and output.
*******************************************************************************/
uint16_t MC_ATTRB MC_TransformParkInverse_Assembly( const MC_DQ_T *pDQ,
                                                              const MC_SINCOS_T *pSinCos,
                                                              MC_ALPHABETA_T *pAlphaBeta);

/*******************************************************************************
  Function:
    uint16_t MC_TransformClarkeInverseSwappedInput_Assembly( const MC_ALPHABETA_T *pAlphaBeta, MC_ABC_T *pABC)

  Summary:
    This function calculates the scaled reference vectors using inputs in an alpha-beta reference frame.

  Description:
    This function calculates the scaled reference vectors in an a-b-c reference frame using
    inputs from an alpha-beta reference frame, as described by the equation: 
    <code>
    a = beta
    b = -beta/2 + (sqrt(3)/2) * alpha
    c = -beta/2 - (sqrt(3)/2) * alpha
    </code>
    
    This is a modified variant of the inverse Clarke transformation where alpha & beta are
    swapped compared to the normal inverse Clarke transformation. This function is designed
    to work with the CalculateSpaceVectorPhaseShifted() in order to simplify the calculation of
    three-phase duty cycle values from a given set of inputs in the alpha-beta reference frame.

  Precondition:
    None.

  Parameters:
    pAlphaBeta      - Input - This parameter is a pointer to a MC_ALPHABETA_T type structure.
    pABC            - Output - This parameter is a pointer to a MC_ABC_T type structure.

  Returns:
    Unsigned integer value '1'.

  Example:
    <code>
    MC_ALPHABETA_T mcVAlphaBeta;    
    MC_ABC_T mcVabc;
    temp = MC_TransformClarkeInverseSwappedInput_Assembly(&mcVAlphaBeta, &mcVabc);
    </code>

  Remarks:
    This routine works for any q format so long as it is consistent across input and output.
*******************************************************************************/
uint16_t MC_ATTRB MC_TransformClarkeInverseSwappedInput_Assembly( const MC_ALPHABETA_T *pAlphaBeta,
                                                                  MC_ABC_T *pABC);


/*******************************************************************************
  Function:
    uint16_t MC_TransformClarkeInverse_Assembly( const MC_ALPHABETA_T *pAlphaBeta, MC_ABC_T *pABC)

  Summary:
    This function calculates the scaled reference vectors using inputs in an alpha-beta reference frame.

  Description:
    This function calculates the scaled reference vectors in an a-b-c reference frame using
    inputs from an alpha-beta reference frame, as described by the equation: 
    <code>
    a = alpha
    b = -alpha/2 + (sqrt(3)/2) * beta
    c = -alpha/2 - (sqrt(3)/2) * beta
    </code>
    
    This is the conventional form of the inverse Clarke transformation. This function is designed
    to work with the CalculateSpaceVector().

  Precondition:
    None.

  Parameters:
    pAlphaBeta      - Input - This parameter is a pointer to a MC_ALPHABETA_T type structure.
    pABC            - Output - This parameter is a pointer to a MC_ABC_T type structure.

  Returns:
    Unsigned integer value '1'.

  Example:
    <code>
    MC_ALPHABETA_T mcVAlphaBeta;    
    MC_ABC_T mcVabc;
    temp = MC_TransformClarkeInverse_Assembly(&mcVAlphaBeta, &mcVabc);
    </code>

  Remarks:
    This routine works for any q format so long as it is consistent across input and output.
*******************************************************************************/
uint16_t MC_ATTRB MC_TransformClarkeInverse_Assembly( const MC_ALPHABETA_T *pAlphaBeta,
                                                                     MC_ABC_T *pABC);

/*******************************************************************************
  Function:
    void MC_TransformClarkeInverseNoAccum_Assembly( const MC_ALPHABETA_T *pAlphaBeta, MC_ABC_T *pABC)

  Summary:
    This function calculates the scaled reference vectors using inputs in an alpha-beta reference frame.

  Description:
    This function calculates the scaled reference vectors in an a-b-c reference frame using
    inputs from an alpha-beta reference frame, as described by the equation: 
    <code>
    a = alpha
    b = -alpha/2 + (sqrt(3)/2) * beta
    c = -alpha/2 - (sqrt(3)/2) * beta
    </code>
    
    This is the conventional form of the inverse Clarke transformation. This function is designed
    to work with the function CalculateSpaceVector() for duty cycle generation. It does not make use
    of the DSP accumulators.

  Precondition:
    None.

  Parameters:
    pAlphaBeta      - Input - This parameter is a pointer to a MC_ALPHABETA_T type structure.
    pABC            - Output - This parameter is a pointer to a MC_ABC_T type structure.

  Returns:
    void

  Example:
    <code>
    MC_ALPHABETA_T mcVAlphaBeta;    
    MC_ABC_T mcVabc;
    MC_TransformClarkeInverseNoAccum_Assembly(&mcVAlphaBeta, &mcVabc);
    </code>

  Remarks:
    This routine works for any q format so long as it is consistent across input and output.
*******************************************************************************/
void MC_ATTRB MC_TransformClarkeInverseNoAccum_Assembly( const MC_ALPHABETA_T *pAlphaBeta,
                                                                    MC_ABC_T *pABC);

/*******************************************************************************
  Function:
    uint16_t MC_CalculateSpaceVectorPhaseShifted_Assembly( const MC_ABC_T *pABC, uint16_t iPwmPeriod, MC_DUTYCYCLEOUT_T *pDutyCycleOut)

  Summary:
    This function calculates the duty cycle values based on the three scaled
    reference vectors in the a-b-c reference frame and the PWM period value.

  Description:
    This function calculates the duty cycle values based on the three scaled reference
    vectors in the a-b-c reference frame and the PWM period value. 

    This function is designed to work with the TransformClarkeInverseSwappedInput() 
    in order to simplify the calculation of three-phase duty cycle values from a given 
    set of inputs in the alpha-beta reference frame.
    This function uses a reference axis that is phase shifted by 30 degrees relative
    to the standard Space Vector Modulation reference axis. This phase-shifted reference
    axis is accommodated by using reference vector inputs from a modified version of
    the inverse Clarke transform which swaps the alpha-beta values at its input.

  Precondition:
    None.

  Parameters:
    pABC            - Input - This parameter is a pointer to a MC_ABC_T type structure.
    iPwmPeriod      - Input - This parameter is an unsigned integer value of the PWM period.
    pDutyCycleOut   - Output - This parameter is a pointer to a MC_DUTYCYCLEOUT_T type structure.

  Returns:
    Unsigned integer value '1'.

  Example:
    <code>
    MC_ABC_T mcVabc;
    uint16_t iPwmPeriod;
    MC_DUTYCYCLEOUT_T mcDutyCycleOut;
    temp = MC_CalculateSpaceVectorPhaseShifted_Assembly(&mcVabc, iPwmPeriod, &mcDutyCycleOut);
    </code>

  Remarks:
    This routine works for any q format so long as it is consistent across input and output.
*******************************************************************************/
uint16_t MC_ATTRB MC_CalculateSpaceVectorPhaseShifted_Assembly( const MC_ABC_T *pABC,
                                                                uint16_t iPwmPeriod,
                                                                MC_DUTYCYCLEOUT_T *pDutyCycleOut);

/*******************************************************************************
  Function:
    uint16_t MC_CalculateSpaceVector_Assembly( const MC_ABC_T *pABC, uint16_t iPwmPeriod,
                                                             MC_DUTYCYCLEOUT_T *pDutyCycleOut)

  Summary:
    This function calculates the duty cycle values based on the three scaled
    reference vectors in the a-b-c reference frame and the PWM period value.

  Description:
    This function calculates the duty cycle values based on the three scaled reference
    vectors in the a-b-c reference frame and the PWM period value. 

    This function works with the conventional Clark inverse transform variants 
    in order to simplify the calculation of three-phase duty cycle values from a given
    set of inputs in the alpha-beta reference frame.
    The duty-cycle generation part of the function is identical to the phase shifted version
    for SVM generation, so the initial part of this function compensates for the  
    swapped input inverse clark function. This ensures the motor rotation direction remains
    the same.

  Precondition:
    None.

  Parameters:
    pABC            - Input - This parameter is a pointer to a MC_ABC_T type structure.
    iPwmPeriod      - Input - This parameter is an unsigned integer value of the PWM period.
    pDutyCycleOut   - Output - This parameter is a pointer to a MC_DUTYCYCLEOUT_T type structure.

  Returns:
    Unsigned integer value '1'.

  Example:
    <code>
    MC_ABC_T mcVabc;
    uint16_t iPwmPeriod;
    MC_DUTYCYCLEOUT_T mcDutyCycleOut;
    temp = MC_CalculateSpaceVector_Assembly(&mcVabc, iPwmPeriod, &mcDutyCycleOut);
    </code>

  Remarks:
    This routine works for any q format so long as it is consistent across input and output.
*******************************************************************************/
uint16_t MC_ATTRB MC_CalculateSpaceVector_Assembly( const MC_ABC_T *pABC,
                                                               uint16_t iPwmPeriod,
                                                               MC_DUTYCYCLEOUT_T *pDutyCycleOut);

/*******************************************************************************
  Function:
    uint16_t MC_TransformClarke_Assembly( const MC_ABC_T *pABC, MC_ALPHABETA_T *pAlphaBeta)

  Summary:
    This function calculates the Clarke transformation.

  Description:
    This function transforms inputs in an a-b-c reference frame to an alpha-beta
    reference frame using the equation:
    <code>
    alpha = a
    beta = a*(1/sqrt(3)) + 2*b*(1/sqrt(3))
    </code>

  Precondition:
    None.

  Parameters:
    pABC            - Input - This parameter is a pointer to a MC_ABC_T type structure.
    pAlphaBeta      - Output - This parameter is a pointer to a MC_ALPHABETA_T type structure.

  Returns:
    Unsigned integer value '1'.

  Example:
    <code>
    MC_ABC_T mcIabc;
    MC_ALPHABETA_T mcIAlphaBeta;    
    temp = MC_TransformClarke_Assembly(&mcIabc, &mcIAlphaBeta);
    </code>  
    
  Remarks:
   This routine works for any q format so long as it is consistent across input and output.
*******************************************************************************/
uint16_t MC_ATTRB MC_TransformClarke_Assembly( const MC_ABC_T *pABC,
                                               MC_ALPHABETA_T *pAlphaBeta);


/*******************************************************************************
  Function:
    uint16_t MC_TransformPark_Assembly( const MC_ALPHABETA_T *pAlphaBeta, const MC_SINCOS_T *pSinCos, MC_DQ_T *pDQ)

  Summary:
    This function calculates the Park transformation.

  Description:
    This function transforms inputs in an alpha-beta reference frame to a stationary
    d-q reference frame using the equation:
    <code>
    d = alpha*cos + beta*sin
    q = -alpha*sin + beta*cos
    </code>

  Precondition:
    None.

  Parameters:
    pAlphaBeta      - Input - This parameter is a pointer to a MC_ALPHABETA_T type structure.
    pSinCos         - Input - This parameter is a pointer to a MC_SINCOS_T type structure.
    pDQ             - Output - This parameter is a pointer to a MC_DQ_T type structure.
    
  Returns:
    Unsigned integer value '1'.

  Example:
    <code>
    MC_ALPHABETA_T mcIAlphaBeta;
    MC_SINCOS_T mcSinCos;
    MC_DQ_T mcIDQ;
    temp = MC_TransformPark_Assembly(&mcIAlphaBeta, &mcSinCos, &mcIDQ);
    </code>  
    
  Remarks:
    This routine works for any q format so long as it is consistent across input and output.
*******************************************************************************/
uint16_t MC_ATTRB MC_TransformPark_Assembly( const MC_ALPHABETA_T *pAlphaBeta,
                                             const MC_SINCOS_T *pSinCos,
                                             MC_DQ_T *pDQ);


/*******************************************************************************
  Function:
    uint16_t MC_ControllerPIUpdate_Assembly(int16_t inReference, int16_t inMeasure, MC_PISTATE_T *pPIState, int16_t *pPIParmOutput)
                                        
  Summary:
    This function calculates the PI correction.

  Description:
    This function calculates a PI correction output from a given measured input and a reference.
    The equation for PI output is:
    <code>
    out = Kp*(inReference-inMeasure) + Ki*Integral[inReference-inMeasure, dt] - Kc*Excess
    </code>
    Where,
    out = Fractional 1.15 output, is limited to between outMax and outMin.
    Kp = Proportional gain co-efficient term
    Ki = Integral gain co-efficient term
    Kc = Excess gain co-efficient term
    Excess = Excess error after "out" is limited to between outMax and outMin.
    This implementation includes an anti-windup term to limit the integral windup.

  Precondition:
    None.

  Parameters:
    inReference     - Input - This parameter is a 1.15 fractional format reference input.
    inMeasure       - Input - This parameter is a 1.15 fractional format measured input.
    pPIState        - Input/Output - This paramater is a pointer to a MC_PISTATE_T type structure.
    pPIParmOutput   - Output - This paramater is a pointer to a signed integer type variable.

  Returns:
    Unsigned integer value '1'.

  Example:
    <code>
    MC_PIPARMIN_T mcPIParmInput;
    MC_PIPARMOUT_T mcPIParmOutput;
    temp = MC_ControllerPIUpdate_Assembly(mcPIParmInput.inReference, mcPIParmInput.inMeasure, &mcPIParmInput.piState, &mcPIParmOutput.out);
    </code>  
    
  Remarks:
    This routine requires inputs in the 1.15 format, except for Kp which is in 1.11 format.
    The constant Kp is scaled so it can be represented in 1.15 format by adjusting the constant 
    by a power of 2.
*******************************************************************************/
uint16_t MC_ATTRB MC_ControllerPIUpdate_Assembly(int16_t inReference,
                                                 int16_t inMeasure,
                                                 MC_PISTATE_T *pPIState,
                                                 int16_t *pPIParmOutput);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif // _MOTOR_CONTROL_DECLARATIONS_H_





