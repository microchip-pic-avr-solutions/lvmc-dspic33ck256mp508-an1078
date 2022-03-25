/*******************************************************************************
  Motor Control Library Types Header File

  File Name:
    motor_control_types.h

  Summary:
    This header file lists all the types used by the Motor Control library.

  Description:
    This header file lists the type defines for structures used by the Motor 
    Control library. 
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

#ifndef _MOTOR_CONTROL_TYPES_H_    // Guards against multiple inclusion
#define _MOTOR_CONTROL_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

/** Min Max values stored together 

   Description:
    minimum and maximum values 
*/
 typedef struct
{
    //  minimum
    int16_t min;
    
    //  maximum
    int16_t max;    
} MC_minmax16_t;

// *****************************************************************************
/** Alpha-Beta reference frame data type

  Description:
    Parameters related to Alpha-Beta reference frame.
*/
typedef struct
{
    // Alpha component
    int16_t alpha;
    
    // Beta component
    int16_t beta;
} MC_ALPHABETA_T;

// *****************************************************************************
/** Sine-Cosine data type

  Description:
    Parameters related to Sine and Cosine components of the motor angle.
*/
typedef struct
{
    // Cosine component
    int16_t cos;
    
    // Sine component
    int16_t sin;
} MC_SINCOS_T;

// *****************************************************************************
/** D-Q reference frame data type

  Description:
    Parameters related to D-Q reference frame.
*/
typedef struct
{
    // D-axis component
    int16_t d;
    
    // Q-axis component
    int16_t q;
} MC_DQ_T;

// *****************************************************************************
/** Duty-cycle data type

  Description:
    Parameters related to PWM module Duty Cycle values.
*/
typedef struct
{
    // Duty cycle for phase #1
    uint16_t dutycycle1;
    
    // Duty cycle for phase #2
    uint16_t dutycycle2;
    
    // Duty cycle for phase #3
    uint16_t dutycycle3;
} MC_DUTYCYCLEOUT_T;

// *****************************************************************************
/** ABC reference frame data type

  Description:
    Parameters related to ABC reference frame.
*/
typedef struct
{
    // Phase A component 
    int16_t a;
    
    // Phase B component
    int16_t b;
    
    // Phase C component
    int16_t c;
} MC_ABC_T;

// *****************************************************************************
/** PI Controller State data type

  Description:
    Parameters related to the PI Controller state.
*/
typedef struct
{
    // Integrator sum
    int32_t integrator;
    
    // Proportional gain co-efficient term
    int16_t kp;
    
    // Integral gain co-efficient term
    int16_t ki;
    
    // Excess gain co-efficient term
    int16_t kc;

    // Maximum output limit
    int16_t outMax;
    
    // Minimum output limit
    int16_t outMin;
} MC_PISTATE_T;

// *****************************************************************************
/** PI Controller Input data type

  Summary:
    PI Controller input type define

  Description:
    Parameters related to the PI Controller input. PI
    controller state is a part of the PI Controller input.
*/
typedef struct
{
    // PI state as input parameter to the PI controller
    MC_PISTATE_T piState;
    
    // Input reference to the PI controller
    int16_t inReference;
    
    // Input measured value
    int16_t inMeasure;
} MC_PIPARMIN_T;

// *****************************************************************************
/** PI Controller Output data type

  Description:
    Parameters related to the PI Controller output.
*/
typedef struct
{
    // Output of the PI controller
    int16_t out;        
} MC_PIPARMOUT_T;

// *****************************************************************************
/** SIN and DIFF table  data type

  Description:
    SIN and the successive difference for the interpolation points
    for the lookup table version of the SINCOS function 
*/
typedef  struct
{
    // offset into the table of SIN entries
    int16_t offset; 
    
    // stores the successive difference of SIN entries
    int16_t diff;    
}MC_SINDIFF_T;


#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif // _MOTOR_CONTROL_TYPES_H_





