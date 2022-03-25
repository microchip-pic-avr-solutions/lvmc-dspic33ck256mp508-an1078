/*******************************************************************************
  Motor Control Library Interface Header File

  File Name:
    motor_control.h

  Summary:
    This header file lists all the interfaces used by the Motor Control library.

  Description:
    This header file lists the type defines for structures used by the Motor 
    Control library. Library function definitions are also listed along with
    information regarding the arguments of each library function. This header file
    also includes another header file that hosts inline definitions of certain
    library functions.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/* *********************************************************************
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
* Some parts of this header file are protected by #ifdef __XC16__. These protections
  are provided to accommodate non-XC16 compilers to work with this header file.
  Similarly, sections of the header file related to the MATLAB-based compiler are
  protected by #ifdef __MATLAB_MEX__ protections.
* Some of the function declarations have a MC_ATTRB prefix. This prefix has been
  provided as a placeholder for adding attributes for supporting future versions
  of the compiler.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _MOTOR_CONTROL_H_    // Guards against multiple inclusion
#define _MOTOR_CONTROL_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#ifdef __XC16__  // See comments at the top of this header file
#include <xc.h>
#endif // __XC16__

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

#ifdef __MATLAB_MEX__ // See comments at the top of this header file
#define inline
#endif // __MATLAB_MEX


#include "motor_control_declarations.h"
#include "motor_control_inline_declarations.h"


#ifdef __XC16__   // See comments at the top of this header file
#include "./motor_control_inline_dspic.h"
#endif // __XC16__

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif // _MOTOR_CONTROL_H





