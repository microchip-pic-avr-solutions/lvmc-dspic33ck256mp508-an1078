/*******************************************************************************
  Motor Control library -- DSP registers

  Company:
    Microchip Technology Inc.

  File Name:
    motor_control_dsp.h

  Summary:
    DSP register declarations

  Description:
    Contains DSP register declarations in global variables.
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

#ifndef _MOTOR_CONTROL_DSP_H_
#define _MOTOR_CONTROL_DSP_H_

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
#ifndef DSP_ACCUMULATOR_A_DEFINED		
#define DSP_ACCUMULATOR_A_DEFINED		
/** DSP accumulator A */
volatile register int a_Reg asm("A");
#endif

#ifndef DSP_ACCUMULATOR_B_DEFINED
#define DSP_ACCUMULATOR_B_DEFINED		
/** DSP accumulator B */
volatile register int b_Reg asm("B");
#endif

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif

#endif // _MOTOR_CONTROL_DSP_H_




