/**********************************************************************
* © 2012 Microchip Technology Inc.
*
* SOFTWARE LICENSE AGREEMENT:
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
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
* IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
* STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
* ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
* ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
* THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and 
* determining its suitability.  Microchip has no obligation to modify, test, 
* certify, or support the code.
*
******************************************************************************/
#include "fdweak.h"
#include "general.h"
#include "smcpos.h"

tFdWeakParm	FdWeakParm;

int16_t FieldWeakening(int16_t qMotorSpeed)
{
    int16_t iTempInt1, iTempInt2;

    /* If the speed is less than one for activating the FW */
    if (qMotorSpeed <= FdWeakParm.qFwOnSpeed) 
    {
        /* Set Idref as first value in magnetizing curve */
        FdWeakParm.qIdRef = FdWeakParm.qFwCurve[0];
    } 
    else 
    {
        /* Get the index parameter */
        /* Index in FW-Table */
        FdWeakParm.qIndex = (qMotorSpeed - FdWeakParm.qFwOnSpeed) >> SPEED_INDEX_CONST;

        iTempInt1 = FdWeakParm.qFwCurve[FdWeakParm.qIndex] -
                    FdWeakParm.qFwCurve[FdWeakParm.qIndex + 1];
        iTempInt2 = (FdWeakParm.qIndex << SPEED_INDEX_CONST) +
                    FdWeakParm.qFwOnSpeed;
        iTempInt2 = qMotorSpeed - iTempInt2;

        /* Interpolation between two results from the Table */
        FdWeakParm.qIdRef = FdWeakParm.qFwCurve[FdWeakParm.qIndex]-
                (int16_t) (__builtin_mulss(iTempInt1, iTempInt2) >> SPEED_INDEX_CONST);
    }
    return FdWeakParm.qIdRef;
}

void FWInit(void)
{
    /* Field Weakening constant for constant torque range */
    /* Flux reference value */
    FdWeakParm.qIdRef = IDREF_BASESPEED;
    /* Start speed for Field weakening  */
    FdWeakParm.qFwOnSpeed = FWONSPEED;

    /* Initialize magnetizing curve values */
    FdWeakParm.qFwCurve[0] = IDREF_SPEED0;
    FdWeakParm.qFwCurve[1] = IDREF_SPEED1;
    FdWeakParm.qFwCurve[2] = IDREF_SPEED2;
    FdWeakParm.qFwCurve[3] = IDREF_SPEED3;
    FdWeakParm.qFwCurve[4] = IDREF_SPEED4;
    FdWeakParm.qFwCurve[5] = IDREF_SPEED5;
    FdWeakParm.qFwCurve[6] = IDREF_SPEED6;
    FdWeakParm.qFwCurve[7] = IDREF_SPEED7;
    FdWeakParm.qFwCurve[8] = IDREF_SPEED8;
    FdWeakParm.qFwCurve[9] = IDREF_SPEED9;
    FdWeakParm.qFwCurve[10] = IDREF_SPEED10;
    FdWeakParm.qFwCurve[11] = IDREF_SPEED11;
    FdWeakParm.qFwCurve[12] = IDREF_SPEED12;
    FdWeakParm.qFwCurve[13] = IDREF_SPEED13;
    FdWeakParm.qFwCurve[14] = IDREF_SPEED14;
    FdWeakParm.qFwCurve[15] = IDREF_SPEED15;
    FdWeakParm.qFwCurve[16] = IDREF_SPEED16;
    FdWeakParm.qFwCurve[17] = IDREF_SPEED17;
}
