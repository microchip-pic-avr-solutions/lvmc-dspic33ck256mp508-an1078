
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
#include <libq.h>
#include "userparms.h"
#include "singleshunt.h"


SINGLE_SHUNT_PARM_T singleShuntParam;
inline static void SingleShunt_CalculateSwitchingTime(SINGLE_SHUNT_PARM_T *,uint16_t );

// *****************************************************************************

/* Function:
    CalcSVGen_ss ()

  Summary:
 Space Vector modulation generation from Va,Vb,Vc

  Description:
    Space Vector modulation generation from Va,Vb,Vc

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void SingleShunt_InitializeParameters(SINGLE_SHUNT_PARM_T *pSingleShunt)
{    
    /* Set minimum window time to measure current through single shunt */
	pSingleShunt->tcrit = SSTCRIT;
    /* Set delay due to dead time and slew rate etc.*/
	pSingleShunt->tDelaySample = SS_SAMPLE_DELAY;	
    /*Trigger  values of Bus Current Samples made equal to zero */
    pSingleShunt->trigger1 = 0;
    pSingleShunt->trigger2 = 0;
        /*Trigger  values of Bus Current Samples made equal to zero */
    pSingleShunt->Ibus1 = 0;
    pSingleShunt->Ibus2 = 0;
    pSingleShunt->adcSamplePoint = 0;
}
// *****************************************************************************

/* Function:
    CalcSVGen_ss ()

  Summary:
 Space Vector modulation generation from Va,Vb,Vc

  Description:
    Space Vector modulation generation from Va,Vb,Vc

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
uint16_t SingleShunt_CalculateSpaceVectorPhaseShifted(MC_ABC_T *abc,
                                    uint16_t iPwmPeriod,
                                    SINGLE_SHUNT_PARM_T *pSingleShunt)
{ 
    uint16_t mcCorconSave = CORCON;
    CORCON = 0x00E2;
    
    MC_DUTYCYCLEOUT_T *pdcout1 = &pSingleShunt->pwmDutycycle1;
    MC_DUTYCYCLEOUT_T *pdcout2 = &pSingleShunt->pwmDutycycle2;   
    if (abc->a >= 0)
    {
        // (xx1)
        if (abc->b >= 0)
        {
            // (x11)
            // Must be Sector 3 since Sector 7 not allowed
            // Sector 3: (0,1,1)  0-60 degrees
            pSingleShunt->sectorSVM  = 3; 
            pSingleShunt->T1 = abc->a;
            pSingleShunt->T2 = abc->b;
            SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
            pdcout1->dutycycle1 = pSingleShunt->Ta1;
            pdcout1->dutycycle2 = pSingleShunt->Tb1;
            pdcout1->dutycycle3 = pSingleShunt->Tc1;
            pdcout2->dutycycle1 = pSingleShunt->Ta2;
            pdcout2->dutycycle2 = pSingleShunt->Tb2;
            pdcout2->dutycycle3 = pSingleShunt->Tc2;
        }
        else
        {
            // (x01)
            if (abc->c >= 0)
            {
                // Sector 5: (1,0,1)  120-180 degrees
                pSingleShunt->sectorSVM  = 5;
                pSingleShunt->T1 = abc->c;
                pSingleShunt->T2 = abc->a;
                SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
                pdcout1->dutycycle1 = pSingleShunt->Tc1;
                pdcout1->dutycycle2 = pSingleShunt->Ta1;
                pdcout1->dutycycle3 = pSingleShunt->Tb1;
                pdcout2->dutycycle1 = pSingleShunt->Tc2;
                pdcout2->dutycycle2 = pSingleShunt->Ta2;
                pdcout2->dutycycle3 = pSingleShunt->Tb2;
            }
            else
            {
                // Sector 1: (0,0,1)  60-120 degrees
                pSingleShunt->sectorSVM  = 1;
                pSingleShunt->T1 = -abc->c;
                pSingleShunt->T2 = -abc->b;
                SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
                pdcout1->dutycycle1 = pSingleShunt->Tb1;
                pdcout1->dutycycle2 = pSingleShunt->Ta1;
                pdcout1->dutycycle3 = pSingleShunt->Tc1;
                pdcout2->dutycycle1 = pSingleShunt->Tb2;
                pdcout2->dutycycle2 = pSingleShunt->Ta2;
                pdcout2->dutycycle3 = pSingleShunt->Tc2;
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
                pSingleShunt->sectorSVM  = 6;
                pSingleShunt->T1 = abc->b;
                pSingleShunt->T2 = abc->c;
                SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
                pdcout1->dutycycle1 = pSingleShunt->Tb1;
                pdcout1->dutycycle2 = pSingleShunt->Tc1;
                pdcout1->dutycycle3 = pSingleShunt->Ta1;
                pdcout2->dutycycle1 = pSingleShunt->Tb2;
                pdcout2->dutycycle2 = pSingleShunt->Tc2;
                pdcout2->dutycycle3 = pSingleShunt->Ta2;
            }
            else
            {
                // Sector 2: (0,1,0)  300-0 degrees
                pSingleShunt->sectorSVM  = 2;
                pSingleShunt->T1 = -abc->a;
                pSingleShunt->T2 = -abc->c;
                SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
                pdcout1->dutycycle1 = pSingleShunt->Ta1;
                pdcout1->dutycycle2 = pSingleShunt->Tc1;
                pdcout1->dutycycle3 = pSingleShunt->Tb1;
                pdcout2->dutycycle1 = pSingleShunt->Ta2;
                pdcout2->dutycycle2 = pSingleShunt->Tc2;
                pdcout2->dutycycle3 = pSingleShunt->Tb2;
            }
        }
        else
        {
            // (x00)
            // Must be Sector 4 since Sector 0 not allowed
            // Sector 4: (1,0,0)  180-240 degrees
            pSingleShunt->sectorSVM  = 4;
            pSingleShunt->T1 = -abc->b;
            pSingleShunt->T2 = -abc->a;
            SingleShunt_CalculateSwitchingTime(&singleShuntParam,iPwmPeriod);
            pdcout1->dutycycle1 = pSingleShunt->Tc1;
            pdcout1->dutycycle2 = pSingleShunt->Tb1;
            pdcout1->dutycycle3 = pSingleShunt->Ta1;
            pdcout2->dutycycle1 = pSingleShunt->Tc2;
            pdcout2->dutycycle2 = pSingleShunt->Tb2;
            pdcout2->dutycycle3 = pSingleShunt->Ta2;
        }
    }
    /* Calculate two triggers for the ADC that will fall in between PWM
        so a valid measurement is done using a single shunt resistor.
        tDelaySample is added as a delay so no erroneous measurement is taken*/
	
    pSingleShunt->trigger1 = (iPwmPeriod + pSingleShunt->tDelaySample);
    pSingleShunt->trigger1 = pSingleShunt->trigger1 - ((pSingleShunt->Ta1 + pSingleShunt->Tb1) >> 1) ;
    pSingleShunt->trigger2 = (iPwmPeriod +  pSingleShunt->tDelaySample);
    pSingleShunt->trigger2 = pSingleShunt->trigger2 - ((pSingleShunt->Tb1 + pSingleShunt->Tc1) >> 1) ;
	PWM_TRIGB = singleShuntParam.trigger1;
    PWM_TRIGC = singleShuntParam.trigger2;
    CORCON = mcCorconSave;
    return(1);
}
// *****************************************************************************

/* Function:
    CalcTimes_ss_Inline ()

  Summary:
 Duty cycle calcuation from T1 and T2 

  Description:
    Duty cycle calcuation from T1 and T2

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
inline static void SingleShunt_CalculateSwitchingTime(SINGLE_SHUNT_PARM_T *pSingleShunt,
        uint16_t iPwmPeriod)
{
	
    // First of all, calculate times corresponding to actual Space Vector
	// modulation output.

    pSingleShunt->T1 = (int16_t) (__builtin_mulss(iPwmPeriod,pSingleShunt->T1) >> 15);
    pSingleShunt->T2 = (int16_t) (__builtin_mulss(iPwmPeriod,pSingleShunt->T2) >> 15);
    pSingleShunt->T7 = (iPwmPeriod-pSingleShunt->T1-pSingleShunt->T2)>>1;

	/* If PWM counter is already counting down, in which case any modification to 
        duty cycles will take effect until PWM counter starts counting up again. 
        This is why the correction of any modifications done during PWM Timer is
        Counting down  will be updated here so that it takes effect during PWM Counter
        counting up.
        If PWM counter is already counting up, in which case any modification to 
        duty cycles will take effect until PWM reaches the maximum value and 
        starts counting down. Modification to pattern is done here if times
        T1 or T2 are lesser than tcrit. Change in duty cycle will take effect
        until PWM timer starts counting down.
	
	
     If T1 is greater than the minimum measurement window tcrit,
     then no modification to the pattern is needed.*/
    if (pSingleShunt->T1 > pSingleShunt->tcrit)
    {
        pSingleShunt->Tc1 = pSingleShunt->T7;
        pSingleShunt->Tc2 = pSingleShunt->T7;
    }
    /* If PWM Timer is counting down and there is not enough time to measure 
    current through single shunt, that is when T1 is lesser than tcrit, 
    then modify pattern by adding a minimum time of  tcrit and compensate the added 
    value by subtracting it from T1 which will take effect when PWM Counter 
    is countin up  */
    else
    {
        pSingleShunt->Tc1 = pSingleShunt->T7 - (pSingleShunt->tcrit-pSingleShunt->T1);
        pSingleShunt->Tc2 = pSingleShunt->T7 + (pSingleShunt->tcrit-pSingleShunt->T1);
    }
    pSingleShunt->Tb1 = pSingleShunt->T7 + pSingleShunt->T1;
    pSingleShunt->Tb2 = pSingleShunt->Tb1;
    // If T2 is greater than the minimum measurement window tcrit,
    // then no modification to the pattern is needed.
    if (pSingleShunt->T2 > pSingleShunt->tcrit)
    {
        pSingleShunt->Ta1 = pSingleShunt->Tb1 + pSingleShunt->T2;
        pSingleShunt->Ta2 = pSingleShunt->Tb2 + pSingleShunt->T2;
    }
     /* If PWM Timer is counting down and there is not enough time to measure 
    current through single shunt, that is when T2 is lesser than tcrit, 
    then modify pattern by adding a minimum time of  tcrit and compensate the added 
    value by subtracting it from T2 which will take effect when PWM Counter 
    is countin up  */
    else
    {
        pSingleShunt->Ta1 = pSingleShunt->Tb1 + pSingleShunt->tcrit;
        pSingleShunt->Ta2 = pSingleShunt->Tb2 + pSingleShunt->T2 + pSingleShunt->T2 - pSingleShunt->tcrit;
    }
	
}    
// *****************************************************************************

/* Function:
    PhaseCurrent_Reconstruction ()

  Summary:
 Phase Currents reconstruction from bus current

  Description:
    Phase Currents reconstruction from bus current

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void SingleShunt_PhaseCurrentReconstruction(SINGLE_SHUNT_PARM_T *pSingleShunt)
{
    switch(pSingleShunt->sectorSVM)
    {
        case 1:
            pSingleShunt->Ib = pSingleShunt->Ibus1;
            pSingleShunt->Ic = -pSingleShunt->Ibus2;
            pSingleShunt->Ia = -pSingleShunt->Ic - pSingleShunt->Ib;
        break;
        case 2:
            pSingleShunt->Ia = pSingleShunt->Ibus1;
            pSingleShunt->Ib = -pSingleShunt->Ibus2;
            pSingleShunt->Ic = -pSingleShunt->Ia - pSingleShunt->Ib;
        break;
        case 3:
            pSingleShunt->Ia = pSingleShunt->Ibus1; 
            pSingleShunt->Ic = -pSingleShunt->Ibus2;
            pSingleShunt->Ib = -pSingleShunt->Ia - pSingleShunt->Ic;
        break;
        case 4:
            pSingleShunt->Ic = pSingleShunt->Ibus1; 
            pSingleShunt->Ia = -pSingleShunt->Ibus2; 
            pSingleShunt->Ib = -pSingleShunt->Ia - pSingleShunt->Ic;
        break;
        case 5:
            pSingleShunt->Ib = pSingleShunt->Ibus1; 
            pSingleShunt->Ia = -pSingleShunt->Ibus2; 
            pSingleShunt->Ic = -pSingleShunt->Ia - pSingleShunt->Ib;
        break;
        case 6:
            pSingleShunt->Ic = pSingleShunt->Ibus1; 
            pSingleShunt->Ib = -pSingleShunt->Ibus2;
            pSingleShunt->Ia = -pSingleShunt->Ic - pSingleShunt->Ib;
        break;   
    }  
}