/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef __SINGLESHUNT_H
#define	__SINGLESHUNT_H

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

#include <clock.h>     
#include "pwm.h"    
#include "general.h"
#include "userparms.h"    
#include "motor_control_noinline.h" 
      
/* Scaling factor for current Bus Current */
#define KCURRBUS        Q15(-0.5) 
/*  Critical Minimum window in seconds to measure current through single shunt*/  
#define SSTCRITINSEC	3.0E-6	
/* Single shunt algorithm defines *2 because is same resolution as PDCx registers */    							
#define SSTCRIT         (uint16_t)(SSTCRITINSEC*FCY*2)  
/* Single shunt algorithm defines *2 because is same resolution as PDCx registers */    							
#define SS_SAMPLE_DELAY  100  
    
 /* Description:
    This structure will host parameters related to measured currents
 */
typedef struct
{
    int16_t T1;
    int16_t T2;
    int16_t T7;
    int16_t Ta1;
    int16_t Ta2;
    int16_t Tb1;
    int16_t Tb2;
    int16_t Tc1;
    int16_t Tc2;
    int16_t sectorSVM;			// Variable indicating the active sector in which space
                                // vector modulation is in. The Sector value is used to
                                // identify which duty cycle registers to be modified
                                // in order to measure current through single shunt
    int16_t tcrit;			// variable used to create minimum window to measure
                                // current through single shunt resistor when enabled
                                // Define minimum window in UserParms.h, SSTCRITINSEC
    int16_t tDelaySample;		// variable used to create a delay for single shunt
                                // current measurement. Delay is defined in seconds in
                                // the actual dead time definition in UserParms.h,
                                // DEADTIMESEC
    
    int16_t Ia;				/* Reconstructed value for Ia */
    int16_t Ib;				/* Reconstructed value for Ib */
    int16_t Ic;				/* Reconstructed value for Ic */
    int16_t Ibus1;
    int16_t Ibus2;	
    int16_t trigger1;       /* This variable holds the first trigger value 
                               to be stored in TRIG1 register for 
                               A/D conversion. The converted value will be used 
                               for current reconstruction later on. */
    int16_t trigger2;       /* Variable holding the second trigger value to 
                               be stored in TRIG1 register to trigger 
                               A/D conversion */
    int16_t adcSamplePoint;
    MC_DUTYCYCLEOUT_T pwmDutycycle1;
    MC_DUTYCYCLEOUT_T pwmDutycycle2;
    
} SINGLE_SHUNT_PARM_T;

typedef enum tagSSADCSAMPLE_STATE
{ 
    SS_SAMPLE_BUS1 = 0,     /* Bus Current Sample1 */
    SS_SAMPLE_BUS2 = 1,     /* Bus Current Sample2 */   
     
}SSADCSAMPLE_STATE;
			
extern SINGLE_SHUNT_PARM_T singleShuntParam;
uint16_t SingleShunt_CalculateSpaceVectorPhaseShifted(MC_ABC_T *pABC,
                                                     uint16_t iPwmPeriod,
                                                     SINGLE_SHUNT_PARM_T *);
void SingleShunt_PhaseCurrentReconstruction(SINGLE_SHUNT_PARM_T *);
void SingleShunt_InitializeParameters(SINGLE_SHUNT_PARM_T *);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

