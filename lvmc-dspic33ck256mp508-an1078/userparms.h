/*******************************************************************************
* Copyright (c) 2020 released Microchip Technology Inc.  All rights reserved.
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
#ifndef USERPARMS_H
#define USERPARMS_H

#ifdef __cplusplus
extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <xc.h>

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
        
/*************** PWM and Control Timing Parameters ****************************/
/* Specify PWM Frequency in Hertz */
#define PWMFREQUENCY_HZ         20000
/* Specify dead time in micro seconds */
/* Specify PWM Period in seconds, (1/ PWMFREQUENCY_HZ) */
#define LOOPTIME_SEC            0.00005

/* Definition for tuning - if active the speed reference is a ramp with a 
constant slope. The slope is determined by TUNING_DELAY_RAMPUP constant.
 the software ramp implementing the speed increase has a constant slope, 
 adjusted by the delay TUNING_DELAY_RAMPUP when the speed is incremented.
 The potentiometer speed reference is overwritten. The speed is
 increased from 0 up to the END_SPEED_RPM in open loop – with the speed
 increase typical to open loop, the transition to closed loop is done
 and the software speed ramp reference is continued up to MAXIMUM_SPEED_RPM. */
#undef TUNING

/* if TUNING was define, then the ramp speed is needed: */
#ifdef TUNING
    /* the smaller the value, the quicker the ramp */
    #define TUNING_DELAY_RAMPUP   0xF
#endif


/* open loop continuous functioning */
/* closed loop transition disabled  */
#undef OPEN_LOOP_FUNCTIONING

/* Definition for torque mode - for a separate tuning of the current PI
controllers, tuning mode will disable the speed PI controller */
#undef TORQUE_MODE

/* undef to work with dual Shunt  */    
#define SINGLE_SHUNT  
    
/* undef to work with External Op-Amp*/
#define INTERNAL_OPAMP_CONFIG

/****************************** Motor Parameters ******************************/
/********************  support xls file definitions begin *********************/
/* The following values are given in the xls attached file */
// Values used to test Hurst Motor "NT Dynamo DMB0224C10002" at 24VDC input.
// Motor datasheet at www.hurstmfg.com
/* Motor's number of pole pairs */
#define NOPOLESPAIRS 5
/* Open loop speed ramp up end value Value in RPM*/
#define END_SPEED_RPM 500
/* Nominal speed of the motor in RPM */
#define NOMINAL_SPEED_RPM    2000
/* Maximum speed of the motor in RPM - given by the Motor's manufacturer */
#define MAXIMUM_SPEED_RPM    3500

#define FW_NOMINAL_SPEED_RPM 2000

/* The following values are given in the xls attached file */
#define NORM_CURRENT_CONST     0.000671

/* normalized ls/dt value */
#define NORM_LSDTBASE 9738
/* normalized rs value */
#define NORM_LSDTBASE_SCALINGFACTOR 8

#define NORM_RS  27503
#define NORM_RS_SCALINGFACTOR 2

/**********************  support xls file definitions end *********************/

/* current transformation macro, used below */
#define NORM_CURRENT(current_real) (Q15(current_real/NORM_CURRENT_CONST/32768))

/* Open loop startup constants */

/* The following values depends on the PWM frequency,
 lock time is the time needed for motor's poles alignment 
before the open loop speed ramp up */
/* This number is: 20,000 is 1 second. */
#define LOCK_TIME 4000
/* Open loop speed ramp up end value Value in RPM*/
#define END_SPEED_RPM 500
/* Open loop acceleration */
#define OPENLOOP_RAMPSPEED_INCREASERATE 10
/* Open loop q current setup - */
#define Q_CURRENT_REF_OPENLOOP NORM_CURRENT(1.0) 
    
/* Specify Over Current Limit - DC BUS */
#define Q15_OVER_CURRENT_THRESHOLD NORM_CURRENT(3.0)

/* Maximum motor speed converted into electrical speed */
#define MAXIMUMSPEED_ELECTR MAXIMUM_SPEED_RPM*NOPOLESPAIRS
/* Nominal motor speed converted into electrical speed */
#define NOMINALSPEED_ELECTR NOMINAL_SPEED_RPM*NOPOLESPAIRS

/* End speed converted to fit the startup ramp */
#define END_SPEED (END_SPEED_RPM * NOPOLESPAIRS * LOOPTIME_SEC * 65536 / 60.0)*1024
/* End speed of open loop ramp up converted into electrical speed */
#define ENDSPEED_ELECTR END_SPEED_RPM*NOPOLESPAIRS

/* In case of the potentiometer speed reference, a reference ramp
is needed for assuring the motor can follow the reference imposed /
minimum value accepted */
#define SPEEDREFRAMP   Q15(0.00003)

/* The Speed Control Loop Executes every  SPEEDREFRAMP_COUNT */
#define SPEEDREFRAMP_COUNT   3
/* PI controllers tuning values - */

/* D Control Loop Coefficients */
#define D_CURRCNTR_PTERM       Q15(0.02)     
#define D_CURRCNTR_ITERM       Q15(0.001)   
#define D_CURRCNTR_CTERM       Q15(0.999)
#define D_CURRCNTR_OUTMAX      0x7FFF

/* Q Control Loop Coefficients */
#define Q_CURRCNTR_PTERM       Q15(0.02)     
#define Q_CURRCNTR_ITERM       Q15(0.001)   
#define Q_CURRCNTR_CTERM       Q15(0.999)
#define Q_CURRCNTR_OUTMAX      0x7FFF

/* Velocity Control Loop Coefficients */
#define SPEEDCNTR_PTERM        Q15(0.5)
#define SPEEDCNTR_ITERM        Q15(0.005)
#define SPEEDCNTR_CTERM        Q15(0.999)
#define SPEEDCNTR_OUTMAX       0x5000

//***********************SMC Params*********************************************//
#define LOOPTIMEINSEC (1.0/PWMFREQUENCY_HZ) // PWM Period = 1.0 / PWMFREQUENCY
#define SPEEDLOOPFREQ	1000		// Speed loop Frequency in Hertz. This value must
									// be an integer to avoid pre-compiler error
#define SPEEDLOOPTIME (float)(1.0/SPEEDLOOPFREQ) // Speed Control Period
#define IRP_PERCALC (uint16_t)(SPEEDLOOPTIME/LOOPTIMEINSEC)	// PWM loops per velocity calculation
#define TRANSITION_STEPS   IRP_PERCALC/4

#define SMCGAIN			0.85		// Slide Mode Controller Gain (0.0 to 0.9999)
#define MAXLINEARSMC    0.005		// If measured current - estimated current
								// is less than MAXLINEARSMC, the slide mode
								// Controller will have a linear behavior
								// instead of ON/OFF. Value from (0.0 to 0.9999)

#define STARTUPRAMP_THETA_OPENLOOP_SCALER       10

#define MAX_VOLTAGE_VECTOR                      0.98
// Vd and Vq vector limitation variables

#define SMO_SPEED_EST_MULTIPLIER Q15(0.9155273)

#define THETA_FILTER_CNST   Q15(0.104719*LOOPTIME_SEC*32768.0)    //2*pi/60*Ts*32768
    
//***********************End of SMC Params************************************//

/******************************** Field Weakening *****************************/
/* Field Weakening constant for constant torque range 
   Flux reference value */
#define IDREF_BASESPEED         NORM_CURRENT(0.0)

/*-------------------------------------------------------------
   IMPORTANT:--------------------------------------------------
  -------------------------------------------------------------
   In flux weakening of the surface mounted permanent magnets
   PMSMs the mechanical damage of the rotor and the
   demagnetization of the permanent magnets is possible if
   cautions measures are not taken or the motor’s producer
   specifications are not respected.
  -------------------------------------------------------------
   IMPORTANT:--------------------------------------------------
  -------------------------------------------------------------
   In flux weakening regime implementation, if the FOC is lost
   at high speed above the nominal value, the possibility of
   damaging the inverter is eminent. The reason is that the
   BEMF will have a greater value than the one that would be
   obtained for the nominal speed exceeding the DC bus voltage
   value and though the inverter’s power semiconductors and DC
   link capacitors would have to support it. Since the tuning
   proposed implies iterative coefficient corrections until
   the optimum functioning is achieved, the protection of the
   inverter with corresponding circuitry should be assured in
   case of stalling at high speeds.                            */

/* speed index is increase */
#define SPEED_INDEX_CONST 10

#define FWONSPEED FW_NOMINAL_SPEED_RPM*NOPOLESPAIRS

/* the following values indicate the d-current variation with speed 
 please consult app note for details on tuning */
#define	IDREF_SPEED0	NORM_CURRENT(0)     /* up to 2800 RPM */
#define	IDREF_SPEED1	NORM_CURRENT(-0.7)  /* ~2950 RPM */
#define	IDREF_SPEED2	NORM_CURRENT(-0.9)  /* ~3110 RPM */
#define	IDREF_SPEED3	NORM_CURRENT(-1.0)  /* ~3270 RPM */
#define	IDREF_SPEED4	NORM_CURRENT(-1.4)  /* ~3430 RPM */
#define	IDREF_SPEED5	NORM_CURRENT(-1.7)  /* ~3600 RPM */
#define	IDREF_SPEED6	NORM_CURRENT(-2.0)  /* ~3750 RPM */
#define	IDREF_SPEED7	NORM_CURRENT(-2.1)  /* ~3910 RPM */
#define	IDREF_SPEED8	NORM_CURRENT(-2.2)  /* ~4070 RPM */
#define	IDREF_SPEED9	NORM_CURRENT(-2.25) /* ~4230 RPM */
#define	IDREF_SPEED10	NORM_CURRENT(-2.3)  /* ~4380 RPM */
#define	IDREF_SPEED11	NORM_CURRENT(-2.35) /* ~4550 RPM */
#define	IDREF_SPEED12	NORM_CURRENT(-2.4)  /* ~4700 RPM */
#define	IDREF_SPEED13	NORM_CURRENT(-2.45) /* ~4860 RPM */
#define	IDREF_SPEED14	NORM_CURRENT(-2.5)  /* ~5020 RPM */
#define	IDREF_SPEED15	NORM_CURRENT(-2.5)  /* ~5180 RPM */
#define	IDREF_SPEED16	NORM_CURRENT(-2.5)  /* ~5340 RPM */
#define	IDREF_SPEED17	NORM_CURRENT(-2.5)  /* ~5500 RPM */

// This pre-processor condition will generate an error if maximum speed is out of
// range on Q15 when calculating Omega.
#if (FW_NOMINAL_SPEED_RPM < NOMINAL_SPEED_RPM)
	#error FIELDWEAKSPEEDRPM must be greater than NOMINALSPEEDINRPM for field weakening.
	#error if application does not require Field Weakening, set FIELDWEAKSPEEDRPM value
	#error equal to NOMINALSPEEDINRPM
#elif (((FW_NOMINAL_SPEED_RPM*NOPOLESPAIRS*2)/(60*SPEEDLOOPFREQ)) >= 1)
		#error FIELDWEAKSPEEDRPM will generate an Omega value greater than 1 which is the
		#error maximum in Q15 format. Reduce FIELDWEAKSPEEDRPM value, or increase speed
		#error control loop frequency, SPEEDLOOPFREQ
#endif
/******************** End of Field Weakening Params ***************************/

#ifdef __cplusplus
}
#endif

#endif /* __USERPARMS_H */
