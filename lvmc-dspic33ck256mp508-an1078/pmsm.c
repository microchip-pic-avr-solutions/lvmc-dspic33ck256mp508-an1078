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
#ifdef __XC16__  // See comments at the top of this header file
#include <xc.h>
#endif // __XC16__
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libq.h>  

#include "motor_control_noinline.h"

#include "general.h"   
#include "userparms.h"

#include "control.h"   
#include "smcpos.h"

#include "fdweak.h"

#include "hal/measure.h"


#include "hal/board_service.h"

#include "diagnostics.h"

#include "singleshunt.h"

volatile UGF_T uGF;

CTRL_PARM_T ctrlParm;
MOTOR_STARTUP_DATA_T motorStartUpData; 

volatile int16_t thetaElectrical = 0,thetaElectricalOpenLoop = 0;
uint16_t pwmPeriod;

MC_ALPHABETA_T valphabeta,ialphabeta;
MC_SINCOS_T sincosTheta;
MC_DQ_T vdq,idq;
MC_DUTYCYCLEOUT_T pwmDutycycle;
MC_ABC_T   vabc,iabc;

MC_PIPARMIN_T piInputIq;
MC_PIPARMOUT_T piOutputIq;
MC_PIPARMIN_T piInputId;
MC_PIPARMOUT_T piOutputId;
MC_PIPARMIN_T piInputOmega;
MC_PIPARMOUT_T piOutputOmega;



SMC smc1 = SMC_DEFAULTS;

uint16_t  trans_counter = 0;
int16_t Theta_error = 0;// This value is used to transition from open loop to closed loop. 
						// At the end of open loop ramp, there is a difference between 
						// forced angle and estimated angle. This difference is stored in 
						// Theta_error, and added to estimated theta (smc1.Theta) so the 
						// effective angle used for commutating the motor is the same at 
						// the end of open loop, and at the beginning of closed loop. 
						// This Theta_error is then subtracted from estimated theta 
						// gradually in increments of 0.05 degrees until the error is less
						// than 0.05 degrees.
int16_t PrevTheta = 0;	// Previous theta which is then subtracted from Theta to get
						// delta theta. This delta will be accumulated in AccumTheta, and
						// after a number of accumulations Omega is calculated.
int16_t AccumTheta = 0;	// Accumulates delta theta over a number of times
uint16_t AccumThetaCnt = 0;	// Counter used to calculate motor speed. Is incremented
						// in SMC_Position_Estimation() subroutine, and accumulates
						// delta Theta. After N number of accumulations, Omega is 
						// calculated. This N is diIrpPerCalc which is defined in
						// UserParms.h.

volatile uint16_t adcDataBuffer;
MCAPP_MEASURE_T measureInputs;
void InitControlParameters(void);
void DoControl(void);
void CalculateParkAngle(void);
void ResetParmeters(void);

// *****************************************************************************
/* Function:
   main()

  Summary:
    main() function

  Description:
    program entry point, calls the system initialization function group 
    containing the buttons polling loop

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */

int main ( void )
{
    /* Initialize Peripherals */
    Init_Peripherals();

	/* Turn on LED1(LD10) to indicate the device is programmed */
    LED1 = 1;

    DiagnosticsInit();

    BoardServiceInit();

    CORCONbits.SATA = 1;
    CORCONbits.SATB = 1;
    CORCONbits.ACCSAT = 1;

    CORCONbits.SATA = 0;

    while(1)
    {
        /* Initialize PI control parameters */
        InitControlParameters();

        /* Initialize SMC estimator parameters */
        SMCInit(&smc1);

        /* Initialize flux weakening parameters */
        FWInit();

        /* Reset parameters used for running motor through Inverter A */
        ResetParmeters();
        
        while(1)
        {
            DiagnosticsStepMain();
            BoardService();
            if (IsPressed_Button1())
            {
                if (uGF.bits.RunMotor == 1)
                {
                    ResetParmeters();
					trans_counter = 0;
                    LED2 = 0;
                }
                else
                {
					EnablePWMOutputs();
                    uGF.bits.RunMotor = 1;
                    LED2 = 1;
                }

            }
            // Monitoring for Button 2 press in LVMC
                if (IsPressed_Button2())
                {
                    if ((uGF.bits.RunMotor == 1) && (uGF.bits.OpenLoop == 0))
                    {
                        uGF.bits.ChangeSpeed = !uGF.bits.ChangeSpeed;
                    }
                }


        }//inner while loop

    }//outer while loop
    // should never get here
    while(1){}
}// End of Main loop
// *****************************************************************************
/* Function:
    ResetParmeters()

  Summary:
    This routine resets all the parameters required for Motor through Inv-A

  Description:
    Reinitializes the duty cycle,resets all the counters when restarting motor

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void ResetParmeters(void)
{
    /* Make sure ADC does not generate interrupt while initializing parameters*/
	DisableADCInterrupt();
    
#ifdef SINGLE_SHUNT
    /* Initialize Single Shunt Related parameters */
    SingleShunt_InitializeParameters(&singleShuntParam);
    PWM_TRIGA = ADC_SAMPLING_POINT;
    PWM_TRIGB = LOOPTIME_TCY>>1;
    PWM_TRIGC = LOOPTIME_TCY-1;
    PWM_PHASE3 = MIN_DUTY;
    PWM_PHASE2 = MIN_DUTY;
    PWM_PHASE1 = MIN_DUTY;
#else
    PWM_TRIGA = ADC_SAMPLING_POINT;
#endif    
    /* Re initialize the duty cycle to minimum value */
    PWM_PDC3 = MIN_DUTY;
    PWM_PDC2 = MIN_DUTY;
    PWM_PDC1 = MIN_DUTY;
	DisablePWMOutputs();
    
    /* Stop the motor   */
    uGF.bits.RunMotor = 0;
    /* Set the reference speed value to 0 */
    ctrlParm.qVelRef = 0;
    /* Restart in open loop */
    uGF.bits.OpenLoop = 1;
    /* Change speed */
    uGF.bits.ChangeSpeed = 0;
    /* Change mode */
    uGF.bits.ChangeMode = 1;

   /* Initialize PI control parameters */
    InitControlParameters();        
    /* Initialize estimator parameters */
    SMCInit(&smc1);
    /* Initialize flux weakening parameters */
    FWInit();
    /* Initialize measurement parameters */
    MCAPP_MeasureCurrentInit(&measureInputs);
    MCAPP_MeasureAvgInit(&measureInputs.MOSFETTemperature,
            MOSFET_TEMP_AVG_FILTER_SCALE);

    /* Enable ADC interrupt and begin main loop timing */
    ClearADCIF();
	adcDataBuffer = ClearADCIF_ReadADCBUF();
    EnableADCInterrupt();
}
// *****************************************************************************
/* Function:
    DoControl()

  Summary:
    Executes one PI iteration for each of the three loops Id,Iq,Speed

  Description:
    This routine executes one PI iteration for each of the three loops
    Id,Iq,Speed

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void DoControl( void )
{
    /* Temporary variables for sqrt calculation of q reference */
    volatile int16_t temp_qref_pow_q15;

    if (uGF.bits.OpenLoop)
    {
        /* OPENLOOP:  force rotating angle,Vd and Vq */
        if (uGF.bits.ChangeMode)
        {
            /* Just changed to open loop */
            uGF.bits.ChangeMode = 0;

            /* Synchronize angles */
            /* VqRef & VdRef not used */
            ctrlParm.qVqRef = 0;
            ctrlParm.qVdRef = 0;
            
            // Initialize SMC
			smc1.Valpha = 0;
			smc1.Ealpha = 0;
			smc1.EalphaFinal = 0;
			smc1.Zalpha = 0;
			smc1.EstIalpha = 0;
			smc1.Vbeta = 0;
			smc1.Ebeta = 0;
			smc1.EbetaFinal = 0;
			smc1.Zbeta = 0;
			smc1.EstIbeta = 0;
			smc1.Ialpha = 0;
			smc1.IalphaError = 0;
			smc1.Ibeta = 0;
			smc1.IbetaError = 0;
			smc1.Theta = 0;
			smc1.Omega = 0;

            /* Reinitialize variables for initial speed ramp */
            motorStartUpData.startupLock = 0;
            motorStartUpData.startupRamp = 0;

            #ifdef TUNING
                /* Tuning speed ramp value  */
                motorStartUpData.tuningAddRampup = 0;
                /* tuning speed ramp increase delay */
                motorStartUpData.tuningDelayRampup = 0;
            #endif
        }
        /* Speed reference */
        ctrlParm.qVelRef = Q_CURRENT_REF_OPENLOOP;
        /* q current reference is equal to the velocity reference 
         while d current reference is equal to 0
        for maximum startup torque, set the q current to maximum acceptable 
        value represents the maximum peak value */
        ctrlParm.qVqRef = ctrlParm.qVelRef;

        /* PI control for Q */
        piInputIq.inMeasure = idq.q;
        piInputIq.inReference = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;

        /* PI control for D */
        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d = piOutputId.out;

    }
    else
    /* Closed Loop Vector Control */
    {
        /* if change speed indication, double the speed */
        if (uGF.bits.ChangeSpeed)
        {
           /* Potentiometer value is scaled between NOMINALSPEED_ELECTR and 
             * MAXIMUMSPEED_ELECTR to set the speed reference*/
            ctrlParm.targetSpeed = (__builtin_mulss(measureInputs.potValue,
                    MAXIMUMSPEED_ELECTR-NOMINALSPEED_ELECTR)>>15)+
                    NOMINALSPEED_ELECTR;
        }
        else
        {
             /* Potentiometer value is scaled between ENDSPEED_ELECTR 
             * and NOMINALSPEED_ELECTR to set the speed reference*/
            
            ctrlParm.targetSpeed = (__builtin_mulss(measureInputs.potValue,
                    NOMINALSPEED_ELECTR-ENDSPEED_ELECTR)>>15) +
                    ENDSPEED_ELECTR;
        }
        
        if (ctrlParm.speedRampCount < SPEEDREFRAMP_COUNT)
        {
           ctrlParm.speedRampCount++; 
        }
        else
        {
            /* Ramp generator to limit the change of the speed reference
              the rate of change is defined by CtrlParm.qRefRamp */
            ctrlParm.qDiff = ctrlParm.qVelRef - ctrlParm.targetSpeed;
            /* Speed Ref Ramp */
            if (ctrlParm.qDiff < 0)
            {
                /* Set this cycle reference as the sum of
                previously calculated one plus the reference ramp value */
                ctrlParm.qVelRef = ctrlParm.qVelRef+ctrlParm.qRefRamp;
            }
            else
            {
                /* Same as above for speed decrease */
                ctrlParm.qVelRef = ctrlParm.qVelRef-ctrlParm.qRefRamp;
            }
            /* If difference less than half of ref ramp, set reference
            directly from the pot */
            if (_Q15abs(ctrlParm.qDiff) < (ctrlParm.qRefRamp << 1))
            {
                ctrlParm.qVelRef = ctrlParm.targetSpeed;
            }
            ctrlParm.speedRampCount = 0;
        }
        /* Tuning is generating a software ramp
        with sufficiently slow ramp defined by 
        TUNING_DELAY_RAMPUP constant */
        #ifdef TUNING
            /* if delay is not completed */
            if (motorStartUpData.tuningDelayRampup > TUNING_DELAY_RAMPUP)
            {
                motorStartUpData.tuningDelayRampup = 0;
            }
            /* While speed less than maximum and delay is complete */
            if ((motorStartUpData.tuningAddRampup < (MAXIMUMSPEED_ELECTR - ENDSPEED_ELECTR)) &&
                                                  (motorStartUpData.tuningDelayRampup == 0) )
            {
                /* Increment ramp add */
                motorStartUpData.tuningAddRampup++;
            }
            motorStartUpData.tuningDelayRampup++;
            /* The reference is continued from the open loop speed up ramp */
            ctrlParm.qVelRef = ENDSPEED_ELECTR +  motorStartUpData.tuningAddRampup;
        #endif

        if ( uGF.bits.ChangeMode )
        {
            /* Just changed from open loop */
            uGF.bits.ChangeMode = 0;
            piInputOmega.piState.integrator = (int32_t)ctrlParm.qVqRef << 13;
            ctrlParm.qVelRef = ENDSPEED_ELECTR;
        }

        /* If TORQUE MODE skip the speed controller */
        #ifndef	TORQUE_MODE
            /* Execute the velocity control loop */
			piInputOmega.inMeasure = smc1.OmegaFltred;
            piInputOmega.inReference = ctrlParm.qVelRef;
            MC_ControllerPIUpdate_Assembly(piInputOmega.inReference,
                                           piInputOmega.inMeasure,
                                           &piInputOmega.piState,
                                           &piOutputOmega.out);
            ctrlParm.qVqRef = piOutputOmega.out;
        #else
            ctrlParm.qVqRef = ctrlParm.qVelRef;
        #endif

        /* Flux weakening control - the actual speed is replaced 
        with the reference speed for stability 
        reference for d current component 
        adapt the estimator parameters in concordance with the speed */
        ctrlParm.qVdRef=FieldWeakening(_Q15abs(ctrlParm.qVelRef));

        /* PI control for D */
        piInputId.inMeasure = idq.d;
        piInputId.inReference  = ctrlParm.qVdRef;
        MC_ControllerPIUpdate_Assembly(piInputId.inReference,
                                       piInputId.inMeasure,
                                       &piInputId.piState,
                                       &piOutputId.out);
        vdq.d    = piOutputId.out;

         /* Dynamic d-q adjustment with d component priority
		 	Vector limitation
		 	Vd is not limited
		 	Vq is limited so the vector Vs is less than a maximum of 95%.
		 	Vs = SQRT(Vd^2 + Vq^2) < 0.95
		 	Vq = SQRT(0.95^2 - Vd^2)*/
        temp_qref_pow_q15 = (int16_t)(__builtin_mulss(piOutputId.out ,
                                                      piOutputId.out) >> 15);
        temp_qref_pow_q15 = Q15(MAX_VOLTAGE_VECTOR) - temp_qref_pow_q15;
        piInputIq.piState.outMax = _Q15sqrt(temp_qref_pow_q15);
        piInputIq.piState.outMin = -piInputIq.piState.outMax;

        /* PI control for Q */
        piInputIq.inMeasure  = idq.q;
        piInputIq.inReference  = ctrlParm.qVqRef;
        MC_ControllerPIUpdate_Assembly(piInputIq.inReference,
                                       piInputIq.inMeasure,
                                       &piInputIq.piState,
                                       &piOutputIq.out);
        vdq.q = piOutputIq.out;
    }
}
// *****************************************************************************
/* Function:
    _ADCInterrupt()
  Summary:
    _ADCInterrupt() ISR routine

  Description:
    Does speed calculation and executes the vector update loop
    The ADC sample and conversion is triggered by the PWM period.
    The speed calculation assumes a fixed time interval between calculations.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void __attribute__((__interrupt__,no_auto_psv)) _ADCInterrupt()
{
#ifdef SINGLE_SHUNT 
    if (IFS4bits.PWM1IF ==1)
    {
        singleShuntParam.adcSamplePoint = 0;
        IFS4bits.PWM1IF = 0;
    }    
    /* If single shunt algorithm is enabled, two ADC interrupts will be
     serviced every PWM period in order to sample current twice and
     be able to reconstruct the three phases */

    switch(singleShuntParam.adcSamplePoint)
    {
        case SS_SAMPLE_BUS1:
            /*Set Trigger to measure BusCurrent Second sample during PWM 
              Timer is counting up*/
            singleShuntParam.adcSamplePoint = 1;  
            /* Ibus is measured and offset removed from measurement*/
            singleShuntParam.Ibus1 = (int16_t)(ADCBUF_IBUS) - 
                                            measureInputs.current.offsetIbus;                        
        break;

        case SS_SAMPLE_BUS2:
            /*Set Trigger to measure BusCurrent first sample during PWM 
              Timer is counting up*/
            PWM_TRIGA = ADC_SAMPLING_POINT;
            singleShuntParam.adcSamplePoint = 0;
            /* this interrupt corresponds to the second trigger and 
                save second current measured*/
            /* Ibus is measured and offset removed from measurement*/
            singleShuntParam.Ibus2 = (int16_t)(ADCBUF_IBUS) - 
                                            measureInputs.current.offsetIbus;
			/*Common Software Trigger for POT,VBUS and MOSFET Temp*/								
            ADCON3Lbits.SWCTRG = 1; 
        break;

        default:
        break;  
    }
#endif
    if ( uGF.bits.RunMotor )
    {
        if (singleShuntParam.adcSamplePoint == 0)
        {
            measureInputs.current.Ia = ADCBUF_IPHASE1;
            measureInputs.current.Ib = ADCBUF_IPHASE2;
            
#ifdef SINGLE_SHUNT
        
        /* Reconstruct Phase currents from Bus Current*/  
        SingleShunt_PhaseCurrentReconstruction(&singleShuntParam);
        MCAPP_MeasureCurrentCalibrate(&measureInputs);
        iabc.a = singleShuntParam.Ia;
        iabc.b = singleShuntParam.Ib;
#else
        /* Calculate qIa,qIb */
        MCAPP_MeasureCurrentCalibrate(&measureInputs);
        iabc.a = measureInputs.current.Ia;
        iabc.b = measureInputs.current.Ib;
#endif
        /* Calculate qIalpha,qIbeta from qIa,qIb */
        MC_TransformClarke_Assembly(&iabc,&ialphabeta);

        /* Calculate qId,qIq from qSin,qCos,qIalpha,qIbeta */
        MC_TransformPark_Assembly(&ialphabeta,&sincosTheta,&idq);

        /* Speed and field angle estimation */
        smc1.Ialpha = ialphabeta.alpha;
        smc1.Ibeta = ialphabeta.beta;
        smc1.Valpha = valphabeta.alpha;
        smc1.Vbeta = valphabeta.beta;
        SMC_Position_Estimation_Inline(&smc1);

        /* Calculate control values */
        DoControl();
        /* Calculate qAngle */
        CalculateParkAngle();
        /* if open loop */
        if (uGF.bits.OpenLoop == 1)
        {
            /* the angle is given by park parameter */
            thetaElectrical = thetaElectricalOpenLoop;
        }
        else
        {
            /* if closed loop, angle generated by estimator */
            thetaElectrical = smc1.Theta + Theta_error;
        }

		/* Calculate qSin,qCos from the thetaElectrical */
        MC_CalculateSineCosine_Assembly_Ram(thetaElectrical,&sincosTheta);

		/* Calculate qValpha,qVbeta from qSin,qCos,qVd,qVq */
        MC_TransformParkInverse_Assembly(&vdq,&sincosTheta,&valphabeta);

		/* Calculate qVa,qVb,qVc vectors from qValpha,qVbeta */
        MC_TransformClarkeInverseSwappedInput_Assembly(&valphabeta,&vabc);
        
#ifdef SINGLE_SHUNT
       SingleShunt_CalculateSpaceVectorPhaseShifted(&vabc,pwmPeriod,&singleShuntParam);

       PWMDutyCycleSetDualEdge(&singleShuntParam.pwmDutycycle1,&singleShuntParam.pwmDutycycle2); 
#else
		/* Generate SV-PWM from the voltage vectors and PWM frequency */
        MC_CalculateSpaceVectorPhaseShifted_Assembly(&vabc,pwmPeriod,
                                                    &pwmDutycycle);
        PWMDutyCycleSet(&pwmDutycycle);
        
#endif
        }
    }
        
    else
    {
        PWM_TRIGA = ADC_SAMPLING_POINT;
#ifdef SINGLE_SHUNT
        PWM_TRIGB = LOOPTIME_TCY>>1;
        PWM_TRIGC = LOOPTIME_TCY-1;
        singleShuntParam.pwmDutycycle1.dutycycle3 = MIN_DUTY;
        singleShuntParam.pwmDutycycle1.dutycycle2 = MIN_DUTY;
        singleShuntParam.pwmDutycycle1.dutycycle1 = MIN_DUTY;
        singleShuntParam.pwmDutycycle2.dutycycle3 = MIN_DUTY;
        singleShuntParam.pwmDutycycle2.dutycycle2 = MIN_DUTY;
        singleShuntParam.pwmDutycycle2.dutycycle1 = MIN_DUTY;
        PWMDutyCycleSetDualEdge(&singleShuntParam.pwmDutycycle1,
                &singleShuntParam.pwmDutycycle2);
#else
        pwmDutycycle.dutycycle3 = MIN_DUTY;
        pwmDutycycle.dutycycle2 = MIN_DUTY;
        pwmDutycycle.dutycycle1 = MIN_DUTY;
        PWMDutyCycleSet(&pwmDutycycle);
#endif
        
    }
    if (singleShuntParam.adcSamplePoint == 0)
    {
        if (uGF.bits.RunMotor == 0)
        {
            measureInputs.current.Ia = ADCBUF_IPHASE1;
            measureInputs.current.Ib = ADCBUF_IPHASE2; 
            measureInputs.current.Ibus = ADCBUF_IBUS;
        }
        if (MCAPP_MeasureCurrentOffsetStatus(&measureInputs) == 0)
        {
            MCAPP_MeasureCurrentOffset(&measureInputs);
        }
        else
        {
            BoardServiceStepIsr();
        }
        measureInputs.potValue = (int16_t)( ADCBUF_SPEED_REF>>1);
        measureInputs.dcBusVoltage = (int16_t)( ADCBUF_VBUS>>1);
        MCAPP_MeasureTemperature(&measureInputs,(int16_t)(ADCBUF_MOSFET_TEMP>>1));
        DiagnosticsStepIsr();
    }
	/*Reading ADC Buffer and Clearing Interrupt Flag*/
    adcDataBuffer = ClearADCIF_ReadADCBUF();
    ClearADCIF();
}
// *****************************************************************************
/* Function:
    CalculateParkAngle ()

  Summary:
    Function calculates the angle for open loop control

  Description:
    Generate the start sine waves feeding the motor terminals
    Open loop control, forcing the motor to align and to start speeding up .
 
  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void CalculateParkAngle(void)
{
    /* if open loop */
    if (uGF.bits.OpenLoop)
    {
        /* begin with the lock sequence, for field alignment */
        if (motorStartUpData.startupLock < LOCK_TIME)
        {
            motorStartUpData.startupLock += 1;
        }
        /* Then ramp up till the end speed */
        else if (motorStartUpData.startupRamp < END_SPEED)
        {
            motorStartUpData.startupRamp += OPENLOOP_RAMPSPEED_INCREASERATE;
        }
        /* Switch to closed loop */
        else 
        {
            #ifndef OPEN_LOOP_FUNCTIONING
                uGF.bits.ChangeMode = 1;
                uGF.bits.OpenLoop = 0;
            #endif
            Theta_error = thetaElectricalOpenLoop - smc1.Theta;
        }
        /* The angle set depends on startup ramp */
        thetaElectricalOpenLoop += (int16_t)(motorStartUpData.startupRamp >> 
                                    STARTUPRAMP_THETA_OPENLOOP_SCALER);

    }
    /* Switched to closed loop */
    else 
    {
        if ( (abs(Theta_error) > _0_05DEG)&&(trans_counter == 0))
        {
            if (Theta_error < 0)
                Theta_error += _0_05DEG;
            else
                Theta_error -= _0_05DEG;
        }       
    }
}
// *****************************************************************************
/* Function:
    InitControlParameters()

  Summary:
    Function initializes control parameters

  Description:
    Initialize control parameters: PI coefficients, scaling constants etc.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitControlParameters(void)
{


    ctrlParm.qRefRamp = SPEEDREFRAMP;
    ctrlParm.speedRampCount = SPEEDREFRAMP_COUNT;
    trans_counter = 0;

    /* Set PWM period to Loop Time */
    pwmPeriod = LOOPTIME_TCY;

    /* PI - Id Current Control */
    piInputId.piState.kp = D_CURRCNTR_PTERM;
    piInputId.piState.ki = D_CURRCNTR_ITERM;
    piInputId.piState.kc = D_CURRCNTR_CTERM;
    piInputId.piState.outMax = D_CURRCNTR_OUTMAX;
    piInputId.piState.outMin = -piInputId.piState.outMax;
    piInputId.piState.integrator = 0;
    piOutputId.out = 0;

    /* PI - Iq Current Control */
    piInputIq.piState.kp = Q_CURRCNTR_PTERM;
    piInputIq.piState.ki = Q_CURRCNTR_ITERM;
    piInputIq.piState.kc = Q_CURRCNTR_CTERM;
    piInputIq.piState.outMax = Q_CURRCNTR_OUTMAX;
    piInputIq.piState.outMin = -piInputIq.piState.outMax;
    piInputIq.piState.integrator = 0;
    piOutputIq.out = 0;

    /* PI - Speed Control */
    piInputOmega.piState.kp = SPEEDCNTR_PTERM;
    piInputOmega.piState.ki = SPEEDCNTR_ITERM;
    piInputOmega.piState.kc = SPEEDCNTR_CTERM;
    piInputOmega.piState.outMax = SPEEDCNTR_OUTMAX;
    piInputOmega.piState.outMin = -piInputOmega.piState.outMax;
    piInputOmega.piState.integrator = 0;
    piOutputOmega.out = 0;
}

/*PWM Interrupt for PCI - Over current fault detection*/
void __attribute__((__interrupt__,no_auto_psv)) _PWMInterrupt()
{
    ResetParmeters();
    ClearPWMPCIFault();
    ClearPWMIF(); 
}