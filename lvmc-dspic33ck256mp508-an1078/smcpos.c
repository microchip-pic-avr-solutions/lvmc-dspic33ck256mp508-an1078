/**********************************************************************
* © 2020 Microchip Technology Inc.
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
 /**********************************************************************
 *      Code Description                                               *
 *                                                                     *
 *  This file implements a slide mode observer. This observer is used  *
 *  to estimate rotor position and speed. Rotor position, Theta, is    *
 *  then compensated from phase delays introduced by the filters       *
 *                                                                     *
 **********************************************************************/

#include "general.h"
#include "smcpos.h"

MOTOR_ESTIM_PARM_T motorParm;
extern SMC smc1;
ESTIM_PARM_T estimator;

void SMCInit(SMC *s)
{
    //                R * Ts
    // Fsmopos = 1 - --------
    //                  L
    //            Ts
    // Gsmopos = ----
    //            L
    // Ts = Sampling Period. If sampling at PWM, Ts = 50 us
    // R = Phase Resistance. If not provided by motor datasheet,
    //     measure phase to phase resistance with multimeter, and
    //     divide over two to get phase resistance. If 4 Ohms are
    //     measured from phase to phase, then R = 2 Ohms
    // L = Phase inductance. If not provided by motor datasheet,
    //     measure phase to phase inductance with multimeter, and
    //     divide over two to get phase inductance. If 2 mH are
    //     measured from phase to phase, then L = 1 mH
    motorParm.qLsDtBase = NORM_LSDTBASE;
    motorParm.qLsDt = motorParm.qLsDtBase;
    motorParm.qRs = NORM_RS;

	if (((int32_t)motorParm.qRs<<NORM_RS_SCALINGFACTOR) >= ((int32_t)motorParm.qLsDt<<NORM_LSDTBASE_SCALINGFACTOR))

		s->Fsmopos = Q15(0.0);
	else
		s->Fsmopos = (0x7FFF - __builtin_divsd(((int32_t)motorParm.qRs<<(15+NORM_RS_SCALINGFACTOR-NORM_LSDTBASE_SCALINGFACTOR)),motorParm.qLsDt));

    if (((int32_t)motorParm.qLsDt<<NORM_LSDTBASE_SCALINGFACTOR)<32767)
		s->Gsmopos = 0x7FFF;
	else
		s->Gsmopos = __builtin_divsd(((int32_t)0x7FFF<<(15-NORM_LSDTBASE_SCALINGFACTOR)),motorParm.qLsDt);

	s->Kslide = Q15(SMCGAIN);
	s->MaxSMCError = Q15(MAXLINEARSMC);
	s->FiltOmCoef = (int16_t)(__builtin_mulss(ENDSPEED_ELECTR,THETA_FILTER_CNST)>>15);
	return;
}

inline void SMC_Position_Estimation_Inline (SMC* s)
{
    int16_t Kslf_min;
    PUSHCORCON();
	CORCONbits.SATA = 1;
	CORCONbits.SATB = 1;
	CORCONbits.ACCSAT = 1;

    CalcEstI();

	// Sliding control calculator
    // Sliding control filter -> back EMF calculator
	// s->Ealpha = s->Ealpha + s->Kslf * s->Zalpha -
	//						   s->Kslf * s->Ealpha
	// s->Ebeta = s->Ebeta + s->Kslf * s->Zbeta -
	//						 s->Kslf * s->Ebeta
	// s->EalphaFinal = s->EalphaFinal + s->KslfFinal * s->Ealpha
	//								   - s->KslfFinal * s->EalphaFinal
	// s->EbetaFinal = s->EbetaFinal + s->KslfFinal * s->Ebeta
	//								 - s->KslfFinal * s->EbetaFinal
	CalcBEMF();

    s->Theta = atan2CORDIC(-s->EalphaFinal,s->EbetaFinal);

    AccumTheta += s->Theta - PrevTheta;
	PrevTheta = s->Theta;

	AccumThetaCnt++;
	if (AccumThetaCnt == IRP_PERCALC)
	{
        //                    AccumThetaCnt * 60
        // eRPM = -----------------------------
        //               SpeedLoopTime * 65535
        //           eRPM * 2
        // RPM = ------------
        //               P
        //        For example:
        //    AccumThetaCnt = 16384
        //    SpeedLoopTime = 0.001
        //Then:
        //    Speed in eRPM is 15000 and RPM is 3000RPM
        //
        //                                   60
        // SMO_SPEED_EST_MULTIPLIER = -------------------------
        //                              SpeedLoopTime * 65535
        
        s->Omega = (int16_t)(__builtin_mulss(AccumTheta,SMO_SPEED_EST_MULTIPLIER)>>15);
		AccumThetaCnt = 0;
		AccumTheta = 0;
	}
    trans_counter++;
    if ( trans_counter == TRANSITION_STEPS) trans_counter = 0;

    s->OmegaFltred = s->OmegaFltred +(__builtin_mulss(s->FiltOmCoef,(s->Omega-s->OmegaFltred))>>15);

    s->Kslf = s->KslfFinal = (int16_t)(__builtin_mulss(s->OmegaFltred,THETA_FILTER_CNST)>>15);

	// Since filter coefficients are dynamic, we need to make sure we have a minimum
	// so we define the lowest operation speed as the lowest filter coefficient
    Kslf_min = (int16_t)(__builtin_mulss(ENDSPEED_ELECTR,THETA_FILTER_CNST)>>15);
	if (s->Kslf < Kslf_min)
	{
		s->Kslf = Kslf_min;
		s->KslfFinal = s->Kslf;
	}
	s->ThetaOffset = CONSTANT_PHASE_SHIFT;
	s->Theta = s->Theta + s->ThetaOffset;
    s->Kslide = Q15(SMCGAIN);

    POPCORCON();
    return;
}

void CalcEstI()
{
    int16_t temp_int1,temp_int2,temp_int3;
    temp_int1 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Valpha)>>15);
    temp_int2 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Ealpha)>>15);
    temp_int3 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Zalpha)>>15);

    temp_int1 = temp_int1 - temp_int2;
    temp_int1 = temp_int1 - temp_int3;
    smc1.EstIalpha = temp_int1 + (int16_t)(__builtin_mulss(smc1.Fsmopos,smc1.EstIalpha)>>15);

    temp_int1 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Vbeta)>>15);
    temp_int2 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Ebeta)>>15);
    temp_int3 = (int16_t) (__builtin_mulss(smc1.Gsmopos,smc1.Zbeta)>>15);

    temp_int1 = temp_int1 - temp_int2;
    temp_int1 = temp_int1 - temp_int3;
    smc1.EstIbeta = temp_int1 + (int16_t)(__builtin_mulss(smc1.Fsmopos,smc1.EstIbeta)>>15);

    smc1.IalphaError = smc1.EstIalpha - smc1.Ialpha;
    smc1.IbetaError  = smc1.EstIbeta - smc1.Ibeta;

    if( _Q15abs(smc1.IalphaError)<smc1.MaxSMCError)
    {
        		// s->Zalpha = (s->Kslide * s->IalphaError) / s->MaxSMCError
		// If we are in the linear range of the slide mode controller,
		// then correction factor Zalpha will be proportional to the
		// error (Ialpha - EstIalpha) and slide mode gain, Kslide.
		CalcZalpha();
    }
    else if(smc1.IalphaError>0)
    {
        smc1.Zalpha = smc1.Kslide;
    }
    else
    {
        smc1.Zalpha = -smc1.Kslide;
    }
    if( _Q15abs(smc1.IbetaError)<smc1.MaxSMCError)
    {
		CalcZbeta();
    }
    else if(smc1.IbetaError>0)
    {
        smc1.Zbeta = smc1.Kslide;
    }
    else
    {
        smc1.Zbeta = -smc1.Kslide;
    }
}

void CalcZalpha()
{
    int32_t temp_int;
    temp_int = __builtin_mulss(smc1.Kslide,smc1.IalphaError);
    smc1.Zalpha = __builtin_divsd(temp_int,smc1.MaxSMCError);
}

void CalcZbeta()
{
    int32_t temp_int;
    temp_int = __builtin_mulss(smc1.Kslide,smc1.IbetaError);
    smc1.Zbeta = __builtin_divsd(temp_int,smc1.MaxSMCError);
}

void CalcBEMF()
{
    int16_t temp_int1,temp_int2;
    temp_int1 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Zalpha)>>15);
    temp_int2 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Ealpha)>>15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.Ealpha = smc1.Ealpha + temp_int1;
    temp_int1 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Ealpha)>>15);
    temp_int2 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.EalphaFinal)>>15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.EalphaFinal = smc1.EalphaFinal + temp_int1;

    temp_int1 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Zbeta)>>15);
    temp_int2 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Ebeta)>>15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.Ebeta = smc1.Ebeta + temp_int1;
    temp_int1 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.Ebeta)>>15);
    temp_int2 = (int16_t)(__builtin_mulss(smc1.Kslf,smc1.EbetaFinal)>>15);
    temp_int1 = temp_int1 - temp_int2;
    smc1.EbetaFinal = smc1.EbetaFinal + temp_int1;
}