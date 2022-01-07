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
#ifndef smcpos_H
#define smcpos_H

#include "userparms.h"

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
typedef struct	{
				int16_t  Valpha;   		// Input: Stationary alfa-axis stator voltage
                int16_t  Ealpha;   		// Variable: Stationary alfa-axis back EMF
				int16_t  EalphaFinal;	// Variable: Filtered EMF for Angle calculation
                int16_t  Zalpha;      	// Output: Stationary alfa-axis sliding control
                int16_t  Gsmopos;    	// Parameter: Motor dependent control gain
                int16_t  EstIalpha;   	// Variable: Estimated stationary alfa-axis stator current
                int16_t  Fsmopos;    	// Parameter: Motor dependent plant matrix
                int16_t  Vbeta;   		// Input: Stationary beta-axis stator voltage
                int16_t  Ebeta;  		// Variable: Stationary beta-axis back EMF
				int16_t  EbetaFinal;	// Variable: Filtered EMF for Angle calculation
                int16_t  Zbeta;      	// Output: Stationary beta-axis sliding control
                int16_t  EstIbeta;    	// Variable: Estimated stationary beta-axis stator current
                int16_t  Ialpha;  		// Input: Stationary alfa-axis stator current
                int16_t  IalphaError; 	// Variable: Stationary alfa-axis current error
                int16_t  Kslide;     	// Parameter: Sliding control gain
                int16_t  MaxSMCError;  	// Parameter: Maximum current error for linear SMC
                int16_t  Ibeta;  		// Input: Stationary beta-axis stator current
                int16_t  IbetaError;  	// Variable: Stationary beta-axis current error
                int16_t  Kslf;       	// Parameter: Sliding control filter gain
                int16_t  KslfFinal;    	// Parameter: BEMF Filter for angle calculation
                int16_t  FiltOmCoef;   	// Parameter: Filter Coef for Omega filtered calc
				int16_t  ThetaOffset;	// Output: Offset used to compensate rotor angle
                int16_t  Theta;			// Output: Compensated rotor angle 
				int16_t  Omega;     	// Output: Rotor speed
				int16_t  OmegaFltred;  	// Output: Filtered Rotor speed for speed PI
				} SMC;

typedef SMC *SMC_handle;

/* Estimator Parameter data type

  Description:
    This structure will host parameters related to angle/speed estimator
    parameters.
 */
typedef struct
{
    /* Integration constant */
    int16_t qDeltaT;
    /* angle of estimation */
    int16_t qRho;
    /* internal variable for angle */
    int32_t qRhoStateVar;
    /* primary speed estimation */
    int16_t qOmegaMr;
    /* last value for Ialpha */
    int16_t qLastIalpha;
    /* last value for Ibeta */
    int16_t qLastIbeta;
    /* difference Ialpha */
    int16_t qDIalpha;
    /* difference Ibeta */
    int16_t qDIbeta;
    /* BEMF alpha */
    int16_t qEsa;
    /* BEMF beta */
    int16_t qEsb;
    /* BEMF d */
    int16_t qEsd;
    /* BEMF q */
    int16_t qEsq;
    /* counter in Last DI tables */
    int16_t qDiCounter;
    /* dI*Ls/dt alpha */
    int16_t qVIndalpha;
    /* dI*Ls/dt beta */
    int16_t qVIndbeta;
    /* BEMF d filtered */
    int16_t qEsdf;
    /* state variable for BEMF d Filtered */
    int32_t qEsdStateVar;
    /* BEMF q filtered */
    int16_t qEsqf;
    /* state variable for BEMF q Filtered */
    int32_t qEsqStateVar;
    /* filter constant for d-q BEMF */
    int16_t qKfilterEsdq;
    /* Estimated speed */
    int16_t qVelEstim;
    /* Filter constant for Estimated speed */
    int16_t qVelEstimFilterK;
    /* State Variable for Estimated speed */
    int32_t qVelEstimStateVar;
    /* Value from last control step Ialpha */
    int16_t qLastValpha;
    /* Value from last control step Ibeta */
    int16_t qLastVbeta;
    /* dIalphabeta/dt */
    int16_t qDIlimitLS;
    /* dIalphabeta/dt */
    int16_t qDIlimitHS;
    /*  last  value for Ialpha */
    int16_t qLastIalphaHS[8];
    /* last  value for Ibeta */
    int16_t qLastIbetaHS[8];
    /* estimator angle initial offset */
    int16_t qRhoOffset;

} ESTIM_PARM_T;

/* Motor Estimator Parameter data type

  Description:
    This structure will host motor parameters parameters required by angle
    estimator.
 */
typedef struct
{
    /* Rs value - stator resistance */
    int16_t qRs;
    /* Ls/dt value - stator inductance / dt - variable with speed */
    int16_t qLsDt;
    /* Ls/dt value - stator inductance / dt for base speed (nominal) */
    int16_t qLsDtBase;
    /* InvKfi constant value ( InvKfi = Omega/BEMF ) */
    int16_t qInvKFi;
    /* InvKfi constant - base speed (nominal) value */
    int16_t qInvKFiBase;
} MOTOR_ESTIM_PARM_T;

extern ESTIM_PARM_T estimator;
extern MOTOR_ESTIM_PARM_T motorParm;


#define SMC_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}

// Define this in Degrees, from 0 to 360
#define THETA_AT_ALL_SPEED 90
#define THETA_ALL (uint16_t)(THETA_AT_ALL_SPEED / 180.0 * 32768.0)
#define CONSTANT_PHASE_SHIFT THETA_ALL

#define PUSHCORCON()  {__asm__ volatile ("push CORCON");}
#define POPCORCON()   {__asm__ volatile ("pop CORCON");}
#define _PI 3.1416

//void SMC_Position_Estimation(SMC_handle);
void SMCInit(SMC_handle);
void SMC_Position_Estimation_Inline(SMC_handle);
void CalcEstI(void);
void CalcIError(void);
void CalcZalpha(void);
void CalcZbeta(void);
void CalcBEMF(void);
void CalcOmegaFltred(void);
int16_t FracMpy(int16_t mul_1, int16_t mul_2);
int16_t FracDiv(int16_t num_1, int16_t den_1);
int16_t atan2CORDIC(int16_t, int16_t);

extern uint16_t  trans_counter;

extern int16_t PrevTheta;	// Previous theta which is then substracted from Theta to get
						// delta theta. This delta will be accumulated in AccumTheta, and
						// after a number of accumulations Omega is calculated.

extern int16_t AccumTheta;	// Accumulates delta theta over a number of times

extern uint16_t AccumThetaCnt;	// Counter used to calculate motor speed. Is incremented
						// in SMC_Position_Estimation() subroutine, and accumulates
						// delta Theta. After N number of accumulations, Omega is 
						// calculated. This N is diIrpPerCalc which is defined in
						// UserParms.h.

#ifdef __cplusplus
}
#endif

#endif
