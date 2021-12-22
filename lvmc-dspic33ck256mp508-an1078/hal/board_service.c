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
#include <stdint.h>
#include <stdbool.h>
#include "board_service.h"
#include "general.h"
#include "userparms.h"
#include "cmp.h"
#include "pwm.h"

BUTTON_T buttonStartStop;
BUTTON_T buttonSpeedHalfDouble;
uint16_t boardServiceISRCounter = 0;

void BoardServiceInit(void);
void BoardServiceStepIsr(void);
void BoardService(void);
void EnablePWMOutputs(void);
void DisablePWMOutputs(void);
void ClearPWMPCIFault(void);

bool IsPressed_Button1(void);
bool IsPressed_Button2(void);

void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *,MC_DUTYCYCLEOUT_T *);
void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *);
void pwmDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *,uint16_t,uint16_t);

static void ButtonGroupInitialize(void);
static void ButtonScan(BUTTON_T * ,bool);

bool IsPressed_Button1(void)
{
    if (buttonStartStop.status)
    {
        buttonStartStop.status = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool IsPressed_Button2(void)
{
    if (buttonSpeedHalfDouble.status)
    {
        buttonSpeedHalfDouble.status = false;
        return true;
    }
    else
    {
        return false;
    }
}

void BoardServiceStepIsr(void)
{
    if (boardServiceISRCounter <  BOARD_SERVICE_TICK_COUNT)
    {
        boardServiceISRCounter += 1;
    }
}
void BoardService(void)
{
    if (boardServiceISRCounter ==  BOARD_SERVICE_TICK_COUNT)
    {
        /* Button scanning loop for Button 1 to start Motor A */
        ButtonScan(&buttonStartStop,BUTTON_START_STOP);

        /* Button scanning loop for SW2 to enter into filed
            weakening mode */
        ButtonScan(&buttonSpeedHalfDouble,BUTTON_SPEED_HALF_DOUBLE);

        boardServiceISRCounter = 0;
    }
}
void BoardServiceInit(void)
{
    ButtonGroupInitialize();
    boardServiceISRCounter = BOARD_SERVICE_TICK_COUNT;
}

void ButtonScan(BUTTON_T *pButton,bool button) 
{
    if (button == true) 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->debounceCount--;
            pButton->state = BUTTON_DEBOUNCE;
        }
    } 
    else 
    {
        if (pButton->debounceCount < BUTTON_DEBOUNCE_COUNT) 
        {
            pButton->state = BUTTON_NOT_PRESSED;
        } 
        else 
        {
            pButton->state = BUTTON_PRESSED;
            pButton->status = true;
        }
        pButton->debounceCount = 0;
    }
}
void ButtonGroupInitialize(void)
{
    buttonStartStop.state = BUTTON_NOT_PRESSED;
    buttonStartStop.debounceCount = 0;
    buttonStartStop.state = false; 
    buttonSpeedHalfDouble.state = BUTTON_NOT_PRESSED;
    buttonSpeedHalfDouble.debounceCount = 0;
    buttonSpeedHalfDouble.state = false;	  
}
/* Function:
    InitPeripherals()

  Summary:
    Routine initializes controller peripherals

  Description:
    Routine to initialize Peripherals used for Inverter Control

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void Init_Peripherals(void)
{                
	/* Configures the clock settings (MIPS) */
    InitOscillator();
    
    /* Configures and Maps GPIO Function */
    SetupGPIOPorts();  
    
    /*Configures the Comparator#1 for PCI fault*/
    uint16_t cmpReference = 0;
    CMP_Initialize();
    CMP1_ModuleEnable(true);
    cmpReference = (uint16_t)(__builtin_mulss(Q15_OVER_CURRENT_THRESHOLD,2047)>>15);
    cmpReference = cmpReference + 2048; 
    CMP1_ReferenceSet(cmpReference);
    
    /* Initialize PWM generators */
    InitPWMGenerators();
    
    /* Initialize the ADC used for sensing Inverter A parameters */
    InitializeADCs();
}

void EnablePWMOutputs(void)
{
    //PWM controls signals in complementary mode, over-ride disabled
	// 0 = PWM Generator provides data for the PWM4H pin
    PG4IOCONLbits.OVRENH = 0; 
    // 0 = PWM Generator provides data for the PWM4L pin
    PG4IOCONLbits.OVRENL = 0; 
    
    // 0 = PWM Generator provides data for the PWM2H pin
    PG2IOCONLbits.OVRENH = 0;
    // 0 = PWM Generator provides data for the PWM2L pin
    PG2IOCONLbits.OVRENL = 0; 
    
    // 0 = PWM Generator provides data for the PWM1H pin
    PG1IOCONLbits.OVRENH = 0;  
    // 0 = PWM Generator provides data for the PWM1L pin
    PG1IOCONLbits.OVRENL = 0;   
}

void DisablePWMOutputs(void)
{
    /** Set Override Data on all PWM outputs */
    // 0b00 = State for PWM4H,L, if Override is Enabled
    PG4IOCONLbits.OVRDAT = 0;
    // 0b00 = State for PWM2H,L, if Override is Enabled
    PG2IOCONLbits.OVRDAT = 0; 
    // 0b00 = State for PWM1H,L, if Override is Enabled
    PG1IOCONLbits.OVRDAT = 0;  
    
    // 1 = OVRDAT<1> provides data for output on PWM4H
    PG4IOCONLbits.OVRENH = 1; 
    // 1 = OVRDAT<0> provides data for output on PWM4L
    PG4IOCONLbits.OVRENL = 1; 
    
    // 1 = OVRDAT<1> provides data for output on PWM2H
    PG2IOCONLbits.OVRENH = 1;
    // 1 = OVRDAT<0> provides data for output on PWM2L
    PG2IOCONLbits.OVRENL = 1; 
    
    // 1 = OVRDAT<1> provides data for output on PWM1H
    PG1IOCONLbits.OVRENH = 1;  
    // 1 = OVRDAT<0> provides data for output on PWM1L
    PG1IOCONLbits.OVRENL = 1;     
}

void ClearPWMPCIFault(void)
{
    /* SWTERM: PCI Software Termination bit
       A write of ?1? to this location will produce a termination event*/
    PG1FPCILbits.SWTERM = 1;
    PG2FPCILbits.SWTERM = 1;
    PG4FPCILbits.SWTERM = 1;
}

/*PWM Mode 4 - Center-Aligned Mode for Dual Shunt operation
  Write to PGxDC registers*/
void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *pPwmDutycycle)
{
    pwmDutyCycleLimitCheck(pPwmDutycycle,(DDEADTIME>>1),(LOOPTIME_TCY - (DDEADTIME>>1)));  
    PWM_PDC3 = pPwmDutycycle->dutycycle3;
    PWM_PDC2 = pPwmDutycycle->dutycycle2;
    PWM_PDC1 = pPwmDutycycle->dutycycle1;
}

/*PWM Mode 6 - Dual Edge Center-Aligned PWM Mode - Single update per cycle for Single Shunt
  Write to PGxDC and PGxPHASE */
void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *pPwmDutycycle1,MC_DUTYCYCLEOUT_T *pPwmDutycycle2)
{
    pwmDutyCycleLimitCheck(pPwmDutycycle1,(DDEADTIME>>1),(LOOPTIME_TCY - (DDEADTIME>>1)));
    
    PWM_PHASE3 = pPwmDutycycle1->dutycycle3 + (DDEADTIME>>1);
    PWM_PHASE2 = pPwmDutycycle1->dutycycle2 + (DDEADTIME>>1);
    PWM_PHASE1 = pPwmDutycycle1->dutycycle1 + (DDEADTIME>>1);
    
    pwmDutyCycleLimitCheck(pPwmDutycycle2,(DDEADTIME>>1),(LOOPTIME_TCY - (DDEADTIME>>1)));
    
    PWM_PDC3 = pPwmDutycycle2->dutycycle3 - (DDEADTIME>>1);
    PWM_PDC2 = pPwmDutycycle2->dutycycle2 - (DDEADTIME>>1);
    PWM_PDC1 = pPwmDutycycle2->dutycycle1 - (DDEADTIME>>1);
}

/*Limiting the PWM duty ratio within specified limits */
void pwmDutyCycleLimitCheck (MC_DUTYCYCLEOUT_T *pPwmDutycycle,uint16_t min,uint16_t max)
{
    if (pPwmDutycycle->dutycycle1 < min)
    {
        pPwmDutycycle->dutycycle1 = min;
    }
    else if (pPwmDutycycle->dutycycle1 > max)
    {
        pPwmDutycycle->dutycycle1 = max;
    }
    
    if (pPwmDutycycle->dutycycle2 < min)
    {
        pPwmDutycycle->dutycycle2 = min;
    }
    else if (pPwmDutycycle->dutycycle2 > max)
    {
        pPwmDutycycle->dutycycle2 = max;
    }
    
    if (pPwmDutycycle->dutycycle3 < min)
    {
        pPwmDutycycle->dutycycle3 = min;
    }
    else if (pPwmDutycycle->dutycycle3 > max)
    {
        pPwmDutycycle->dutycycle3 = max;
    }
}