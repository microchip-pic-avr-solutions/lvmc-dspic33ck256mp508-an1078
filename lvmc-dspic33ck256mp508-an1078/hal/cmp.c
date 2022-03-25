// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * cmp.c
 *
 * This file includes subroutine for initializing Comparators.
 * The DAC is configured only for its basic operation.
 * 
 * Definitions in this file are for dsPIC33CK256MP508.
 * 
 * Component: HAL - Comparator with DAC
 * 
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">
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
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Header Files ">

#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__

#include <stdint.h>
#include <stdbool.h>
#include "cmp.h"

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="FUNCTION DECLARATIONS ">
static void CMP1_Initialize(void);

// </editor-fold> 

/**
 * Function to configure Comparators
 * @param None.
 * @return None.
 * @example
 * <code>
 * status = CMP_Initialize();
 * </code>
 */
void CMP_Initialize (void)
{
    /** Initialize DACCTRL1 REGISTER */
    DACCTRL1L = 0;
    /** Common DAC Module Enable bit
        1 = Enables DAC modules
        0 = Disables DAC modules */
    DACCTRL1Lbits.DACON = 0;
    /** DAC Stop in Idle Mode bit
        1 = Discontinues module operation when device enters Idle mode
        0 = Continues module operation in Idle mode */
    DACCTRL1Lbits.DACSIDL = 0;
    /** DAC Clock Source Select bits 
        0b11 = FPLL,0b10 = AFPLL,0b01 = FVCO/PLLDIVbits.VCODIV,
        0b00 = AFVCO/APLLDIVbits.VCODIV */
    DACCTRL1Lbits.CLKSEL = 3;
    /** DAC Clock Divider bits(1)
        0b11 = Divide by 4,0b10 = Divide by 3,0b01 = Divide by 2,0b00 = 1x */
    DACCTRL1Lbits.CLKDIV = 1;
    /** Comparator Filter Clock Divider bits
        111 = Divide by 8
             ......
        000 = 1x */
    DACCTRL1Lbits.FCLKDIV = 0b111;
    
    /** Initialize DACCTRL2L REGISTER */
    DACCTRL2L = 0;
    /** Transition Mode Duration bits
        The value for TMODTIME<9:0> should be less than the SSTIME<9:0>.*/
    DACCTRL2Lbits.TMODTIME = 0;
    
    /** Initialize DACCTRL2H REGISTER */
    DACCTRL2H = 0;
    /** Time from Start of Transition Mode till Steady-State Filter is Enabled 
        The value for SSTIME<9:0> should be greater than the TMODTIME<9:0>.*/
    DACCTRL2Hbits.SSTIME = 0;

    /** Function to initialize Comparator1 Module */
    CMP1_Initialize();    
}
/**
 * Function to initialize CMP1 module of the core
 * @param None.
 * @return None.
 * @example
 * <code>
 *  CMP1_Initialize();
 * </code>
 */
void CMP1_Initialize(void)
{
    /** Initialize DAC1CONL REGISTER */
    DAC1CONL = 0;
    /** Individual DACx Module Enable bit
        1 = Enables DACx module
        0 = Disables DACx module and disables FSCM clock */
    DAC1CONLbits.DACEN = 0;
    /** Interrupt Mode select bits
        11 = Generates an interrupt on either a rising or falling edge detect
        10 = Generates an interrupt on a falling edge detect
        01 = Generates an interrupt on a rising edge detect
        00 = Interrupts are disabled */
    DAC1CONLbits.IRQM = 0;
    /** Comparator Blank Enable bit
        1 = Enables the analog comparator output to be blanked 
        0 = Disables the blanking signal to the analog comparator; */
    DAC1CONLbits.CBE = 0;
    /** DACx Output Buffer Enable bit
        1 = DACx analog voltage is connected to the DACOUT1 pin
        0 = DACx analog voltage is not connected to the DACOUT1 pin */
    DAC1CONLbits.DACOEN = 0;
    /** Comparator Digital Filter Enable bit
        1 = Digital filter is enabled
        0 = Digital filter is disabled */
    DAC1CONLbits.FLTREN = 1;
    /** Comparator Status bits -The current state of the comparator output 
        including the CMPPOL selection */
    DAC1CONLbits.CMPSTAT = 0;
    /** Comparator Output Polarity Control bit
        1 = Output is inverted
        0 = Output is non-inverted */
    DAC1CONLbits.CMPPOL = 0;
    /** Comparator Input Source Select bits - Refer Data sheet for selection 
       011-111 = Reserved
       011 = CMPxD input pin
       010 = CMPxC input pin
       001 = CMPxB input pin
       000 = CMPxA input pin
    Comparator Input source in LVMC board : IBUS_FILT_EXT - DACOUT1/AN3/CMP1C/RA3 */
    DAC1CONLbits.INSEL = 2;   
    /** Comparator Hysteresis Polarity Select bit
        1 = Hysteresis is applied to the falling edge of comparator output
        0 = Hysteresis is applied to the rising edge of comparator output */
    DAC1CONLbits.HYSPOL = 0; 
    /** Comparator Hysteresis Select bits
        0b11 = 45 mv hysteresis, 0b10 = 30 mv hysteresis
        0b01 = 15 mv hysteresis,0b00 = No hysteresis is selected */
    DAC1CONLbits.HYSSEL = 0b11; 

    /** Initialize DAC1CONH REGISTER */
    DAC1CONH = 0;
    /** DACx Leading-Edge Blanking bits
        These register bits specify the blanking period for the comparator 
        following changes to the DAC output during Change-of-State (COS) for the
        input signal selected by the HCFSEL<3:0> bits */
    DAC1CONHbits.TMCB = 0;

    /** Initialize DAC1DATL REGISTER */
    /** DACx Data bits - In Hysteretic mode, Slope Generator mode and 
        Triangle mode, this register specifies the low data value
        and/or limit for the DACx module. */
    DAC1DATL = 0;
    
    /** Initialize DAC1DATH REGISTER */
    /** DACx Data bits - This register specifies the high DACx data value. */
    DAC1DATH = 0;

    /** Initialize SLP1CONL REGISTER */
    SLP1CONL = 0;
    /** Hysteretic Comparator Function Input Select bits
        The selected input signal controls the switching between the 
        DACx high limit (DACxDATH) and the DACx low limit (DACxDATL) as the 
        data source for the PDM DAC */
    SLP1CONLbits.HCFSEL = 0;
    /** Slope Stop A Signal Select bits
        The selected Slope Stop A signal is logically OR?d with the selected 
        Slope Stop B signal to terminate the slope function.*/
    SLP1CONLbits.SLPSTOPA = 0 ;
    /** Slope Stop B Signal Select bits
        The selected Slope Stop B signal is logically OR?d with the selected 
        Slope Stop A signal to terminate the slope function.*/
    SLP1CONLbits.SLPSTOPB = 0 ;  
    /** Slope Start Signal Select bits */
    SLP1CONLbits.SLPSTRT = 0 ;    
    
    /** Initialize SLP1CONH REGISTER */
    SLP1CONH = 0;
    /** Slope Function Enable/On bit
        1 = Enables slope function
        0 = Disables slope function */
    SLP1CONHbits.SLOPEN = 0;
    /** Hysteretic Mode Enable bit
        1 = Enables Hysteretic mode for DACx
        0 = Disables Hysteretic mode for DACx 
        HME mode requires the user to disable the slope function (SLOPEN = 0).*/
    SLP1CONHbits.HME = 0 ;
    /** Triangle Wave Mode Enable bit(2)
        1 = Enables Triangle Wave mode for DACx
        0 = Disables Triangle Wave mode for DACx 
        TWME mode requires the user to enable the slope function (SLOPEN = 1).*/
    SLP1CONHbits.TWME = 0 ;  
    /** Positive Slope Mode Enable bit
        1 = Slope mode is positive (increasing)
        0 = Slope mode is negative (decreasing) */
    SLP1CONHbits.PSE = 0 ;   
    
    /** Initialize SLP1DAT REGISTER */
    /** Slope Ramp Rate Value bits */
    SLP1DAT = 0;
}
/**
 * Function to write data to the DACxDATA register
 * @param data - Specify the data to be written to the DACxDATH register.
 * @return None.
 * @example
 * <code>
 *  CMP1_ReferenceSet(1000);
 * </code>
 */
void CMP1_ReferenceSet(uint16_t data)
{
    /** Initialize DAC1DATH REGISTER */
    /** DACx Data bits - This register specifies the high DACx data value. */
    DAC1DATH = data;
}
/**
 * Function to enable/disable DAC1 Module and its output to DAC1OUT pin.
 * Ensure that other Slave DACx are disabled, before setting DAC1CONLbits.DACOEN
 * to '1' allowing DACx analog voltage to the DACOUT1 pin.
 * @param state  - Specify to enable or disable the DACx and DACxOUT
 * @return None.
 * @example
 * <code>
 *  DAC1_ModuleOutputConfigure(true);
 * </code>
 */
void CMP1_ModuleEnable(bool state)
{
    if (state == true)
    {
        /** Individual DACx Module Enable bit
            1 = Enables DACx module
            0 = Disables DACx module and disables FSCM clock */
        DAC1CONLbits.DACEN = 1;
        /** Common DAC Module Enable bit
            1 = Enables DAC modules
            0 = Disables DAC modules */
        DACCTRL1Lbits.DACON = 1;
    }
    else
    {
        /** DACx Output Buffer Enable bit
            1 = DACx analog voltage is connected to the DACOUT1 pin
            0 = DACx analog voltage is not connected to the DACOUT1 pin */
        DAC1CONLbits.DACOEN = 0;
        /** Individual DACx Module Enable bit
            1 = Enables DACx module
            0 = Disables DACx module and disables FSCM clock */
        DAC1CONLbits.DACEN = 0;
        /** Common DAC Module Enable bit
            1 = Enables DAC modules
            0 = Disables DAC modules */
        DACCTRL1Lbits.DACON = 0;
    }
}

