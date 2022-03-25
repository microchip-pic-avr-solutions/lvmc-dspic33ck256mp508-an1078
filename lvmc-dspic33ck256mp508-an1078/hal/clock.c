/*******************************************************************************
  Clock Configuration Routine source File

  File Name:
    clock.c

  Summary:
    This file includes subroutine for initializing Oscillator and Reference 
    Clock Output

  Description:
    Definitions in the file are for dsPIC33CK256MP508

*******************************************************************************/
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
#include "clock.h"

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
void InitOscillator(void);
void EnableREFCLKOutput(uint16_t);
// *****************************************************************************
/* Function:
    void InitOscillator(void)

  Summary:
    Routine to configure controller oscillator

  Description:
    Function configure oscillator PLL to generate desired processor clock

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
 
void InitOscillator(void)

{
    /*DOZEN: Doze Mode Enable bit
    0 = Processor clock and peripheral clock ratio is forced to 1:1           */
    CLKDIVbits.DOZEN = 0;
    
    /* FRCDIV<2:0>: Internal Fast RC Oscillator Post-scaler bits
       000 = FRC divided by 1 (default)                                       */
    CLKDIVbits.FRCDIV = 0;
    
        /* VCODIV<1:0>: PLL VCO Output Divider Select bits                        
     * 0b11=VCO clock; 0b10=VCO/2 clock; 0b01=VCO/3 clock ; 0b00=VCO/4 clock  */
    PLLDIVbits.VCODIV = 2;
        
     /* In this device Internal RC Oscillator is 8MHz
     * Also,In all Motor Control Development boards primary oscillator or 
     * Crystal oscillator output frequency is  8MHz
     * Hence, FPLLI (PLL Input frequency)is 8MHz in the application
     * 
     * FOSC (Oscillator output frequency),FCY (Device Operating Frequency),
     * FVCO (VCO Output Frequency )is:
     *         ( FPLLI * M)     (8 * 150)           
     * FVCO = -------------- = -----------  = 1200 MHz
     *               N1             1    
     *
     *         (FPLLI * M)     1    (8 * 150)      1     
     * FOSC = -------------- * - = -----------  * ---  = 200 MHz
     *        (N1 * N2 * N3)   2   (1 * 3 * 1)     2
     *
     * FCY  = 200 MHz / 2 =  100 MHz
     *
     * where,
     * N1 = CLKDIVbits.PLLPRE = 1 
     * N2 = PLLDIVbits.POST1DIV = 3
     * N3 = PLLDIVbits.POST2DIV = 1 
     * M = PLLFBDbits.PLLFBDIV = 150
     */
    
    /* PLL Feedback Divider bits (also denoted as ?M?, PLL multiplier)
     * M = (PLLFBDbits.PLLFBDIV)= 150                                         */
    PLLFBDbits.PLLFBDIV = 150;

    /* PLL Phase Detector I/P Divider Select bits(denoted as ?N1?,PLL pre-scaler)
     * N1 = CLKDIVbits.PLLPRE = 1                                             */
    CLKDIVbits.PLLPRE = 1;

    /* PLL Output Divider #1 Ratio bits((denoted as 'N2' or POSTDIV#1)
     * N2 = PLLDIVbits.POST1DIV = 3                                           */
    PLLDIVbits.POST1DIV = 3;
    
    /* PLL Output Divider #2 Ratio bits((denoted as 'N3' or POSTDIV#2)
     * N3 = PLLDIVbits.POST2DIV = 1                                           */
    PLLDIVbits.POST2DIV = 1;
    
    /* Initiate Clock Switch to FRC Oscillator with PLL (NOSC=0b001)
     *  NOSC = 0b001 = Fast RC Oscillator with PLL (FRCPLL)                   */
    __builtin_write_OSCCONH(0x01);

    /* Request oscillator switch to selection specified by the NOSC<2:0>bits  */
    __builtin_write_OSCCONL(OSCCON | 0x01);

    /* Wait for Clock switch to occur */
    while (OSCCONbits.OSWEN!= 0);

    /* Wait for PLL to lock */
    while (OSCCONbits.LOCK != 1);

}
// *****************************************************************************
/* Function:
    void EnableREFCLKOutput(uint16_t Divider)

  Summary:
    Routine to configure Reference Clock Output

  Description:
    Function configure Reference Clock output on to a device pin

  Precondition:
    None.

  Parameters:
    Divider - Specify Reference Clock Divider Ratio

  Returns:
    None.

  Remarks:
    Function assumes remap-able pin is configured as reference clock output
 */
void EnableREFCLKOutput(uint16_t Divider)
{
    
    if(REFOCONLbits.ROACTIVE == 0)
    {
        REFOCONHbits.RODIV = Divider;
        REFOCONLbits.ROSLP = 1;
        REFOCONLbits.ROSIDL = 1;
        REFOCONLbits.ROSEL = 1;   
        REFOCONLbits.ROOUT = 1;
        REFOCONLbits.ROEN = 1;
    }
}
