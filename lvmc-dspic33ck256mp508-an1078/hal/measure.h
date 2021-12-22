// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file measure.h
 *
 * @brief This module has functions for signal conditioning of measured
 *        analog feedback signals.
 *
 * Component: MEASURE
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

#ifndef __MEASURE_H
#define __MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include "general.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DEFINITIONS ">

#define OFFSET_COUNT_BITS   (int16_t)10
#define OFFSET_COUNT_MAX    (int16_t)(1 << OFFSET_COUNT_BITS)
    
#define OFFSET_COUNT_MOSFET_TEMP 4964
#define MOSFET_TEMP_COEFF Q15(0.010071108)    //3.3V/(32767*0.01V)
#define MOSFET_TEMP_AVG_FILTER_SCALE     8
    
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">

typedef struct
{
    int16_t
        offsetIa,       /* A phase current offset */
        offsetIb,       /* B phase current offset */
        offsetIbus,     /* BUS current offset */
        Ia,             /* A phase Current Feedback */
        Ib,             /* B phase Current Feedback */
        Ibus,           /* BUS current Feedback */
        counter,        /* counter */
        status;         /* flag to indicate offset measurement completion */

    int32_t
        sumIa,          /* Accumulation of Ia */
        sumIb,          /* Accumulation of Ib */
        sumIbus;        /* Accumulation of Ibus */

} MCAPP_MEASURE_CURRENT_T;

typedef struct
{
    int16_t input;
    uint16_t index;
    uint16_t maxIndex;
    uint16_t scaler;
    int16_t avg;
    int32_t sum;
    uint16_t status;
    
}MCAPP_MEASURE_AVG_T;

typedef struct
{
    int16_t 
        potValue;         /* Measure potentiometer */
    int16_t
        dcBusVoltage;
    int16_t
        MOSFETTemperatureAvg;  
    MCAPP_MEASURE_AVG_T MOSFETTemperature;
    MCAPP_MEASURE_CURRENT_T
        current;     /* Current measurement parameters */
            
}MCAPP_MEASURE_T;

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_MeasureCurrentOffset (MCAPP_MEASURE_T *);
void MCAPP_MeasureCurrentCalibrate (MCAPP_MEASURE_T *);
void MCAPP_MeasureCurrentInit (MCAPP_MEASURE_T *);
int16_t MCAPP_MeasureCurrentOffsetStatus (MCAPP_MEASURE_T *);
void MCAPP_MeasureTemperature(MCAPP_MEASURE_T *,int16_t );
void MCAPP_MeasureAvgInit(MCAPP_MEASURE_AVG_T *,uint16_t );
int16_t MCAPP_MeasureAvg(MCAPP_MEASURE_AVG_T *);

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif /* end of __MEASURE_H */
