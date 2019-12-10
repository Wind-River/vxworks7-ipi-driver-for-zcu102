/*
* Copyright (c) <YEAR(s)>, Wind River Systems, Inc. 
*
* Redistribution and use in source and binary forms, with or without modification, are
* permitted provided that the following conditions are met:
* 
* 1) Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 
* 2) Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
* 
* 3) Neither the name of Wind River Systems nor the names of its contributors may be
* used to endorse or promote products derived from this software without specific
* prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/ 

 
/*
modification history
--------------------
29mar19,drp Created
*/

#ifndef __INCvxbFdtZynqmpIpih
#define __INCvxbFdtZynqmpIpih

#include <vxWorks.h>
#include <hwif/vxBus.h>
#include <dllLib.h>

#include <hwif/util/vxbIsrDeferLib.h>


#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE

/* defines */

#undef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))


/* base addresses of IPI Channel registers */
#define IPI_BASE_CH0_APU_DEF   0xff300000
#define IPI_BASE_CH1_RPU0      0xff310000
#define IPI_BASE_CH2_RPU1      0xff320000
#define IPI_BASE_CH3_PMU_0     0xff330000
#define IPI_BASE_CH4_PMU_1     0xff331000
#define IPI_BASE_CH5_PMU_2     0xff332000
#define IPI_BASE_CH6_PMU_3     0xff333000
#define IPI_BASE_CH7_PL_0      0xff340000
#define IPI_BASE_CH8_PL_1      0xff350000
#define IPI_BASE_CH9_PL_2      0xff360000
#define IPI_BASE_CH10_PL_3     0xff370000

/* bitmasks for IPI Channel registers */
#define IPI_MASK_CH0_APU_DEF   0x00000001
#define IPI_MASK_CH1_RPU0      0x00000100
#define IPI_MASK_CH2_RPU1      0x00000200
#define IPI_MASK_CH3_PMU_0     0x00001000
#define IPI_MASK_CH4_PMU_1     0x00002000
#define IPI_MASK_CH5_PMU_2     0x00003000
#define IPI_MASK_CH6_PMU_3     0x00008000
#define IPI_MASK_CH7_PL_0      0x01000000
#define IPI_MASK_CH8_PL_1      0x02000000
#define IPI_MASK_CH9_PL_2      0x04000000
#define IPI_MASK_CH10_PL_3     0x08000000

/* IPI register offsets */
#define IPI_TRIG_OFFSET          (0x0)    /* IPI trigger register offset */
#define IPI_OBS_OFFSET           (0x4)    /* IPI observation register offset */
#define IPI_ISR_OFFSET           (0x10)    /* IPI interrupt status register offset */
#define IPI_IMR_OFFSET           (0x14)    /* IPI interrupt mask register offset */
#define IPI_IER_OFFSET           (0x18)    /* IPI interrupt enable register offset */
#define IPI_IDR_OFFSET           (0x1C)    /* IPI interrupt disable register offset */


/* These are the values of the vectors chosen in the OpenAMP example 
and the associated masks for the IPI.  Note that the mask for the APU is the one
that Linux uses */
#define RPU_IPI_VECT		1
#define APU_IPI_VECT            0
#define RPU_IPI_MASK		IPI_MASK_CH1_RPU0
#define APU_IPI_MASK            IPI_MASK_CH7_PL_0


typedef void (*ipi_isr_handler)(int vector,void* data);

typedef struct zynqmpIpiDrvCtrl
{
    VXB_DEV_ID          pInst;
    void *              ipiChBase;      /* the start of the IPI registers for the channel */                
    void *              ipiHandle;      
    VXB_RESOURCE *      intRes;
    int 		rxIpiVector;
    UINT32		ipiMask;
    /*deferred isr*/
    ISR_DEFER_QUEUE_ID  queueId;
    struct {
        BOOL            isrGiDeferBusy;  /* General Purpose interrupt is currently deferred */
        ISR_DEFER_JOB   isrGiDefJob;     /* isrDefer Job, internal to this Mu ISR */
        ipi_isr_handler ipiFunc;         /* callback to external modules */
        int             ipiVector;       /* vector ID*/
        void*           ipiData;         /* data associated with callback above */
  	} isrData;
} ZYNQMP_IPI_DRVCTRL;


#endif    /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* __INCvxbFdtZynqmpIpih */
