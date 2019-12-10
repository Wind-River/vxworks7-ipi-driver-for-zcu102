/*
 * Copyright (c) 2019, Wind River Systems, Inc. 
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
modification history
--------------------
29mar19,drp Created
*/

/*
DESCRIPTION

This component provides IPI support for the Zynqmp to which supplies Inter-Core Message delivering functionality -- MVP for openamp demo.

 For the current implementation we only care about OpenAMP support of communicating between
 two processors: A53 and R5. Only the IPI triggering and ISR is supported.  IPI Message buffers are not supported. 

 Currently, there is no association between the Openamp callout functions
 and the driver instance.  Instead, the driver is treated as a singleton device with its instance
 preserved as a global variable.  This device is referenced in the callout functions.

To add the driver to the vxWorks image, add the following component to the
kernel configuration.

\cs
vxprj component add DRV_IPI_XLNX_ZYNQMP
\ce

This driver is bound to device tree, and the device tree node ipist specify 
below parameters:

\is

\i <reg>
This parameter specifies the register base address and length of this module.

\i <compatible>
This parameter specifies the name of the IP Ipi driver, "xlnx,zynq-ipi".

\ie

An example of device node is shown below:

\cs
dts for A53:
    ipi: ipi@ff340000
            {
            compatible = "xlnx,zynqmp-ipi";
            reg = <0 0xff340000 0 0x1000>;
            interrupts = <29 0 4>;
            interrupt-parent = <&intc>;
            };


or dts for R5:
    ipi: ipi@ff310000
            {
            compatible = "xlnx,zynqmp-ipi";
            reg = <0xff310000 0x10000>;
            interrupts = <65 0 4>;
            interrupt-parent = <&intc>;
            };
\ce

INCLUDE FILES: vxBus.h string.h vxbFdtLib.h
*/

/* includes */
#include <vxWorks.h>
#include <intLib.h>
#include <vxLib.h>
#include <stdio.h>
#include <string.h>
#include <taskLib.h>
#include <subsys/int/vxbIntLib.h>
#include <hwif/vxBus.h>
#include <hwif/vxbus/vxbLib.h>
#include <hwif/buslib/vxbFdtLib.h>

#include "vxbFdtZynqmpIpi.h"

/* defines */
#define XLNX_ZYNQMP_IPI_DBG

/* debug macro */
#ifdef  XLNX_ZYNQMP_IPI_DBG

/* turning local symbols into global symbols */
#ifdef  LOCAL
#undef  LOCAL
#define LOCAL
#endif

UINT32 zynqmpIpiLevel = 0;

#include <private/kwriteLibP.h>         /* _func_kprintf */

#undef  ZYNQMP_IPI_DBG
#define ZYNQMP_IPI_DBG(lvl, fmt, ...)                                      \
        if ((zynqmpIpiLevel >= lvl) && (_func_kprintf != NULL))        \
            (* _func_kprintf)("[%s]%s(%d): " fmt, taskName(taskIdSelf()),  \
                               __FUNCTION__, __LINE__, __VA_ARGS__)
#else
#undef  ZYNQMP_IPI_DBG
#define ZYNQMP_IPI_DBG(...)
#endif  /* XLNX_ZYNQMP_IPI_DBG */

#ifdef _WRS_CONFIG_OPENAMP
extern void *vx_openamp_ipi_register;
extern void *vx_openamp_ipi_enable;
extern void *vx_openamp_ipi_notify;
extern void *vx_openamp_ipi_disable;
#endif

/* register low level access routines */

#ifdef ARMBE8
#undef SWAP32
#define SWAP32 vxbSwap32
#else
#undef SWAP32
#define SWAP32 
#endif /* ARMBE8 */

/* registers access */

#undef CSR_READ_4
#define CSR_READ_4(pCtrl, addr)                     \
        SWAP32 (vxbRead32 ((pCtrl->ipiHandle),      \
                           (UINT32 *)((UINT8 *)(pCtrl->ipiChBase) + addr)))

#undef CSR_WRITE_4
#define CSR_WRITE_4(pCtrl, addr, data)              \
        vxbWrite32 ((pCtrl->ipiHandle),             \
                   (UINT32 *)((UINT8 *)(pCtrl->ipiChBase) + addr), \
                   SWAP32(data))

#undef CSR_SETBIT_4
#define CSR_SETBIT_4(pCtrl, addr, data)             \
        CSR_WRITE_4 (pCtrl, addr, CSR_READ_4 (pCtrl, addr) | (data))

#undef CSR_CLRBIT_4
#define CSR_CLRBIT_4(pCtrl, addr, data)            \
        CSR_WRITE_4 (pCtrl, addr, CSR_READ_4 (pCtrl, addr) & ~(data))

#undef CSR_MASK_SETBIT_4
#define CSR_MASK_SETBIT_4(pCtrl, addr, mask, data)             \
        CSR_WRITE_4 (pCtrl, addr, ((CSR_READ_4 (pCtrl, addr))&(mask)) | (data))

#undef CSR_MASK_CLRBIT_4
#define CSR_MASK_CLRBIT_4(pCtrl, addr, mask, data)            \
        CSR_WRITE_4 (pCtrl, addr, ((CSR_READ_4 (pCtrl, addr))&(mask)) & ~(data))

/* typedefs */

/* forward declarations */

LOCAL STATUS fdtZynqmpIpiAttach (VXB_DEV_ID pDev);
LOCAL STATUS fdtZynqmpIpiProbe (VXB_DEV_ID pDev);

/* locals */
LOCAL VXB_DRV_METHOD fdtZynqmpIpiMethodList[] =
    {
    /* DEVICE API */
    
    { VXB_DEVMETHOD_CALL(vxbDevProbe),  fdtZynqmpIpiProbe  },
    { VXB_DEVMETHOD_CALL(vxbDevAttach), fdtZynqmpIpiAttach },
    { 0, NULL }
    };

LOCAL VXB_FDT_DEV_MATCH_ENTRY fdtZynqmpIpiMatch[] =
    {
        {
        "xlnx,zynqmp-ipi",           /* compatible */
        NULL                            /* no configuration */
        },
        {}                              /* empty terminated list */
    };

LOCAL ZYNQMP_IPI_DRVCTRL * pDrvCtrlSingleton = NULL;

LOCAL void vxbZynqmpIpiInt (VXB_DEV_ID pDev);

/* hook ipi routine for openamp */
LOCAL STATUS openampIpiNotify(int vector_id);
LOCAL STATUS openampIpiRegister(int vector_id,  void* data, ipi_isr_handler cb);
LOCAL STATUS openampIpiEnable(int vector_id);
LOCAL void openampIpiDisable(int vector_id);

/* globals */

VXB_DRV vxbFdtZynqmpIpiDrv =
    {
    { NULL } ,
    "zynqmp-ipi",                        /* Name */
    "Xilinx Zynqmp IPI FDT driver",    /* Description */
    VXB_BUSID_FDT,                      /* Class */
    0,                                  /* Flags */
    0,                                  /* Reference count */
    &fdtZynqmpIpiMethodList[0]           /* Method table */
    };

VXB_DRV_DEF(vxbFdtZynqmpIpiDrv)

/******************************************************************************
*
* fdtZynqmpIpiProbe - probe for device presence at specific address
*
* Check for IPI device at channel address.
*
* RETURNS: OK if probe passes and assumed a valid (or compatible) device. 
* ERROR otherwise.
*
*/

LOCAL STATUS fdtZynqmpIpiProbe
    (
    VXB_DEV_ID pDev             /* device information */
    )
    {
    return vxbFdtDevMatch (pDev, &fdtZynqmpIpiMatch[0], NULL);
    }



/******************************************************************************
*
* vxbZynqmpIpiIsrJob - Zynqmp IPI device ISR Job
*
* Calling usr-ISR.
*
* RETURNS: N/A
*
*/

LOCAL void vxbZynqmpIpiIsrJob(void * pData)
    {
    ZYNQMP_IPI_DRVCTRL *pDrvCtrl = pData;
    UINT32 ipiMask = pDrvCtrl->ipiMask;

    if (pDrvCtrl == NULL)
        return;

    ZYNQMP_IPI_DBG (2, "pDrvCtrl=%p\n", pDrvCtrl);
    pDrvCtrl->isrData.isrGiDeferBusy = TRUE;

    if (NULL != pDrvCtrl->isrData.ipiFunc)
        {
        ZYNQMP_IPI_DBG (2, "Calling ipiFunc=%p,ipiVector=%d,ipiData=%p\n", 
            pDrvCtrl->isrData.ipiFunc, 
            pDrvCtrl->isrData.ipiVector,
            pDrvCtrl->isrData.ipiData);
        }
        (pDrvCtrl->isrData.ipiFunc)(pDrvCtrl->isrData.ipiVector, pDrvCtrl->isrData.ipiData);
    	
    pDrvCtrl->isrData.isrGiDeferBusy = FALSE;


/* enable the interrupt */
           if (ipiMask) {
              CSR_WRITE_4(pDrvCtrl, IPI_IER_OFFSET, ipiMask);
              ZYNQMP_IPI_DBG (3, "wrote 0x%x to %p\n", ipiMask, (UINT32 *)((UINT8 *)(pDrvCtrl->ipiChBase) + IPI_IER_OFFSET));
           }
    }



/******************************************************************************
*
* fdtZynqmpIpiAttach - attach Zynqmp IPI device
*
* This is the Zynqmp IPI initialization routine.
*
* Note that currently only one driver is supported and this function will fail if an instance already exists.
*
* RETURNS: OK, or ERROR if initialization failed.
*
* ERRNO: N/A
*/

LOCAL STATUS fdtZynqmpIpiAttach
    (
    VXB_DEV_ID          pDev
    )
    {
    ZYNQMP_IPI_DRVCTRL   * pDrvCtrl = NULL;
    VXB_RESOURCE     * pResMem  = NULL;
    VXB_RESOURCE     * pResIrq  = NULL;
    void *phys;

    ZYNQMP_IPI_DBG (3, "Enter fdtZynqmpIpiAttach.pDev=%p\n", pDev);

   /* only one driver supported */
   if (pDrvCtrlSingleton) { 
	goto errOut;
   }

    pDrvCtrl = (ZYNQMP_IPI_DRVCTRL *) vxbMemAlloc (sizeof (ZYNQMP_IPI_DRVCTRL));
    if (pDrvCtrl == NULL)
         return ERROR;

    pResMem = vxbResourceAlloc (pDev, VXB_RES_MEMORY, 0);
    if ((pResMem == NULL) || (pResMem->pRes == NULL))
        {
        goto errOut;
        }

    pResIrq = vxbResourceAlloc (pDev, VXB_RES_IRQ, 0);
    if ((pResIrq == NULL) || (pResIrq->pRes == NULL))
        {
        goto errOut;
        }
    
    pDrvCtrl->ipiChBase = (void *)((VXB_RESOURCE_ADR *)(pResMem->pRes))->virtual;
    
    pDrvCtrl->ipiHandle = ((VXB_RESOURCE_ADR *) (pResMem->pRes))->pHandle;
    pDrvCtrl->pInst = pDev;
    pDrvCtrl->intRes = pResIrq;

    /* Keep track of this singelton device which is used in the OpenAMP callouts */
    pDrvCtrlSingleton = pDrvCtrl;
    
    /* save pDrvCtrl in VXB_DEVICE structure */
    vxbDevSoftcSet (pDev, pDrvCtrl);


 phys = (void*)0xff340000;
 phys = ((VXB_RESOURCE_ADR *) (pResMem->pRes))->start;

     switch ((int)phys) {
        case 0xff340000:
           pDrvCtrl->ipiMask = RPU_IPI_MASK;
#if 1
           pDrvCtrl->isrData.ipiVector = APU_IPI_VECT;
#endif
           ZYNQMP_IPI_DBG (3, "Running on APU my IPI mask is 0x%x and the RPU mask is %p.\n", APU_IPI_MASK, RPU_IPI_MASK);
        break;
        case 0xff310000:
           pDrvCtrl->ipiMask = APU_IPI_MASK + RPU_IPI_MASK;
#if 1
           pDrvCtrl->isrData.ipiVector = RPU_IPI_VECT;
#endif
           ZYNQMP_IPI_DBG (3, "Running on RPU my IPI mask is 0x%x and the APU mask is %p.\n", RPU_IPI_MASK, APU_IPI_MASK);
        break;
        default:
           goto errOut;
      }


    /* Set Up deferred ISR */
    if (pDrvCtrl->queueId == NULL)
        {
        pDrvCtrl->queueId = isrDeferQueueGet (pDrvCtrl->pInst, 1, 0, 0);
        }
                
        ZYNQMP_IPI_DBG(3,"queueId = 0x%08x, tid=0x%08x\n", pDrvCtrl->queueId, 
                  ((ISR_DEFER_QUEUE*)(pDrvCtrl->queueId))->tid);
        pDrvCtrl->isrData.isrGiDeferBusy = FALSE;
        pDrvCtrl->isrData.ipiFunc = 0;
        pDrvCtrl->isrData.ipiData = 0;
        pDrvCtrl->isrData.isrGiDefJob.func = vxbZynqmpIpiIsrJob;
        pDrvCtrl->isrData.isrGiDefJob.pData = pDrvCtrl; 

    /*	connect the ISR  */
    (void) vxbIntConnect(pDev, pDrvCtrl->intRes, vxbZynqmpIpiInt, pDev);

    /* enable the interrupt */
    (void) vxbIntEnable(pDev, pDrvCtrl->intRes);

#ifdef _WRS_CONFIG_OPENAMP
    vx_openamp_ipi_register = openampIpiRegister;
    vx_openamp_ipi_enable = openampIpiEnable;
    vx_openamp_ipi_disable = openampIpiDisable;
    vx_openamp_ipi_notify = openampIpiNotify;
#endif

    ZYNQMP_IPI_DBG (3, "Attach OK, ipiChBase=%p\n", pDrvCtrl->ipiChBase);
    return OK;
    
errOut:
    if (pResMem != NULL)
        (void)vxbResourceFree (pDev, pResMem);
    
    if (pResIrq != NULL)
        (void)vxbResourceFree (pDev, pResIrq);
    
    if (pDrvCtrl != NULL)
        vxbMemFree (pDrvCtrl);
    
    return ERROR;
}
    
    
/******************************************************************************
*
* vxbZynqmpIpiInt - Zynqmp IPI device ISR
*
* This is the Zynqmp IPI ISR routine.
* 
* For the current implementation we only care about OpenAMP support of communicationg between
* two processors: A53 and R5. Currently, there is no assoication between the Openamp callout funtions
* and the driver instance.  Instead, the driver is treated as a singleton device with its instance
* preserved as a global variable.  This device is referenced in the callout functions.
*
* In the current implementation of the driver supports one instance only and the assumption that there is
* on remote CPU that is exchanging messages.  The source of the IPI is ingnored and assumed to be the remote.
*
* RETURNS: N/A
*/

LOCAL void vxbZynqmpIpiInt (VXB_DEV_ID pDev)
    {
    volatile FAST UINT32 isr;
    volatile FAST UINT32 imr;
    ZYNQMP_IPI_DRVCTRL   *pDrvCtrl;
    UINT32 ipiMask;


    pDrvCtrl = (ZYNQMP_IPI_DRVCTRL *)(vxbDevSoftcGet(pDev));

/*  get the sender using the source and mask */
    isr = CSR_READ_4(pDrvCtrl, IPI_ISR_OFFSET);  
    ZYNQMP_IPI_DBG (3, "read 0x%0x from %p\n", isr, (UINT32 *)((UINT8 *)(pDrvCtrl->ipiChBase) + IPI_ISR_OFFSET));

        if (NULL != pDrvCtrl->isrData.ipiFunc)
            {
/* clear the interupt mask for aknowledgement (we assume it came from the remote) */
            if (isr) {
              CSR_WRITE_4(pDrvCtrl, IPI_ISR_OFFSET, isr);
              ZYNQMP_IPI_DBG (3, "wrote 0x%0x to %p\n", isr, (UINT32 *)((UINT8 *)(pDrvCtrl->ipiChBase) + IPI_ISR_OFFSET));
            }

/* disable the interrupt */
	   ipiMask = pDrvCtrl->ipiMask;
           if (ipiMask) {
              CSR_WRITE_4(pDrvCtrl, IPI_IDR_OFFSET, ipiMask);
              ZYNQMP_IPI_DBG (3, "wrote 0x%0x to %p\n", ipiMask, (UINT32 *)((UINT8 *)(pDrvCtrl->ipiChBase) + IPI_IDR_OFFSET));
           }
        	
            if (!pDrvCtrl->isrData.isrGiDeferBusy)
                {
                /* push the job into job queue */
                isrDeferJobAdd (pDrvCtrl->queueId, 
                                &pDrvCtrl->isrData.isrGiDefJob);
                }
        }
	
}

/*****************************************************************************
*
* openampIpiNotify - Zynqmp IPI device IPI notify
*
* This is the Zynqmp IPI notification routine.
*
* The implementation of this routine is closely couple to the OpenAMP example
* and uses hardcoded valued for the the vector number to IPI relationships.
*
* RETURNS: OK, or ERROR if initialization failed.
*
* ERRNO: N/A
*/

LOCAL STATUS openampIpiNotify(int vector_id)
    {
    ZYNQMP_IPI_DRVCTRL *pDrvCtrl = pDrvCtrlSingleton;
    UINT32 ipiMask = pDrvCtrl->ipiMask;

    ZYNQMP_IPI_DBG (1, "vector_id=%d\n", vector_id);


   /* This is kind of hack but we assume that there are two vectors: APU and RPU.
      The numbers were chosen abitrarilty but the real IPI vectors are H/W specific
      Here we simply check the vector and send the IPI base on a definition */
    
#if 0
    if (pDrvCtrl->isrData.ipiVector != vector_id) {
#endif
    if (ipiMask) {
        CSR_WRITE_4(pDrvCtrl, IPI_TRIG_OFFSET, ipiMask);
              ZYNQMP_IPI_DBG (3, "wrote 0x%0x to %p\n", ipiMask, (UINT32 *)((UINT8 *)(pDrvCtrl->ipiChBase) + IPI_TRIG_OFFSET));
    }
    else {
       return ERROR;
    }
#if 0
}
#endif

    return OK;
    }

/*****************************************************************************
*
* openampIpiEnable - enable Zynqmp IPI device IPI
*
* This is the Zynqmp IPI enable routine.
*
* RETURNS: OK, or ERROR if interrupt vector invalid.
*
* ERRNO: N/A
*/

LOCAL STATUS openampIpiEnable(int vector_id)
    {
    ZYNQMP_IPI_DRVCTRL *pDrvCtrl = pDrvCtrlSingleton;
    UINT32 ipiMask = pDrvCtrl->ipiMask;;
    
    ZYNQMP_IPI_DBG (2, "vector_id=%d\n", vector_id);
#if 1
    if (pDrvCtrl->isrData.ipiVector == vector_id) {
#endif
    if (ipiMask) {
        CSR_WRITE_4(pDrvCtrl, IPI_IER_OFFSET, ipiMask);
              ZYNQMP_IPI_DBG (3, "wrote 0x%0x to %p\n", ipiMask, (UINT32 *)((UINT8 *)(pDrvCtrl->ipiChBase) + IPI_IER_OFFSET));
    }
    else {
       return ERROR;
    }
#if 1
}
#endif
    return OK;
    }


/******************************************************************************
*
* openampIpiDisable - disable Zynqmp IPI device IPI
*
* This is the Zynqmp IPI disable routine.
*
* RETURNS: N/A
*/

LOCAL void openampIpiDisable(int vector_id)
    {
    ZYNQMP_IPI_DRVCTRL *pDrvCtrl = pDrvCtrlSingleton;
    UINT32 ipiMask = pDrvCtrl->ipiMask;
    
    ZYNQMP_IPI_DBG (2, "vector_id=%d\n", vector_id);

#if 1
    if (pDrvCtrl->isrData.ipiVector == vector_id) {
#endif
    if (ipiMask) {
        CSR_WRITE_4(pDrvCtrl, IPI_IDR_OFFSET, ipiMask);
        ZYNQMP_IPI_DBG (3, "wrote 0x%0x to %p\n", ipiMask, (UINT32 *)((UINT8 *)(pDrvCtrl->ipiChBase) + IPI_IDR_OFFSET));
    }
#if 1
}
#endif
    }

/******************************************************************************
*
* openampIpiRegister - register Zynqmp IPI device IPI
*
* This is the Zynqmp IPI register routine.
*
* RETURNS: OK
*/

LOCAL STATUS openampIpiRegister(int vector_id,  void* data, ipi_isr_handler cb)
    {			
    ZYNQMP_IPI_DRVCTRL *pDrvCtrl = pDrvCtrlSingleton;

    ZYNQMP_IPI_DBG (2, "vector=%d,data=%p,cb=%p\n", vector_id, data, cb);
    if (pDrvCtrl->isrData.ipiVector == vector_id) {
      pDrvCtrl->isrData.ipiFunc = cb;
      pDrvCtrl->isrData.ipiData = data;
    }
    		
    return OK;
    }

