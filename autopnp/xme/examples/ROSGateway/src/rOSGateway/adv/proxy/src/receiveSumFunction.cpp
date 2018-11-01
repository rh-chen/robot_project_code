/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: receiveSumFunction.cpp 7805 2014-03-13 09:54:35Z geisinger $
 */

/**
 * \file
 *         Source file for function receiveSum in component proxy.
 *
 * \author
 *         This file has been generated by the CHROMOSOME Modeling Tool (XMT)
 *         (fortiss GmbH).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "rOSGateway/adv/proxy/include/receiveSumFunction.h"

#include "rOSGateway/adv/proxy/include/receiveSumFunctionWrapper.h"
#include "rOSGateway/adv/proxy/include/proxyComponent.h"
#include "rOSGateway/adv/proxy/include/proxyComponentWrapper.h"
#include "rOSGateway/adv/proxy/include/proxyManifest.h"

#include "xme/core/logUtils.h"

#include "xme/hal/include/mem.h"

// PROTECTED REGION ID(ROSGATEWAY_ADV_PROXY_RECEIVESUMFUNCTION_C_INCLUDES) ENABLED START
// PROTECTED REGION END

/******************************************************************************/
/***   Definitions                                                          ***/
/******************************************************************************/

// PROTECTED REGION ID(ROSGATEWAY_ADV_PROXY_RECEIVESUMFUNCTION_C_DEFINITIONS) ENABLED START
// PROTECTED REGION END

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

// PROTECTED REGION ID(ROSGATEWAY_ADV_PROXY_RECEIVESUMFUNCTION_C_VARIABLES) ENABLED START
// PROTECTED REGION END

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

// PROTECTED REGION ID(ROSGATEWAY_ADV_PROXY_RECEIVESUMFUNCTION_C_PROTOTYPES) ENABLED START
// PROTECTED REGION END

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
rOSGateway_adv_proxy_receiveSumFunction_init
(
    rOSGateway_adv_proxy_proxyComponent_config_t* const componentConfig
)
{
    // PROTECTED REGION ID(ROSGATEWAY_ADV_PROXY_RECEIVESUMFUNCTION_INITIALIZE_C) ENABLED START
    XME_UNUSED_PARAMETER(componentConfig);
    
    return XME_STATUS_SUCCESS;
    // PROTECTED REGION END
}

void
rOSGateway_adv_proxy_receiveSumFunction_step
(
    rOSGateway_adv_proxy_proxyComponent_config_t* const componentConfig
)
{
    xme_status_t status[1];
    
    ROSGateway_topic_sumResponse_t portReceivedSumData; // Required port.
    
    (void)xme_hal_mem_set(&portReceivedSumData, 0u, sizeof(ROSGateway_topic_sumResponse_t));
    
    status[0] = rOSGateway_adv_proxy_proxyComponentWrapper_readPortReceivedSum(&portReceivedSumData);
    
    {
        // PROTECTED REGION ID(ROSGATEWAY_ADV_PROXY_RECEIVESUMFUNCTION_STEP_C) ENABLED START
        XME_UNUSED_PARAMETER(status);

        xme_fallback_printf("Proxy: receiveSum\n");

        componentConfig->response = (ROSGateway_topic_sumResponse_t*)xme_hal_mem_alloc(sizeof(ROSGateway_topic_sumResponse_t));
        componentConfig->response->sum = portReceivedSumData.sum;
        // PROTECTED REGION END
    }
    
    {
        // PROTECTED REGION ID(ROSGATEWAY_ADV_PROXY_RECEIVESUMFUNCTION_STEP_2_C) ENABLED START
        // PROTECTED REGION END
    }
}

void
rOSGateway_adv_proxy_receiveSumFunction_fini
(
    rOSGateway_adv_proxy_proxyComponent_config_t* const componentConfig
)
{
    // PROTECTED REGION ID(ROSGATEWAY_ADV_PROXY_RECEIVESUMFUNCTION_TERMINATE_C) ENABLED START
    XME_UNUSED_PARAMETER(componentConfig);
    // PROTECTED REGION END
}

// PROTECTED REGION ID(ROSGATEWAY_ADV_PROXY_RECEIVESUMFUNCTION_IMPLEMENTATION_C) ENABLED START
// PROTECTED REGION END
