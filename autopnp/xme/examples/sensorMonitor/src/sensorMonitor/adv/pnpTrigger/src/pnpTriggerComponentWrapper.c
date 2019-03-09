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
 * $Id: pnpTriggerComponentWrapper.c 7769 2014-03-11 15:27:13Z geisinger $
 */

/**
 * \file
 *         Component wrapper - implements interface of a component
 *              to the data handler
 *
 * \author
 *         This file has been generated by the CHROMOSOME Modeling Tool (XMT)
 *         (fortiss GmbH).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "sensorMonitor/adv/pnpTrigger/include/pnpTriggerComponentWrapper.h"

#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/log.h"

#ifdef XME_MULTITHREAD
#include "xme/hal/include/tls.h"
#endif // #ifdef XME_MULTITHREAD

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief Number of times this component wrapper has been initialized.
 */
static uint16_t initializationCount = 0U;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
sensorMonitor_adv_pnpTrigger_pnpTriggerComponentWrapper_init(void)
{

    initializationCount++;

    return XME_STATUS_SUCCESS;
}

void
sensorMonitor_adv_pnpTrigger_pnpTriggerComponentWrapper_fini(void)
{
    XME_ASSERT_NORVAL(initializationCount > 0U);
    initializationCount--;

}

