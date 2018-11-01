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
 * $Id: sendFunction.c 7805 2014-03-13 09:54:35Z geisinger $
 */

/**
 * \file
 *         Source file for function send in component publisherLowQuality.
 *
 * \author
 *         This file has been generated by the CHROMOSOME Modeling Tool (XMT)
 *         (fortiss GmbH).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "configuratorExtension/adv/publisherLowQuality/include/sendFunction.h"

#include "configuratorExtension/adv/publisherLowQuality/include/sendFunctionWrapper.h"
#include "configuratorExtension/adv/publisherLowQuality/include/publisherLowQualityComponent.h"
#include "configuratorExtension/adv/publisherLowQuality/include/publisherLowQualityComponentWrapper.h"
#include "configuratorExtension/adv/publisherLowQuality/include/publisherLowQualityManifest.h"

#include "xme/core/logUtils.h"

// PROTECTED REGION ID(CONFIGURATOREXTENSION_ADV_PUBLISHERLOWQUALITY_SENDFUNCTION_C_INCLUDES) ENABLED START

#include "configuratorExtension/topic/dictionary.h"

// PROTECTED REGION END

/******************************************************************************/
/***   Definitions                                                          ***/
/******************************************************************************/

// PROTECTED REGION ID(CONFIGURATOREXTENSION_ADV_PUBLISHERLOWQUALITY_SENDFUNCTION_C_DEFINITIONS) ENABLED START

// PROTECTED REGION END

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief  Variable holding the value of the required output port 'out'.
 *
 * \details If necessary initialize this in the init function.
 *          The value of this variable will be written to the port at the end of
 *          the step function.
 */
static configuratorExtension_topic_data_t
portOutData;

// PROTECTED REGION ID(CONFIGURATOREXTENSION_ADV_PUBLISHERLOWQUALITY_SENDFUNCTION_C_VARIABLES) ENABLED START

/**
 * \brief Value of the 'quality' attribute of this publisher's publication.
 */
static const configuratorExtension_attribute_quality_t quality = 12;

// PROTECTED REGION END

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

// PROTECTED REGION ID(CONFIGURATOREXTENSION_ADV_PUBLISHERLOWQUALITY_SENDFUNCTION_C_PROTOTYPES) ENABLED START

// PROTECTED REGION END

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
configuratorExtension_adv_publisherLowQuality_sendFunction_init
(
    configuratorExtension_adv_publisherLowQuality_publisherLowQualityComponent_config_t* const componentConfig
)
{
    // PROTECTED REGION ID(CONFIGURATOREXTENSION_ADV_PUBLISHERLOWQUALITY_SENDFUNCTION_INITIALIZE_C) ENABLED START
    
    // Nothing to do
    
    XME_UNUSED_PARAMETER(componentConfig);
    
    return XME_STATUS_SUCCESS;
    
    // PROTECTED REGION END
}

void
configuratorExtension_adv_publisherLowQuality_sendFunction_step
(
    configuratorExtension_adv_publisherLowQuality_publisherLowQualityComponent_config_t* const componentConfig
)
{
    xme_status_t status[1];
    
    configuratorExtension_topic_data_t* portOutDataPtr = &portOutData;
    
    {
        // PROTECTED REGION ID(CONFIGURATOREXTENSION_ADV_PUBLISHERLOWQUALITY_SENDFUNCTION_STEP_C) ENABLED START
        
        static uint32_t value = 0;

        XME_UNUSED_PARAMETER(componentConfig);
        XME_UNUSED_PARAMETER(status);

        portOutData.value = value;

        value++;

        (void) configuratorExtension_adv_publisherLowQuality_publisherLowQualityComponentWrapper_writeOutputPortAttribute
        (
            (configuratorExtension_adv_publisherLowQuality_publisherLowQualityComponentWrapper_internalPortId_t)
                CONFIGURATOREXTENSION_ADV_PUBLISHERLOWQUALITY_PUBLISHERLOWQUALITYCOMPONENTWRAPPER_PORT_OUT,
            (xme_core_attribute_key_t) CONFIGURATOREXTENSION_ATTRIBUTE_QUALITY,
            &quality,
            sizeof(configuratorExtension_attribute_quality_t)
        );

        XME_LOG_C(XME_CORE_COMPONENT_TYPE_PUBLISHERLOWQUALITY, XME_LOG_ALWAYS, "sending: %d (quality: %d)\n", portOutData.value, quality);

        // PROTECTED REGION END
    }
    
    status[0] = configuratorExtension_adv_publisherLowQuality_publisherLowQualityComponentWrapper_writePortOut(portOutDataPtr);
    
    {
        // PROTECTED REGION ID(CONFIGURATOREXTENSION_ADV_PUBLISHERLOWQUALITY_SENDFUNCTION_STEP_2_C) ENABLED START
        
        // Nothing to do
        
        // PROTECTED REGION END
    }
}

void
configuratorExtension_adv_publisherLowQuality_sendFunction_fini
(
    configuratorExtension_adv_publisherLowQuality_publisherLowQualityComponent_config_t* const componentConfig
)
{
    // PROTECTED REGION ID(CONFIGURATOREXTENSION_ADV_PUBLISHERLOWQUALITY_SENDFUNCTION_TERMINATE_C) ENABLED START
    
    // Nothing to do
    
    XME_UNUSED_PARAMETER(componentConfig);

    // PROTECTED REGION END
}

// PROTECTED REGION ID(CONFIGURATOREXTENSION_ADV_PUBLISHERLOWQUALITY_SENDFUNCTION_IMPLEMENTATION_C) ENABLED START

// PROTECTED REGION END
