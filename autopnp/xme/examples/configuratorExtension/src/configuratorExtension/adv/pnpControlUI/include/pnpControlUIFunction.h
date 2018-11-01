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
 * $Id: pnpControlUIFunction.h 6351 2014-01-15 16:03:40Z geisinger $
 */

/**
 * \file
 *         Header file for function pnpControlUI in component pnpControlUI.
 *
 * \author
 *         This file has been generated by the CHROMOSOME Modeling Tool (XMT)
 *         (fortiss GmbH).
 */

#ifndef CONFIGURATOREXTENSION_ADV_PNPCONTROLUI_PNPCONTROLUIFUNCTION_H
#define CONFIGURATOREXTENSION_ADV_PNPCONTROLUI_PNPCONTROLUIFUNCTION_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "configuratorExtension/adv/pnpControlUI/include/pnpControlUIComponent.h"

#include "xme/core/component.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initialization of function. 
 *  
 * \details Called once before the function is executed the first time.
 *
 * \param[in,out] componentConfig Pointer to the component-specific configuration
 *                structure of this function's parent component instance.
 *                The configuration is first passed to the component instance
 *                initialization function and then subsequently to the
 *                initialization functions of all active functions of the
 *                component. It can be freely modified by this function.
 *
 * \return XME_STATUS_SUCCESS when initialization was successful.
 */
xme_status_t
configuratorExtension_adv_pnpControlUI_pnpControlUIFunction_init
(
    configuratorExtension_adv_pnpControlUI_pnpControlUIComponent_config_t* const componentConfig
);

/**
 * \brief Executes the function one time.
 *
 * \details Input ports of the function need to be prepared, before calling this.
 *
 * \param[in,out] componentConfig Pointer to the component-specific
 *                configuration structure of this function's parent component instance.
 *                The configuration is first passed to the component instance
 *                initialization function, then to the initialization functions
 *                of all active functions of the component and finally passed
 *                in every call to the step function.
 *                It can be freely modified by this function.
 
 * \param param Function-specific parameter
 */
void
configuratorExtension_adv_pnpControlUI_pnpControlUIFunction_step
(
    configuratorExtension_adv_pnpControlUI_pnpControlUIComponent_config_t* const componentConfig
);

/**
 * \brief Finalization function.
 *
 * \details Called after function will no longer be executed to free allocated resources.
 *          Function must not be executed after fini has been called.
 *
 * \param[in,out] componentConfig Pointer to the component-specific configuration
 *                structure of this function's parent component instance.
 *                The finalization function should free all function-specific
 *                resources in the configuration structure and update it accordingly.
 */
void
configuratorExtension_adv_pnpControlUI_pnpControlUIFunction_fini
(
    configuratorExtension_adv_pnpControlUI_pnpControlUIComponent_config_t* const componentConfig
);

XME_EXTERN_C_END

#endif // #ifndef CONFIGURATOREXTENSION_ADV_PNPCONTROLUI_PNPCONTROLUIFUNCTION_H
