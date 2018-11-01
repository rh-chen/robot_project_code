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
 * $Id: sensorKBComponentWrapper.h 7836 2014-03-14 12:20:24Z wiesmueller $
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

#ifndef SENSORMONITOR_ADV_SENSORKB_SENSORKBCOMPONENTWRAPPER_H
#define SENSORMONITOR_ADV_SENSORKB_SENSORKBCOMPONENTWRAPPER_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/executionManager/include/executionManagerDataStructures.h"

#include "sensorMonitor/topic/dictionaryData.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \enum sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalFunctionId_e
 *
 * \brief Values for identifying functions of sensorKB component.
 */
enum sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalFunctionId_e
{
    SENSORMONITOR_ADV_SENSORKB_SENSORKBCOMPONENTWRAPPER_FUNCTION_READSENSORVALUE = 0  ///< Function 'readSensorValue'
};

/**
 * \enum sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalPortId_e
 *
 * \brief Values for sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalPortId_t.
 */
enum sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalPortId_e
{
    SENSORMONITOR_ADV_SENSORKB_SENSORKBCOMPONENTWRAPPER_PORT_SENSORVALUEOUT = 0  ///< Port 'sensorValueOut'
};

/**
 * \typedef sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalPortId_t
 *
 * \brief Defines internal port ids of component 'sensorKB'.
 *
 * \details These can be used when calling the sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_receivePort function.
 *          For the definition of possible values, see enum sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalPortId_e.
 */
typedef uint8_t sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalPortId_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes this component wrapper.
 *
 * \retval XME_STATUS_SUCCESS on success.
 */
xme_status_t
sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_init(void);

/**
 * \brief Finalizes this component wrapper.
 */
void
sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_fini(void);

/**
 * \brief Associate an internal port number with the corresponding port handle.
 *        For the ids of the individual ports, see the definition of sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalPortId_t.
 *
 * \param dataPacketId Port handle from the dataHandler.
 * \param componentInternalPortId Component internal port number of the above port.
 *
 * \retval XME_STATUS_SUCCESS if no problems occurred.
 * \retval XME_STATUS_INVALID_PARAMETER if componentInternalPortId is unknown.
 */
xme_status_t
sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_receivePort
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalPortId_t componentInternalPortId
);

/**
 * \brief This function is called by the function wrapper after the step
 *        function has been called. It signals to the middleware that all
 *        write operations on ports that actually have been written to
 *        in the step function (via the functions in this component wrapper)
 *        are now completed.
 */
void
sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_completeWriteOperations(void);

/**
 * \brief Write data to port 'sensorValueOut'.
 *
 * \note The write operation is only allowed to be called once per
 *       data packet sending process. A data packet is sent 
 *       as soon as the sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_writeNextPacket()
 *       function is being called or when the step function
 *       returns and data have been written.
 * 
 * \param[in] data User provided storage, from which the data is copied.
 *            When NULL no data will be written to the port (this
 *            is also treated as  success).
 *
 * \retval XME_STATUS_SUCCESS if operation was successful.
 */
xme_status_t
sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_writePortSensorValueOut
(
    const sensorMonitor_topic_sensorData_t* const data
);

xme_status_t
sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_writeOutputPortAttribute
(
    sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalPortId_t portId,
    xme_core_attribute_key_t attributeKey,
    const void* const buffer,
    uint32_t bufferSize
);

xme_status_t
sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_writeNextPacket
(
    sensorMonitor_adv_sensorKB_sensorKBComponentWrapper_internalPortId_t portId
);


XME_EXTERN_C_END

#endif // #ifndef SENSORMONITOR_ADV_SENSORKB_SENSORKBCOMPONENTWRAPPER_H
