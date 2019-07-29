/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#ifndef BLE_THINGY_BATTERY_C_H__
#define BLE_THINGY_BATTERY_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "nrf_sdh_ble.h"
#include "app_util_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_thingy_battery_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_THINGY_BATTERY_C_DEF(_name)                                                                        \
static ble_thingy_battery_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_THINGY_BATTERY_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_thingy_battery_c_on_ble_evt, &_name)

/**@brief   Macro for defining multiple ble_thingy_battery_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 */
#define BLE_THINGY_BATTERY_C_ARRAY_DEF(_name, _cnt)                                                            \
static ble_thingy_battery_c_t _name[_cnt];                                                                     \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                                                                \
                      BLE_THINGY_BATTERY_C_BLE_OBSERVER_PRIO,                                                  \
                      ble_thingy_battery_c_on_ble_evt, &_name, _cnt)

#define THINGY_BATTERY_UUID_BASE        {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}
#define THINGY_BATTERY_UUID_SERVICE     0x2A19

/**@brief THINGY_UIS Client event type. */
typedef enum
{
    BLE_THINGY_BATTERY_C_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the Battery Service has been discovered at the peer. */
    BLE_THINGY_BATTERY_C_EVT_NOTIFICATION      /**< Event indicating that a notification of the Battery characteristic has been received from the peer. */
} ble_thingy_battery_c_evt_type_t;

typedef struct {
  uint8_t value;
} ble_thingy_battery_t;

/**@brief Structure containing the handles related to the Battery Service found on the peer. */
typedef struct
{
    uint16_t battery_cccd_handle;  /**< Handle of the CCCD of the battery characteristic. */
    uint16_t battery_handle;       /**< Handle of the battery characteristic as provided by the SoftDevice. */
} thingy_battery_db_t;

/**@brief Battery Event structure. */
typedef struct
{
    ble_thingy_battery_c_evt_type_t evt_type;    /**< Type of the event. */
    uint16_t             conn_handle; /**< Connection handle on which the event occured.*/
    union
    {
        ble_thingy_battery_t battery;      /**< Battery Value received. This will be filled if the evt_type is @ref BLE_THINGY_BATTERY_C_EVT_NOTIFICATION. */
        thingy_battery_db_t     peer_db;         /**< Battery Service related handles found on the peer device. This will be filled if the evt_type is @ref BLE_THINGY_BATTERY_C_EVT_DISCOVERY_COMPLETE.*/
    } params;
} ble_thingy_battery_c_evt_t;

// Forward declaration of the ble_thingy_battery_c_t type.
typedef struct ble_thingy_battery_c_s ble_thingy_battery_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_thingy_battery_c_evt_handler_t) (ble_thingy_battery_c_t * p_ble_thingy_battery_c, ble_thingy_battery_c_evt_t * p_evt);

/**@brief Battery Client structure. */
struct ble_thingy_battery_c_s
{
    uint16_t                conn_handle;  /**< Connection handle as provided by the SoftDevice. */
    thingy_battery_db_t         peer_thingy_battery_db;  /**< Handles related to THINGY_BATTERY on the peer*/
    ble_thingy_battery_c_evt_handler_t evt_handler;  /**< Application event handler to be called when there is an event related to the Battery service. */
    uint8_t                 uuid_type;    /**< UUID type. */
};

/**@brief Battery Client initialization structure. */
typedef struct
{
    ble_thingy_battery_c_evt_handler_t evt_handler;  /**< Event handler to be called by the Battery Client module whenever there is an event related to the Battery Service. */
} ble_thingy_battery_c_init_t;


/**@brief Function for initializing the Battery client module.
 *
 * @details This function will register with the DB Discovery module. There it registers for the
 *          Battery Service. Doing so will make the DB Discovery module look for the presence
 *          of a Battery Service instance at the peer when a discovery is started.
 *
 * @param[in] p_ble_thingy_battery_c      Pointer to the Battery client structure.
 * @param[in] p_ble_thingy_battery_c_init Pointer to the Battery initialization structure containing the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. Otherwise an error code. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_thingy_battery_c_init(ble_thingy_battery_c_t * p_ble_thingy_battery_c, ble_thingy_battery_c_init_t * p_ble_thingy_battery_c_init);


/**@brief Function for handling BLE events from the SoftDevice.
 *
 * @details This function will handle the BLE events received from the SoftDevice. If a BLE event
 *          is relevant to the Battery Client module, then it uses it to update interval
 *          variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the Battery client structure.
 */
void ble_thingy_battery_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for requesting the peer to start sending notification of the battery
 *        Characteristic.
 *
 * @details This function will enable to notification of the battery at the peer
 *          by writing to the CCCD of the battery Characteristic.
 *
 * @param[in] p_ble_thingy_battery_c Pointer to the Battery Client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 *          NRF_ERROR_INVALID_STATE if no connection handle has been assigned (@ref ble_thingy_battery_c_handles_assign)
 *          NRF_ERROR_NULL if the given parameter is NULL
 */
uint32_t ble_thingy_battery_c_battery_notif_enable(ble_thingy_battery_c_t * p_ble_thingy_battery_c);


/**@brief Function for handling events from the database discovery module.
 *
 * @details Call this function when getting a callback event from the DB discovery module. This
 *          function will handle an event from the database discovery module, and determine if it
 *          relates to the discovery of Battery service at the peer. If so, it will call the
 *          application's event handler indicating that the Battery service has been discovered
 *          at the peer. It also populates the event with the service related information before
 *          providing it to the application.
 *
 * @param[in] p_ble_thingy_battery_c Pointer to the Battery client structure.
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 */
void ble_thingy_battery_on_db_disc_evt(ble_thingy_battery_c_t * p_ble_thingy_battery_c, const ble_db_discovery_evt_t * p_evt);


/**@brief     Function for assigning a Handles to this instance of thingy_battery_c.
 *
 * @details Call this function when a link has been established with a peer to associate this link
 *          to this instance of the module. This makes it  possible to handle several links and
 *          associate each link to a particular instance of this module.
 *
 * @param[in] p_ble_thingy_battery_c    Pointer to the Battery client structure instance to associate.
 * @param[in] conn_handle    Connection handle to associate with the given Battery Client Instance.
 * @param[in] p_peer_handles Battery Service handles found on the peer (from @ref BLE_THINGY_battery_C_EVT_DISCOVERY_COMPLETE event).
 *
 */
uint32_t ble_thingy_battery_c_handles_assign(ble_thingy_battery_c_t *    p_ble_thingy_battery_c,
                                  uint16_t         conn_handle,
                                  const thingy_battery_db_t * p_peer_handles);

/** Read battery level */
uint32_t ble_thingy_battery_c_read(ble_thingy_battery_c_t * p_ble_thingy_battery_c);

/** Init module */
uint32_t ble_thingy_battery_c_init();


#ifdef __cplusplus
}
#endif

#endif // BLE_THINGY_weather_C_H__

/** @} */
