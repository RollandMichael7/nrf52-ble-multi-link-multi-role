/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
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
#include "sdk_common.h"

#include "ble_thingy_battery_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_srv_common.h"
#include "ble_gattc.h"
#define NRF_LOG_MODULE_NAME ble_thingy_battery_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define TX_BUFFER_MASK         0x07                  /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         (TX_BUFFER_MASK + 1)  /**< Size of send buffer, which is 1 higher than the mask. */

/**@brief Structure for holding data to be transmitted to the connected central.
 */
typedef struct
{
    uint16_t     conn_handle;  /**< Connection handle to be used when transmitting this message. */
    uint16_t     read_handle;  /**< Read request message. */
} tx_message_t;


static tx_message_t m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for messages to be transmitted to the central. */
static uint32_t     m_tx_insert_index = 0;        /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t     m_tx_index = 0;               /**< Current index in the transmit buffer from where the next message to be transmitted resides. */

static uint8_t m_uuid_type;


uint32_t ble_thingy_battery_c_init(ble_thingy_battery_c_t * p_ble_thingy_battery_c, ble_thingy_battery_c_init_t * p_ble_thingy_battery_c_init) {
        p_ble_thingy_battery_c->evt_handler = p_ble_thingy_battery_c_init->evt_handler;
        p_ble_thingy_battery_c->conn_handle = BLE_CONN_HANDLE_INVALID;

        uint32_t err_code;
        ble_uuid128_t thingy_battery_base_uuid = {THINGY_BATTERY_UUID_BASE};
        err_code = sd_ble_uuid_vs_add(&thingy_battery_base_uuid, &m_uuid_type);
        VERIFY_SUCCESS(err_code);
        return err_code;
}

/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
    if (m_tx_index != m_tx_insert_index)
    {
        ble_uuid_t uuid = {.uuid = THINGY_BATTERY_UUID_SERVICE, .type = m_uuid_type};
        ble_gattc_handle_range_t range = {.start_handle = 0x0001, .end_handle = 0xffff};
        uint32_t err_code = sd_ble_gattc_char_value_by_uuid_read(m_tx_buffer[m_tx_index].conn_handle, &uuid, &range);

        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("SD Read/Write API returns Success..");
            m_tx_index++;
            m_tx_index &= TX_BUFFER_MASK;
        }
        else
        {
            NRF_LOG_DEBUG("SD Read/Write API returns error code %d. This message sending will be "
                "attempted again..", err_code);
        }
    }
}

/**@brief Function for handling read response events.
 *
 * @param[in] p_ble_thingy_battery_c Pointer to the Battery Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_read_rsp(ble_thingy_battery_c_t * p_ble_thingy_battery_c, ble_evt_t const * p_ble_evt)
{
    // Check if the event if on the link for this instance
    if (p_ble_thingy_battery_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    ble_thingy_battery_c_evt_t evt;
    evt.conn_handle = p_ble_thingy_battery_c->conn_handle;
    evt.evt_type = BLE_THINGY_BATTERY_C_EVT_NOTIFICATION;
   
    ble_gattc_handle_value_t iter;
    memset(&iter, 0, sizeof(ble_gattc_handle_value_t));
    while( sd_ble_gattc_evt_char_val_by_uuid_read_rsp_iter(&p_ble_evt->evt.gattc_evt, &iter) == NRF_SUCCESS) {
        NRF_LOG_DEBUG("Battery level for conn %x: %d", p_ble_thingy_battery_c->conn_handle, *iter.p_value);
        evt.params.battery.value = *iter.p_value;
        p_ble_thingy_battery_c->evt_handler(&p_ble_thingy_battery_c, &evt);
    }
    tx_buffer_process();
}


/**@brief Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details This function will uses the Handle Value Notification received from the SoftDevice
 *          and checks if it is a notification of battery from the peer. If
 *          it is, this function will decode the state of the button and send it to the
 *          application.
 *
 * @param[in] p_ble_thingy_battery_c Pointer to the Battery Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_thingy_battery_c_t * p_ble_thingy_battery_c, ble_evt_t const * p_ble_evt)
{
    // Check if the event is on the link for this instance
    if (p_ble_thingy_battery_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }
    // Check if this is a Battery notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_thingy_battery_c->peer_thingy_battery_db.battery_handle)
    {
        if (p_ble_evt->evt.gattc_evt.params.hvx.len == 1)
        {
            ble_thingy_battery_c_evt_t ble_thingy_battery_c_evt;

            ble_thingy_battery_c_evt.evt_type                   = BLE_THINGY_BATTERY_C_EVT_NOTIFICATION;
            ble_thingy_battery_c_evt.conn_handle                = p_ble_thingy_battery_c->conn_handle;
            ble_thingy_battery_c_evt.params.battery.value      = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            p_ble_thingy_battery_c->evt_handler(p_ble_thingy_battery_c, &ble_thingy_battery_c_evt);
        }
    }
}


/**@brief Function for handling Disconnected event received from the SoftDevice.
 *
 * @details This function check if the disconnect event is happening on the link
 *          associated with the current instance of the module, if so it will set its
 *          conn_handle to invalid.
 *
 * @param[in] p_ble_thingy_battery_c Pointer to the Battery Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_thingy_battery_c_t * p_ble_thingy_battery_c, ble_evt_t const * p_ble_evt)
{
    if (p_ble_thingy_battery_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_thingy_battery_c->conn_handle                    = BLE_CONN_HANDLE_INVALID;
        p_ble_thingy_battery_c->peer_thingy_battery_db.battery_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_thingy_battery_c->peer_thingy_battery_db.battery_handle      = BLE_GATT_HANDLE_INVALID;
    }
}

void ble_thingy_battery_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    ble_thingy_battery_c_t * p_ble_thingy_battery_c = (ble_thingy_battery_c_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_thingy_battery_c, p_ble_evt);
            break;

        case BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP:
            on_read_rsp(p_ble_thingy_battery_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_thingy_battery_c, p_ble_evt);
            break;

        default:
            break;
    }
}

uint32_t ble_thingy_battery_c_handles_assign(ble_thingy_battery_c_t    * p_ble_thingy_battery_c,
                                  uint16_t         conn_handle,
                                  const thingy_battery_db_t * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_thingy_battery_c);

    p_ble_thingy_battery_c->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_thingy_battery_c->peer_thingy_battery_db = *p_peer_handles;
    }
    return NRF_SUCCESS;
}

uint32_t ble_thingy_battery_c_read(ble_thingy_battery_c_t * p_ble_thingy_battery_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_thingy_battery_c);

    if (p_ble_thingy_battery_c->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    NRF_LOG_INFO("Reading Thingy battery");
    tx_message_t * p_msg;

    p_msg              = &m_tx_buffer[m_tx_insert_index++];
    m_tx_insert_index &= TX_BUFFER_MASK;

    p_msg->read_handle = THINGY_BATTERY_UUID_SERVICE;
    p_msg->conn_handle = p_ble_thingy_battery_c->conn_handle;

    tx_buffer_process();

    return NRF_SUCCESS;
}