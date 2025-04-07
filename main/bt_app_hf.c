/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"

#include "bt_app_core.h"
#include "bt_app_hf.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "driver/i2s.h"
#include "time.h"
#include "sys/time.h"
#include "sdkconfig.h"

const char *c_hf_evt_str[] = {
    "CONNECTION_STATE_EVT",              /*!< connection state changed event */
    "AUDIO_STATE_EVT",                   /*!< audio connection state change event */
    "VR_STATE_CHANGE_EVT",                /*!< voice recognition state changed */
    "CALL_IND_EVT",                      /*!< call indication event */
    "CALL_SETUP_IND_EVT",                /*!< call setup indication event */
    "CALL_HELD_IND_EVT",                 /*!< call held indicator event */
    "NETWORK_STATE_EVT",                 /*!< network state change event */
    "SIGNAL_STRENGTH_IND_EVT",           /*!< signal strength indication event */
    "ROAMING_STATUS_IND_EVT",            /*!< roaming status indication event */
    "BATTERY_LEVEL_IND_EVT",             /*!< battery level indication event */
    "CURRENT_OPERATOR_EVT",              /*!< current operator name event */
    "RESP_AND_HOLD_EVT",                 /*!< response and hold event */
    "CLIP_EVT",                          /*!< Calling Line Identification notification event */
    "CALL_WAITING_EVT",                  /*!< call waiting notification */
    "CLCC_EVT",                          /*!< listing current calls event */
    "VOLUME_CONTROL_EVT",                /*!< audio volume control event */
    "AT_RESPONSE",                       /*!< audio volume control event */
    "SUBSCRIBER_INFO_EVT",               /*!< subscriber information event */
    "INBAND_RING_TONE_EVT",              /*!< in-band ring tone settings */
    "LAST_VOICE_TAG_NUMBER_EVT",         /*!< requested number from AG event */
    "RING_IND_EVT",                      /*!< ring indication event */
    "PKT_STAT_EVT",                      /*!< requested number of packet status event */
    "PROF_STATE_EVT",                    /*!< Indicate HF CLIENT init or deinit complete */
};
// esp_hf_client_connection_state_t
const char *c_connection_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "slc_connected",
    "disconnecting",
};

// esp_hf_client_audio_state_t
const char *c_audio_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "connected_msbc",
};

/// esp_hf_vr_state_t
const char *c_vr_state_str[] = {
    "disabled",
    "enabled",
};

// esp_hf_service_availability_status_t
const char *c_service_availability_status_str[] = {
    "unavailable",
    "available",
};

// esp_hf_roaming_status_t
const char *c_roaming_status_str[] = {
    "inactive",
    "active",
};

// esp_hf_client_call_state_t
const char *c_call_str[] = {
    "NO call in progress",
    "call in progress",
};

// esp_hf_client_callsetup_t
const char *c_call_setup_str[] = {
    "NONE",
    "INCOMING",
    "OUTGOING_DIALING",
    "OUTGOING_ALERTING"
};

// esp_hf_client_callheld_t
const char *c_call_held_str[] = {
    "NONE held",
    "Held and Active",
    "Held",
};

// esp_hf_response_and_hold_status_t
const char *c_resp_and_hold_str[] = {
    "HELD",
    "HELD ACCEPTED",
    "HELD REJECTED",
};

// esp_hf_client_call_direction_t
const char *c_call_dir_str[] = {
    "outgoing",
    "incoming",
};

// esp_hf_client_call_state_t
const char *c_call_state_str[] = {
    "active",
    "held",
    "dialing",
    "alerting",
    "incoming",
    "waiting",
    "held_by_resp_hold",
};

// esp_hf_current_call_mpty_type_t
const char *c_call_mpty_type_str[] = {
    "single",
    "multi",
};

// esp_hf_volume_control_target_t
const char *c_volume_control_target_str[] = {
    "SPEAKER",
    "MICROPHONE"
};

// esp_hf_at_response_code_t
const char *c_at_response_code_str[] = {
    "OK",
    "ERROR"
    "ERR_NO_CARRIER",
    "ERR_BUSY",
    "ERR_NO_ANSWER",
    "ERR_DELAYED",
    "ERR_BLACKLILSTED",
    "ERR_CME",
};

// esp_hf_subscriber_service_type_t
const char *c_subscriber_service_type_str[] = {
    "unknown",
    "voice",
    "fax",
};

// esp_hf_client_in_band_ring_state_t
const char *c_inband_ring_state_str[] = {
    "NOT provided",
    "Provided",
};

extern esp_bd_addr_t peer_addr;
// If you want to connect a specific device, add it's address here
// esp_bd_addr_t peer_addr = {0xac, 0x67, 0xb2, 0x53, 0x77, 0xbe};

#if CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI
#define ESP_HFP_RINGBUF_SIZE (64 * 1024)    // Adjustel;d buffer size to manage memory and performance
static RingbufHandle_t m_rb = NULL;
#endif


static void bt_app_hf_client_audio_open(void)
{
    m_rb = xRingbufferCreate(ESP_HFP_RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
}

// Close the audio data path (ring buffer)
static void bt_app_hf_client_audio_close(void)
{
    if (!m_rb) {
        return;
    }
    vRingbufferDelete(m_rb);
}
static TaskHandle_t s_mic_task_handle = NULL;
static void inmp441_reader_task(void *arg)
{
    size_t bytes_read;
    const size_t frame_size = 512;
    uint8_t i2s_buffer[frame_size];

    while (1) {
        if (i2s_read(I2S_NUM_0, i2s_buffer, frame_size, &bytes_read, portMAX_DELAY) == ESP_OK && bytes_read > 0) {
            if (xRingbufferSend(m_rb, i2s_buffer, bytes_read, pdMS_TO_TICKS(10)) != pdTRUE) {
                size_t tmp_size;
                uint8_t *old = (uint8_t *)xRingbufferReceiveUpTo(m_rb, &tmp_size, 0, bytes_read);
                if (old) {
                    vRingbufferReturnItem(m_rb, old);  // Return the old item to prevent overflow
                    if (xRingbufferSend(m_rb, i2s_buffer, bytes_read, pdMS_TO_TICKS(10)) != pdTRUE) {
                        ESP_LOGW("MIC", "Dropped audio, buffer still full after eviction");
                    }
                } else {
                    ESP_LOGW("MIC", "Dropped audio, no items to evict");
                }
            }
            esp_hf_client_outgoing_data_ready();  // Always notify after writing data
        }
    }
}


static void start_mic_reader_task(void)
{
    if (s_mic_task_handle == NULL) {
        xTaskCreatePinnedToCore(inmp441_reader_task, "inmp441_mic_task", 4096, NULL, 3, &s_mic_task_handle, 1);  // Lower prio, core 1
    }
}

static void stop_mic_reader_task(void)
{
    if (s_mic_task_handle != NULL) {
        vTaskDelete(s_mic_task_handle);
        s_mic_task_handle = NULL;
    }
}

uint32_t bt_app_hf_client_outgoing_cb(uint8_t *p_buf, uint32_t sz)
{
    static uint32_t call_count = 0;
    call_count++;
    ESP_LOGV("HFP_OUT", "[%lu] Requesting %lu bytes", call_count, sz);

    size_t item_size;
    uint8_t *data = (uint8_t *)xRingbufferReceiveUpTo(m_rb, &item_size, 0, sz);
if (!data) {
    // Handle case where no data is available (return silence)
    ESP_LOGW("HFP_OUT", "No data available for outgoing buffer, sending silence");
    memset(p_buf, 0, sz);  // Return silence
    return sz;
}
memcpy(p_buf, data, item_size);
vRingbufferReturnItem(m_rb, data);
return item_size;
}

static void bt_app_hf_client_incoming_cb(const uint8_t *buf, uint32_t sz)
{
    if (!m_rb) {
        return;
    }

    // Try to add incoming audio data to the ring buffer
    BaseType_t done = xRingbufferSend(m_rb, (uint8_t *)buf, sz, 0);
    if (!done) {
        ESP_LOGE(BT_HF_TAG, "rb send fail");
    }

    esp_hf_client_outgoing_data_ready();
}
 /* #if CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI */
/* callback for HF_CLIENT */
void bt_app_hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    if (event <= ESP_HF_CLIENT_PROF_STATE_EVT) {
        ESP_LOGI(BT_HF_TAG, "APP HFP event: %s", c_hf_evt_str[event]);
    } else {
        ESP_LOGE(BT_HF_TAG, "APP HFP invalid event %d", event);
    }

    switch (event) {
        case ESP_HF_CLIENT_CONNECTION_STATE_EVT:
            ESP_LOGI(BT_HF_TAG, "--connection state %s", c_connection_state_str[param->conn_stat.state]);
            memcpy(peer_addr, param->conn_stat.remote_bda, ESP_BD_ADDR_LEN);
            break;

        case ESP_HF_CLIENT_AUDIO_STATE_EVT:
            ESP_LOGI(BT_HF_TAG, "--audio state %s", c_audio_state_str[param->audio_stat.state]);

            #if CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI
            if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
                param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
                esp_hf_client_register_data_callback(bt_app_hf_client_incoming_cb, bt_app_hf_client_outgoing_cb);
                bt_app_hf_client_audio_open();
                start_mic_reader_task(); // Start mic when audio is connected
            } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED) {
                stop_mic_reader_task(); // Stop mic when audio is disconnected
                bt_app_hf_client_audio_close();
            }
            #endif
            break;

        // Handle other events as necessary...
        default:
            ESP_LOGE(BT_HF_TAG, "Unknown event %d", event);
            break;
    }
}