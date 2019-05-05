/**
 * Copyright (c) 2019, Tomas Hodor
 *
 * Help by NORDIC SEMICONDUCTOR, provided libraries and drivers for 
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "crc16.h"
#include "fds.h"
#include "fds_record.h"
#include "bp_library.h"

#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_spi.h"

#include "nrf_drv_timer.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "BP_Hodor"                                  /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};
//######################################################################################################### SPI
#define SPI_INSTANCE                    0                                           /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);                /**< SPI instance. */
static volatile bool spi_xfer_done;                                                 /**< Flag used to indicate that SPI instance completed the transfer. */

static uint8_t       m_tx_buf[] = {0x00,0xFF,0xFF,0xFF,0xFF};                       /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(m_tx_buf)];                                  /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);                                   /**< Transfer length. */

//######################################################################################################### FDS
static bool volatile m_fds_initialized;
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;

static void fds_evt_handler(fds_evt_t const * p_evt) {
    switch (p_evt->id) {
        case FDS_EVT_INIT:
            if (p_evt->result == FDS_SUCCESS) {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == FDS_SUCCESS) {
                NRF_LOG_INFO("Record ID:\t0x%04x\n",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x\n",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x\n", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == FDS_SUCCESS) {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            m_delete_all.pending = false;
        } break;

        default:
            break;
    }
}
static void power_manage(void) {
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
}
static void wait_for_fds_ready(void) {
    while (!m_fds_initialized) {
        power_manage();
    }
}

bool record_delete_next(void) {
    fds_find_token_t  tok   = {0};
    fds_record_desc_t desc  = {0};

    if (fds_record_iterate(&desc, &tok) == FDS_SUCCESS) {
        ret_code_t rc = fds_record_delete(&desc);
        if (rc != FDS_SUCCESS)
            return false; 

        return true;
    } else {
        /* No records left to delete. */
        return false;
    }
}
void delete_all_process(void){
    if (   true
        & !m_delete_all.pending) {
        NRF_LOG_INFO("Deleting next record.");

        m_delete_all.delete_next = record_delete_next();
        if (!m_delete_all.delete_next)
            NRF_LOG_INFO("No records left to delete.");
        
    }
}
//######################################################################################################### set config file
static configuration_t m_dummy_cfg =
{
    .timestamp = 0x0,
    .sign_active_power = 0x0,
    .sign_reactive_power = 0x0,
    .overflow_active_energy = 0x0,
    .overflow_reactive_energy = 0x0,
    .data1 = 0x55,
    .crc = 0x11,
};

static fds_record_t const m_dummy_record =
{
    .file_id           = CONFIG_FILE,
    .key               = CONFIG_REC_KEY,
    .data.p_data       = &m_dummy_cfg,
    .data.length_words = (sizeof(m_dummy_cfg) + 3) / sizeof(uint32_t),
};
//######################################################################################################### init fileID and key
/**@brief timestamp for data, by default start at 0 */
uint32_t timestamp = 0;
/**@brief fileID for NFS, by default start at 1 */
uint16_t fileID = 0x01;
/**@brief Key for NFS, by default start at 1 */
uint16_t key = 0x01;
/**@brief Delay of reading data, by default is 5 seconds */
int delay = 5; 

bool delete_all_data = false;
//######################################################################################################### send data
/**@brief Function for sending strings from BLE */
uint32_t send_ble(unsigned char data_arr[]) {
    uint32_t err_code;
    uint16_t length = (uint16_t)strlen(data_arr);
    do {
        err_code = ble_nus_data_send(&m_nus, data_arr, &length, m_conn_handle);

        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND)) {
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_BUSY);
    printf("%s - sended\n",data_arr);
    return err_code;
}
/** NRF Default function */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}
/** NRF Default function */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}
/** NRF Default function */
static void gap_params_init(void) {
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}
/** NRF Default function */
static void nrf_qwr_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Handling receiving data from BLE */
static void nus_data_handler(ble_nus_evt_t * p_evt) {
    uint32_t err_code;
    if (p_evt->type == BLE_NUS_EVT_RX_DATA) {
        char * data_char = (char *) p_evt->params.rx_data.p_data;
        switch (data_char[0]) {
          case 'T': 
              data_char = remove_first_char(data_char);
              timestamp = atoi(data_char);

              fds_record_desc_t desc = {0};
              fds_find_token_t  tok  = {0};

              ret_code_t rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

              if (rc == FDS_SUCCESS) {
                  fds_flash_record_t config = {0};

                  rc = fds_record_open(&desc, &config);
                  APP_ERROR_CHECK(rc);

                  memcpy(&m_dummy_cfg, config.p_data, sizeof(configuration_t));

                  printf("Updating Timestamp to \t0x%04x\n", timestamp);
                  m_dummy_cfg.timestamp = timestamp;

                  rc = fds_record_close(&desc);
                  APP_ERROR_CHECK(rc);

                  rc = fds_record_update(&desc, &m_dummy_record);
                  APP_ERROR_CHECK(rc);

                  send_ble("Timestamp recieved, updating config record");
              } else {
                  printf("Timestamp was not found\n");
              }
              break;
          case 'D':   /**Set delay */
              data_char = remove_first_char(data_char);
              delay = atoi(data_char);
              break;
              
          case 'A':   /**BLE send all data */
              {
                  int records_found = 0;

                  fds_record_desc_t desc = {0};
                  fds_find_token_t  tok  = {0};
                  while (fds_record_iterate(&desc, &tok) != FDS_ERR_NOT_FOUND) {

                      fds_flash_record_t record = {0};
                      ret_code_t rc = fds_record_open(&desc, &record);

                      if (rc == FDS_SUCCESS && record.p_header->record_key != CONFIG_REC_KEY) {
                          records_found++;
                          configuration_t * p_cfg = (configuration_t *)(record.p_data); 
                          //printf("KEY:\t0x%04x \n", record.p_header->record_key);
                         
                          unsigned char time_string[] = "";
                          itoa(p_cfg->timestamp, time_string, 10);
                          err_code = send_ble(time_string);
                          APP_ERROR_CHECK(err_code);

                          unsigned char data1[] = "";
                          itoa(p_cfg->data1, data1, 10);
                          err_code = send_ble(data1);
                          APP_ERROR_CHECK(err_code);
                          //nrf_delay_ms(50);

                          rc = fds_record_close(&desc);
                          APP_ERROR_CHECK(rc); 
                      } else {
                          continue;
                      }
                  }
                  printf("%d found records\n", records_found);

                  if (!records_found) {
                      char data_arr[] = "No saved data";
                      err_code = send_ble(data_arr);
                  }
              }
              break;
          default:
              break;
        }
    }

}
/** NRF Default function */
static void services_init(void) {
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}
/** NRF Default function */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt) {
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}
/** NRF Default function */
static void conn_params_error_handler(uint32_t nrf_error) {
    APP_ERROR_HANDLER(nrf_error);
}
/** NRF Default function */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}
/** NRF Default function */
static void sleep_mode_enter(void) {
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}
/** NRF Default function */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
    uint32_t err_code;

    switch (ble_adv_evt){
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}
/** NRF Default function */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            printf("Connected\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}
/** NRF Default function */
static void ble_stack_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
/** NRF Default function */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}
/** NRF Default function */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}
/** NRF Default function */
void bsp_event_handler(bsp_event_t event) {
    uint32_t err_code;
    switch (event) {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE) {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                    APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}
//######################################################################################################################### NEW DATA
configuration_t newDataInFile (uint32_t timestamp,
                    uint32_t data1) {
  configuration_t data =
{
    .timestamp = timestamp,
    .sign_active_power = 0x0,
    .sign_reactive_power = 0x0,
    .overflow_active_energy = 0x0,
    .overflow_reactive_energy = 0x0,
    .data1 = data1,
    .crc = 0x0, //crc16_compute(
};
  return data;
}
//######################################################################################################################### WRITE FSD
void static write_fds (void const * p_data) {
  fds_record_desc_t desc = {0};
  fds_record_t const rec =
    {
        .file_id           = fileID,
        .key               = key,
        .data.p_data       = p_data,
        .data.length_words = (sizeof(configuration_t) + 3) / sizeof(uint32_t),
    };
  if (key != FDS_RECORD_KEY_DIRTY && fileID != FDS_FILE_ID_INVALID) {
      configuration_t * p_cfg = (configuration_t *)(p_data);
      ret_code_t rc = fds_record_write(&desc, &rec);
      APP_ERROR_CHECK(rc);

      if (rc != FDS_SUCCESS){
          bsp_board_led_on(BSP_BOARD_LED_3);
      } else {
          printf("WRITE COMPLETE - KEY:%d\n", key);
          key++;
      }
  }

}
//######################################################################################################### print all data from FDS
void print_all_data() {
    int records_found = 0;
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};
    printf("PRINT FOUND\n");
    while (fds_record_iterate(&desc, &tok) != FDS_ERR_NOT_FOUND) {

        fds_flash_record_t record = {0};
        ret_code_t rc = fds_record_open(&desc, &record);

        if (rc == FDS_SUCCESS) {
            records_found++;
            configuration_t * p_cfg = (configuration_t *)(record.p_data);
            printf("KEY:\t0x%04x DATA:\t0x%04x T:\t0x%04x, 1:\t0x%04x, c:\t0x%04x\n\n",
            record.p_header->record_key,record.p_data, p_cfg->timestamp, p_cfg->data1, p_cfg->crc);
       
            nrf_delay_ms(50);
            rc = fds_record_close(&desc);
            APP_ERROR_CHECK(rc);
        } else {
            continue;
        }
    }
    printf("%d found records\n", records_found);
}

//######################################################################################################################### UART SEND
void uart_event_handle(app_uart_evt_t * p_event) {
    static uint8_t send_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;
    unsigned char data_arr[] = "";
    switch (p_event->evt_type) {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&send_array[index]));
            index++;
 
            if ((send_array[index - 1] == '\n') || (index >= m_ble_nus_max_data_len)) {
              if (index > 1) {
                do {
                  if (timestamp != 0) {
                    send_array[--index] = 0;
                    char * num_char = (char *) send_array;
                    uint32_t  num = atoi(num_char);

                    configuration_t newData = newDataInFile(timestamp, num);
                    write_fds(&newData);

                    print_all_data();
                    sprintf(data_arr, "%d", newData.data1);
                  } else {
                    strcpy(data_arr,"No timestamp");
                  }
                  //do {
                      err_code = send_ble(data_arr);
                      timestamp += delay;
                      //nrf_delay_ms(1000 * delay); 
                  //} while (delay);

                } while (err_code == NRF_ERROR_RESOURCES);
              }
              index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
//######################################################################################################### handling SPI
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context) {
    spi_xfer_done = true;
    printf("Transfer:\t0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",m_tx_buf[0],m_tx_buf[1],m_tx_buf[2],m_tx_buf[3],m_tx_buf[4]);
    if (m_rx_buf[0] != 0) {
        printf("Received:\t0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",m_rx_buf[0],m_rx_buf[1],m_rx_buf[2],m_rx_buf[3],m_rx_buf[4]);
        uint8_t  data[] = {0, 0, 0, 0};
        mirror_data(m_rx_buf, sizeof(m_rx_buf), data);
        for (int i = 0; i< sizeof(data); i++) {
            printf("0x%02x ",data[i]);
        }
        printf("\n");
    }
}

static void read_spi_data() {
    for(uint8_t i = 0x00; i <= 0x04; i +=2 ) {
        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;
        uint8_t buff[] = {i,0xFF,0xFF,0xFF};
        m_tx_buf[0] = i;
        m_tx_buf[4] = crc8(buff, sizeof(buff));
        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
        while (!spi_xfer_done) {
                __WFE();
        }
        nrf_delay_ms(50);
   }
   nrf_delay_ms(60*1000);
}
/** NRF Default function */
static void uart_init(void) {
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/** NRF Default function */
static void advertising_init(void) {
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}
/** NRF Default function */
static void buttons_leds_init(bool * p_erase_bonds) {
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}
/** NRF Default function */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
/** NRF Default function */
static void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}
/** NRF Default function */
static void idle_state_handle(void) {
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}
/** NRF Default function */
static void advertising_start(void) {
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

/**@brief Application main function.
 */
int main(void) {
    bool erase_bonds;
    ret_code_t rc;

    // Initialize.
    uart_init();
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    spi_config.mode = NRF_SPI_MODE_3;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    printf("\n");
    (void) fds_register(fds_evt_handler);
    rc = fds_init();
    APP_ERROR_CHECK(rc);

    /* Wait for fds to initialize. */
    wait_for_fds_ready();

    if (delete_all_data) {
        delete_all_process();
    }

    fds_stat_t stat = {0};
    rc = fds_stat(&stat);
    APP_ERROR_CHECK(rc);

    printf("Found %d valid records\n", stat.valid_records);
    //printf("Found %d dirty records\n", stat.dirty_records);

    if (stat.dirty_records > 0)
      fds_gc();
    
    key += stat.valid_records - 1;

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    print_all_data();

    if (fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok) == FDS_SUCCESS) {
        fds_flash_record_t config = {0};
        rc = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(rc);

        memcpy(&m_dummy_cfg, config.p_data, sizeof(configuration_t));
        timestamp = m_dummy_cfg.timestamp;
        printf("Config file found, Timestamp: %d. Updated crc: %d\n", m_dummy_cfg.timestamp, ++m_dummy_cfg.crc);

        rc = fds_record_close(&desc);
        APP_ERROR_CHECK(rc);
        rc = fds_record_update(&desc, &m_dummy_record);
        APP_ERROR_CHECK(rc);
    } else {
        printf("Writing config file...\n");
        rc = fds_record_write(&desc, &m_dummy_record);
        APP_ERROR_CHECK(rc);
    }
    advertising_start();

    for (;;) {
        read_spi_data();
        idle_state_handle();
    }
}


/**
 * @}
 */
