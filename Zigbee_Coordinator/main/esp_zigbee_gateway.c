#include <fcntl.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "driver/usb_serial_jtag.h"
#include "esp_coexist.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_zigbee_gateway.h"
#include "zb_config_platform.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

//I2C connection to RPi -- will send disconnects and rtt
static const bool DO_I2C_REPORTING = true; //! used for turning on/off RPi reading easier
#define I2C_SLAVE_ADDR 0x28
#define I2C_SLAVE_SCL_IO 11
#define I2C_SLAVE_SDA_IO 10
#define I2C_SLAVE_NUM I2C_NUM_0
#define I2C_BUF_LEN 128
#define RTT_BYTE_IDENTIFIER 0xB2
#define DISCONNECT_BYTE_IDENTIFIER_WATCHDOG 0xA1
#define DISCONNECT_BYTE_IDENTIFIER_NLME 0xA2
#define DISCONNECT_BYTE_IDENTIFIER_DEV_UNAVAILABLE 0xA3


//Vibration Sensor data collection
#define PIEZO_ADC_CHANNEL    ADC_CHANNEL_2  // GPIO2
#define PIEZO_ADC_UNIT          ADC_UNIT_1
#define SENSOR_DATA_COMMAND_REQ_ID          0x0002
#define SENSOR_READING_DELAY 15000 
static uint16_t sensor_value = 0; 
adc_oneshot_unit_handle_t adc_handle;
static bool created_sensor_task = false; // Prevent multiple task creation, currently starts in ANNCE signal when new device joins network

//Cluster and Endpoints
#define ESP_ZB_ROUTER_ENDPOINT        0x02 //Not necessary for endpoints on different devices to differ, however it allows for easier tracking
#define ESP_ZB_COORDINATOR_ENDPOINT   0x01 
#define CUSTOM_CLUSTER_ID           0xFF00

//Ensure network parameter consistency
#define CUSTOM_PAN_ID               0x1234 
static const esp_zb_ieee_addr_t CUSTOM_EXTENDED_PAN_ID = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};


//LED: 1 = connected and on, 0 = disconnected and off
#define LED_GPIO_PIN GPIO_NUM_5

//Network state
static bool network_initally_opened = false; //One time flag for the keep_network_open() call on device reboot
static uint8_t network_time_open = 255; //In seconds
static uint16_t router_short_address = 0;    //Obtained in ANNCE

//RTT 
static uint64_t previous_send_time = 0;
 
//Tag
static const char *TAG = "ESP_ZB_COORDINATOR"; 


/*
    TASKS
*/
//Constant reopenning of network for developememnt and testing purpose only
void keep_network_live(void *arg) {
    while (1) {
        esp_zb_bdb_open_network(network_time_open);
        vTaskDelay(pdMS_TO_TICKS(network_time_open * 1000));
    }
}

//Read and print adc value every 3 seconds
void adc_read_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(SENSOR_READING_DELAY));

        int raw_data;
        if (adc_oneshot_read(adc_handle, PIEZO_ADC_CHANNEL, &raw_data) == ESP_OK) {
            sensor_value = (uint16_t) raw_data;
            //ESP_LOGI(TAG, "Sensor data: %d", raw_data);

            //Send the ZCL message
            if (router_short_address != 0) {
                esp_zb_zcl_basic_cmd_t zcl_cmd = {
                    .dst_addr_u.addr_short = router_short_address, // This was obtained when the joining device announced its info in the ANNCE signal
                    .dst_endpoint = ESP_ZB_ROUTER_ENDPOINT,
                    .src_endpoint = ESP_ZB_COORDINATOR_ENDPOINT,
                };

                esp_zb_zcl_custom_cluster_cmd_t vibration_req = {
                    .zcl_basic_cmd = zcl_cmd,
                    .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT, 
                    .cluster_id = CUSTOM_CLUSTER_ID,
                    .profile_id = ESP_ZB_AF_HA_PROFILE_ID, 
                    .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, 
                    .custom_cmd_id = SENSOR_DATA_COMMAND_REQ_ID,
                    .data.type =  ESP_ZB_ZCL_ATTR_TYPE_U16,
                    .data.value = &sensor_value,
                };
                
                //RTT tracking
                previous_send_time = esp_timer_get_time();

                //Send sensor data command (returns sequence number)
                esp_zb_zcl_custom_cluster_cmd_req(&vibration_req);
            }
        } else {
            ESP_LOGE("ADC", "Failed to read ADC");
        }
        
    }
}


/*
    CALLBACKS
*/
//Puts commission on the stack in a context its normally not possible
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}




/*
    HANDLERS
*/
static esp_err_t zb_custom_cmd_handler(const esp_zb_zcl_custom_cluster_command_message_t *message) {
    gpio_set_level(LED_GPIO_PIN, 1);


    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)", message->info.status);

    uint64_t rtt = (esp_timer_get_time() - previous_send_time) / 1000; // Convert from micro to milliseconds
    ESP_LOGI(TAG, "Sensor data acknowledged with an RTT of %llu", rtt);

    //Send the RTT I2C or Disconnect I2C
    if (DO_I2C_REPORTING) { //!Will be disabled if DO_I2C_REPORTING = false
        uint8_t buffer[9];

        buffer[0] = RTT_BYTE_IDENTIFIER;  // Identifier
        for (int i = 0; i < 8; i++) {
            buffer[i + 1] = (rtt >> (56 - i * 8)) & 0xFF;
        } 

        i2c_slave_write_buffer(I2C_SLAVE_NUM, buffer, sizeof(buffer), pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

//Handles all responses from cmds whether custom or default, must be registered before zigbee start
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret;
    ret = zb_custom_cmd_handler((esp_zb_zcl_custom_cluster_command_message_t *)message);
    return ret;
}

//Automatically called when signal is given
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network formation");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                if (!network_initally_opened) {
                    xTaskCreate(keep_network_live, "keep_network_live", 4096, NULL, 5, NULL);
                    network_initally_opened = true;
                }
                ESP_LOGI(TAG, "Device rebooted");

            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t ieee_address;
            esp_zb_get_long_address(ieee_address);
            ESP_LOGI(TAG, "Formed network successfully (IEEE Address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4],
                     ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);

        //Store device address to send zcl data or other requests
        router_short_address = dev_annce_params->device_short_addr;

        if (!created_sensor_task) {
            created_sensor_task = true;
            xTaskCreate(adc_read_task, "adc_read_task", 2048, NULL, 5, NULL);
        }

        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);

                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));

            } else {
                //ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        ESP_LOGI(TAG, "Production configuration is %s", err_status == ESP_OK ? "ready" : "not present");
        esp_zb_set_node_descriptor_manufacturer_code(ESP_MANUFACTURER_CODE);
        break;
    case ESP_ZB_NLME_STATUS_INDICATION:
        ESP_LOGE(TAG, "Connection severed from loss of a link"); //Likely due to a range issue or random interference 
        if (DO_I2C_REPORTING) { //!Will be disabled if DO_I2C_REPORTING = false
            uint8_t buffer[9];
            buffer[0] = DISCONNECT_BYTE_IDENTIFIER_NLME;  // Identifier
            for (int i = 0; i < 8; i++) {
                buffer[i + 1] = ((esp_timer_get_time() / 1000) >> (56 - i * 8)) & 0xFF;
            }
            i2c_slave_write_buffer(I2C_SLAVE_NUM, buffer, sizeof(buffer), pdMS_TO_TICKS(10));
        }

        //These errors seem to be blocking and prevent a reconnection, so full resetting device for a new network
        esp_zb_factory_reset();
        esp_restart();

        break;
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        ESP_LOGE(TAG, "Router no longer available"); //Router out of range or lost power
        if (DO_I2C_REPORTING) { //!Will be disabled if DO_I2C_REPORTING = false
            uint8_t buffer[9];
            buffer[0] = DISCONNECT_BYTE_IDENTIFIER_DEV_UNAVAILABLE;  // Identifier
            for (int i = 0; i < 8; i++) {
                buffer[i + 1] = ((esp_timer_get_time() / 1000) >> (56 - i * 8)) & 0xFF;
            }
            i2c_slave_write_buffer(I2C_SLAVE_NUM, buffer, sizeof(buffer), pdMS_TO_TICKS(10));
        }

        //These errors seem to be blocking and prevent a reconnection, so full resetting device for a new network
        esp_zb_factory_reset();
        esp_restart();

        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}


/*
    INITIALIZERS
*/
void init_i2c_slave()
{
    i2c_config_t conf = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .mode = I2C_MODE_SLAVE,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave = {
            .slave_addr = I2C_SLAVE_ADDR,
            .maximum_speed = 100000
        }
    };
    i2c_param_config(I2C_SLAVE_NUM, &conf);
    i2c_driver_install(I2C_SLAVE_NUM, I2C_MODE_SLAVE, I2C_BUF_LEN, I2C_BUF_LEN, 0);
}

void init_adc()
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = PIEZO_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,  // Up to ~3.3V range
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, PIEZO_ADC_CHANNEL, &config));
}

void init_led()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gpio_set_level(LED_GPIO_PIN, 0); 

}


//Create zigbee stack
static void esp_zb_task(void *pvParameters)
{
    //Network config
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG(); 
    esp_zb_init(&zb_nwk_cfg);  
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    //Set pre-defined network parameters
    esp_zb_set_pan_id(CUSTOM_PAN_ID);
    esp_zb_set_extended_pan_id(CUSTOM_EXTENDED_PAN_ID);

    //Create cluster and endpoint lists for zigbee stack
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    //Add mandatory basic and identify clusters 
    esp_zb_cluster_list_add_basic_cluster(cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    //Custom cluster creation for the vibration sensor **Client side (sends data)
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    
    //Define config then add and register the endpoints with this device
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = ESP_ZB_COORDINATOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, //HA is for home automation
        .app_device_id = 0x0107, //For a vibration/occupancy sensor
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
    esp_zb_device_register(ep_list);

    //Register event handler for cmd requests
    esp_zb_core_action_handler_register(zb_action_handler);

    //Start stack
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
    vTaskDelete(NULL);
}

void app_main(void) {
    //For debug
    esp_reset_reason_t reason = esp_reset_reason();
    ESP_LOGI(TAG, "Last reset reason: %d", reason);

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    ESP_ERROR_CHECK(esp_zb_gateway_console_init());
#endif
    init_adc(); 
    init_led(); 
    if (DO_I2C_REPORTING) { //!Will be disabled if DO_I2C_REPORTING = false
        init_i2c_slave();
        if (reason == 4) {
            //Watchdog timeout
            uint8_t buffer[9];
            buffer[0] = DISCONNECT_BYTE_IDENTIFIER_WATCHDOG;  // Identifier
            for (int i = 0; i < 8; i++) {
                buffer[i + 1] = ((esp_timer_get_time() / 1000) >> (56 - i * 8)) & 0xFF;
            }
            i2c_slave_write_buffer(I2C_SLAVE_NUM, buffer, sizeof(buffer), pdMS_TO_TICKS(10));
        } 
    }
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
