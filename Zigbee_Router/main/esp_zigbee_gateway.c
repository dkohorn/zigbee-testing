#include <fcntl.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_coexist.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_zigbee_gateway.h"
#include "zb_config_platform.h"
#include "driver/gpio.h"
#include "esp_timer.h"

//Cluster and Endpoints
#define ESP_ZB_ROUTER_ENDPOINT        0x02
#define ESP_ZB_COORDINATOR_ENDPOINT   0x01
#define CUSTOM_CLUSTER_ID           0xFF00

//LED: 1 = connected and on, 0 = disconnected and off
#define LED_GPIO_PIN GPIO_NUM_5

//Connection detection
static bool steering_in_progress = false; // ensure network steering isn't restarted while already searching

//Connection timeout
#define TIMEOUT_CYCLE_TIME 2000
static bool started_timeout_task = false; // A one-time flag to start the timeout 

//Steering retry and factory resetting
#define FACTORY_RESET_TIME 30000 
static TimerHandle_t factory_reset_timeout_timer = NULL;


//Tag
static const char *TAG = "ESP_ZB_RECEIVER";

/*
    CALLBACKS
*/
static void network_steering_retry_cb(uint8_t param) {
    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

//Happens on 30 seconds of no response from coordinator, triggers full factory reset to clear cache for new network joining
//Pretty much only needed if coordinator restarts for some reason
void timer_callback(TimerHandle_t xTimer) {
    gpio_set_level(LED_GPIO_PIN, 0);

    started_timeout_task = false;
    esp_zb_factory_reset();    
}

/*
    TASKS
*/
//Constant check to handle regular disconnects
static void network_timeout() {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TIMEOUT_CYCLE_TIME));

        if (esp_zb_bdb_dev_joined()) {
            steering_in_progress = false;
            gpio_set_level(LED_GPIO_PIN, 1);
        }
        else {
            gpio_set_level(LED_GPIO_PIN, 0);

            ESP_LOGI(TAG, "No longer in network, beginning steering...");
            if (!steering_in_progress) {
                steering_in_progress = true;
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
        }  
    }
}

/*
    HANDLERS
*/
//Handle the incoming sensor data from sender
static esp_err_t zb_custom_cmd_handler(const esp_zb_zcl_custom_cluster_command_message_t *message) { 
    xTimerReset(factory_reset_timeout_timer, 0);
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)", message->info.status);

    uint16_t sensor_value = *(uint16_t *) message->data.value;
    ESP_LOGI(TAG, "Sensor data received: %d", sensor_value);

    return ESP_OK;
}

//Default handler registered
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID:
        ret = zb_custom_cmd_handler((esp_zb_zcl_custom_cluster_command_message_t *)message);
        break;
    default:
        //ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

//Default signal handler
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        ESP_LOGI(TAG, "Network is currently joinable");
        break;
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network found");

            if (!started_timeout_task) {
                started_timeout_task = true;
                xTaskCreate(network_timeout, "network_timeout", 4096, NULL, 5, NULL); //Handle basic disconnects
                factory_reset_timeout_timer = xTimerCreate("factory_reset_timeout_timer", pdMS_TO_TICKS(FACTORY_RESET_TIME), pdFALSE, NULL, timer_callback); //Handle a possible network restart
                xTimerStart(factory_reset_timeout_timer, 10);
            }

        } else {
            ESP_LOGE(TAG, "Network steering failed, error: %s, retrying...", esp_err_to_name(err_status));

            esp_zb_scheduler_alarm(network_steering_retry_cb, 0, 5000);   //Retry every 5 seconds
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
        if (err_status != ESP_OK) {
            ESP_LOGW(TAG, "Production config not found — continuing without it.");
        } else {
            ESP_LOGI(TAG, "Production config loaded successfully.");
        }
        break;
    //Checks for connection severed
    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        ESP_LOGE(TAG, "Hardware or unknown failure"); //Likely caused by a power outage
        break;
    case ESP_ZB_NLME_STATUS_INDICATION:
        ESP_LOGE(TAG, "Connection severed from loss of a link"); //Likely due to a range issue or random interference
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}



/*
    INITIALIZERS
*/
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

static void esp_zb_task(void *pvParameters)
{
    //Network config
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG(); 
    esp_zb_init(&zb_nwk_cfg);  
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    //Create cluster and endpoint lists for zigbee stack
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    //Add mandatory basic and identify clusters 
    esp_zb_cluster_list_add_basic_cluster(cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    //Custom cluster creation for the vibration sensor **Server side (receives data)
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);
    esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    //Define config then add and register the endpoints with this device
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = ESP_ZB_ROUTER_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, //HA is for home automation
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID, 
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
    init_led();
    
    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
