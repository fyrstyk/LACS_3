#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_sntp.h"

#include "hardware_config.h"
#include "motor_control.h"
#include "sun_tracking.h"
#include "storage.h"
#include "ota.h"
#include "web_server.h"

static const char *TAG = "LACS2";

// Global resources
i2c_master_bus_handle_t i2c_bus = NULL;
i2c_master_dev_handle_t mcp_handle = NULL;
SemaphoreHandle_t hardware_mutex = NULL;
static esp_timer_handle_t heartbeat_timer = NULL;

// MCP23017 write function (needed by multiple modules)
esp_err_t mcp23017_write(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(mcp_handle, data, 2, 1000);
}

static void heartbeat_timer_callback(void *arg) {
    static bool state = false;
    state = !state;
    
    if (state) {
        mcp_porta |= 0x80;    // Set A7 high
    } else {
        mcp_porta &= ~0x80;   // Set A7 low
    }
    
    mcp23017_write(MCP23017_OLATA, mcp_porta);
}

static void ethernet_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_config);
    
    // Set static IP
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 1, 80);
    IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    
    esp_netif_dhcpc_stop(eth_netif);
    esp_netif_set_ip_info(eth_netif, &ip_info);
    
    // Configure SPI for W5500
    spi_bus_config_t spi_bus_config = {
        .mosi_io_num = W5500_MOSI_PIN,
        .miso_io_num = W5500_MISO_PIN,
        .sclk_io_num = W5500_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO));
    
    // Reset W5500
    gpio_set_direction(W5500_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(W5500_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(W5500_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Configure W5500
    spi_device_interface_config_t spi_devcfg = {
        .mode = 0,
        .clock_speed_hz = 8000000,
        .spics_io_num = W5500_CS_PIN,
        .queue_size = 20,
    };
    
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(SPI2_HOST, &spi_devcfg);
    w5500_config.int_gpio_num = W5500_INT_PIN;
    
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    mac_config.rx_task_stack_size = 4096;
    
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
    
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    
    ESP_LOGI(TAG, "Ethernet initialized - IP: 192.168.1.80");
}

void app_main(void) {
    // Initialize storage
    storage_init();
    
    // Create mutex
    hardware_mutex = xSemaphoreCreateMutex();
    
    // Initialize I2C
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));
    
    // Initialize MCP23017
    i2c_device_config_t mcp_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MCP23017_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &mcp_cfg, &mcp_handle));
    
    // Configure MCP23017 ports
    mcp23017_write(MCP23017_IODIRA, 0x68);  // A3, A5, A6 as inputs
    mcp23017_write(MCP23017_IODIRB, 0x60);  // B5, B6 as inputs
    mcp23017_write(MCP23017_OLATA, 0x00);   // All outputs OFF
    mcp23017_write(MCP23017_OLATB, 0x00);   // All outputs OFF
    
    // Initialize motor control
    motor_control_init();
    
    // Load saved data
    load_calibration();
    load_home_bearing();
    load_location();
    calculate_sun_position(&sun_az, &sun_el);
    
    // Start 25Hz heartbeat
    const esp_timer_create_args_t heartbeat_args = {
        .callback = &heartbeat_timer_callback,
        .name = "heartbeat"
    };
    ESP_ERROR_CHECK(esp_timer_create(&heartbeat_args, &heartbeat_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(heartbeat_timer, 20000)); // 20ms = 50Hz toggle = 25Hz signal
    ESP_LOGI(TAG, "25Hz heartbeat started on MCP A7");
    
    // Initialize Ethernet
    gpio_install_isr_service(0);
    ethernet_init();
    
    // Initialize and start web server
    web_server_init();
    
    // Create tasks
    if (xTaskCreate(position_control_task, "position", 4096, NULL, 5, NULL) == pdPASS) {
        ESP_LOGI(TAG, "Position control task started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start position control task");
    }
    
    if (xTaskCreate(auto_tracking_task, "tracking", 4096, NULL, 3, NULL) == pdPASS) {
        ESP_LOGI(TAG, "Auto tracking task started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start auto tracking task");
    }
    
    ESP_LOGI(TAG, "LACS2 v2.0 initialized - Fixed PCNT direction handling");
    ESP_LOGI(TAG, "Web interface: http://192.168.1.80");
}