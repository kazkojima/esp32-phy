/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_now.h"
#include "esp_eth.h"
#include "esp_partition.h"

#include "tcpip_adapter.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#if CONFIG_ENABLE_WIFI_AP
#define ADP_ADDR CONFIG_ADAPTOR_ADDRESS
#endif

#define UDP_SERVER CONFIG_UDP_SERVER_ADDRESS
#define UDP_PORT CONFIG_UDP_PORT

#ifdef CONFIG_PHY_LAN8720
#include "eth_phy/phy_lan8720.h"
#define DEFAULT_ETHERNET_PHY_CONFIG phy_lan8720_default_ethernet_config
#endif
#ifdef CONFIG_PHY_TLK110
#include "eth_phy/phy_tlk110.h"
#define DEFAULT_ETHERNET_PHY_CONFIG phy_tlk110_default_ethernet_config
#endif

#define PIN_PHY_POWER CONFIG_PHY_POWER_PIN
#define PIN_SMI_MDC   CONFIG_PHY_SMI_MDC_PIN
#define PIN_SMI_MDIO  CONFIG_PHY_SMI_MDIO_PIN

static const char *TAG = "sbus_adapter";

static bool client_connected = false;

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
      ESP_LOGI(TAG, "WiFi started");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
       ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d\n",
                MAC2STR(event->event_info.sta_disconnected.mac),
                event->event_info.sta_disconnected.aid);
       client_connected = false;
       break;
     // TODO: case SYSTEM_EVENT_ETH_DISCONNECETED:
    default:
      break;
    }

    return ESP_OK;
}

#ifdef CONFIG_PHY_USE_POWER_PIN
/* This replaces the default PHY power on/off function with one that
   also uses a GPIO for power on/off.

   If this GPIO is not connected on your device (and PHY is always powered), you can use the default PHY-specific power
   on/off function rather than overriding with this one.
*/
static void phy_device_power_enable_via_gpio(bool enable)
{
    assert(DEFAULT_ETHERNET_PHY_CONFIG.phy_power_enable);

    if (!enable) {
        /* Do the PHY-specific power_enable(false) function before powering down */
        DEFAULT_ETHERNET_PHY_CONFIG.phy_power_enable(false);
    }

    if(enable == true) {
        gpio_set_level(PIN_PHY_POWER, 1);
        ESP_LOGD(TAG, "phy_device_power_enable(TRUE)");
    } else {
        gpio_set_level(PIN_PHY_POWER, 0);
        ESP_LOGD(TAG, "power_enable(FALSE)");
    }

    // Allow the power up/down to take effect, min 300us
    vTaskDelay(1);

    if (enable) {
        /* Run the PHY-specific power on operations now the PHY has power */
        DEFAULT_ETHERNET_PHY_CONFIG.phy_power_enable(true);
    }
}
#endif

static void eth_gpio_config_rmii(void)
{
    // RMII data pins are fixed:
    // TXD0 = GPIO19
    // TXD1 = GPIO22
    // TX_EN = GPIO21
    // RXD0 = GPIO25
    // RXD1 = GPIO26
    // CLK == GPIO0
    phy_rmii_configure_data_interface_pins();
    // MDC is GPIO 23, MDIO is GPIO 18
    phy_rmii_smi_configure_pins(PIN_SMI_MDC, PIN_SMI_MDIO);
}

static bool dhcp_done = false;
static in_addr_t eth_addr;
static int sockfd = -1;

void eth_task(void *pvParameter)
{
    tcpip_adapter_ip_info_t ip;
    memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1) {

        vTaskDelay(2000 / portTICK_PERIOD_MS);

        if (tcpip_adapter_get_ip_info(ESP_IF_ETH, &ip))
            continue;

        if (!ip4_addr_isany_val(ip.ip)) {
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            ESP_LOGI(TAG, "ETHIP:"IPSTR, IP2STR(&ip.ip));
            ESP_LOGI(TAG, "ETHPMASK:"IPSTR, IP2STR(&ip.netmask));
            ESP_LOGI(TAG, "ETHPGW:"IPSTR, IP2STR(&ip.gw));
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            eth_addr = ip.ip.addr;
            dhcp_done = true;
            break;
        }
    }
    
    struct sockaddr_in saddr;
    struct sockaddr_in caddr;
    int rtn;

    printf("UDP client task starting...\n");

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s < 0) {
        printf("... Failed to allocate socket.\n");
        goto fail;
    }

    memset((char *) &caddr, 0, sizeof(caddr));
    caddr.sin_family = AF_INET;
    caddr.sin_addr.s_addr =  eth_addr;
    caddr.sin_port = htons(UDP_PORT);

    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr (UDP_SERVER);
    saddr.sin_port = htons(UDP_PORT);

    rtn = bind (s, (struct sockaddr *)&caddr, sizeof(caddr));
    if(rtn < 0) {
        printf("... Failed to bind socket.\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        goto fail;
    }

    rtn = connect(s, (struct sockaddr *) &saddr, sizeof(saddr));
    if (rtn < 0) {
        printf("... Failed to connect socket.\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        goto fail;
    }

    sockfd = s;

 fail:
    while(1) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

#if CONFIG_ENABLE_WIFI_AP
static int wsockfd = -1;
static struct sockaddr_in cli_addr;

void wifi_task(void *pvParameter)
{
    while(sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        printf("... Failed to create wifi socket.\n");
        goto fail;
    }

    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(int));

    struct sockaddr_in saddr;
    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(UDP_PORT);
    saddr.sin_addr.s_addr = inet_addr(ADP_ADDR);
    if (bind(fd, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
        int err;
        u32_t optlen = sizeof(int);
        getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &optlen);
        ESP_LOGW(TAG, "socket error %d %s", err, strerror(err));
        printf("... Failed to bind wifi socket.\n");
        close(fd);
        goto fail;
    }

    wsockfd = fd;

 fail:
    while(1) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
#endif

struct __attribute__((packed)) rcpkt {
    uint32_t version;
    uint64_t timestamp_us;
    uint16_t sequence;
    uint16_t pwms[8];
};

xQueueHandle sbus_queue = NULL;

static void send_task(void *arg)
{
    while(sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    uint16_t rcpkt_count = 0;

    ESP_LOGI(TAG, "Starting loop to sending udp packets");
    while(1) {
        uint16_t pwm[8];
        if (xQueueReceive(sbus_queue, (uint8_t *)pwm, portMAX_DELAY) == pdTRUE) {
            if (CONFIG_COPY_CH6_CH7) {
                uint16_t tmp = pwm[6];
                pwm[6] = pwm[5];
                pwm[5] = tmp;
            }

            struct rcpkt pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.version = 2;
            pkt.sequence = rcpkt_count++;
            pkt.timestamp_us = ((uint64_t)1000) * xTaskGetTickCount();
            for(int i=0; i<8; i++)
                pkt.pwms[i] = pwm[i];

            int n = send(sockfd, &pkt, sizeof(pkt), 0);
            //printf("%d byte sent\n", n);
            if (n < 0) {
                printf("Failed to send udp packet\n");
            }
        }
    }
}

void sbus_task(void *);

void app_main(void)
{
    esp_err_t ret = ESP_OK;

    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
#if CONFIG_ENABLE_WIFI_AP
    tcpip_adapter_ip_info_t info = { 0, };
    ip4addr_aton(ADP_ADDR, &info.ip);
    ip4addr_aton(ADP_ADDR, &info.gw);
    IP4_ADDR(&info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));
    ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CONFIG_SSID,
            .ssid_len = 0,
            .max_connection = 1,
            .password = CONFIG_SSID_PASSWORD,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
#endif

    eth_config_t config = DEFAULT_ETHERNET_PHY_CONFIG;
    /* Set the PHY address in the example configuration */
    config.phy_addr = CONFIG_PHY_ADDRESS;
    config.gpio_config = eth_gpio_config_rmii;
    config.tcpip_input = tcpip_adapter_eth_input;

#ifdef CONFIG_PHY_USE_POWER_PIN
    gpio_pad_select_gpio(PIN_PHY_POWER);
    gpio_set_direction(PIN_PHY_POWER,GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PHY_POWER, 0);
    ESP_LOGD(TAG, "power_enable(FALSE)");
    
    /* Replace the default 'power enable' function with an example-specific
       one that toggles a power GPIO. */
    config.phy_power_enable = phy_device_power_enable_via_gpio;
#endif
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ret = esp_eth_init(&config);

    if(ret == ESP_OK) {
        esp_eth_enable();
        xTaskCreate(eth_task, "eth_task", 2048, NULL, 5, NULL);
    }

    sbus_queue = xQueueCreate(4, 8*sizeof(uint16_t));

#if CONFIG_ENABLE_WIFI_AP
    xTaskCreate(wifi_task, "wifi_task", 2048, NULL, 5, NULL);
#endif

    xTaskCreate(sbus_task, "sbus_task", 2048, NULL, 6, NULL);
    xTaskCreate(send_task, "send_task", 2048, NULL, 7, NULL);

    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_2, level);
        level = !level;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

