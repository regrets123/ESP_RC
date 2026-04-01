#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "car";

// L293D motor driver pins
#define IN1_PIN     GPIO_NUM_12
#define IN2_PIN     GPIO_NUM_14
#define ENABLE1_PIN GPIO_NUM_13

// LEDC PWM
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT  // 0–1023
#define LEDC_FREQ_HZ    5000

// Remote controller MAC address
static const uint8_t remote_mac[ESP_NOW_ETH_ALEN] = {0xDC, 0xB4, 0xD9, 0x9A, 0xCA, 0xF8};

typedef struct {
    int8_t  throttle;   // -100 (reverse) to 100 (forward)
    int8_t  steering;   // -100 (left)    to 100 (right)
    uint8_t button;     // 1 = pressed
} __attribute__((packed)) remote_packet_t;

// ── Motor control ─────────────────────────────────────────────────────────────

static void motor_set(int8_t throttle)
{
    uint32_t duty = (uint32_t)(abs(throttle) * 1023 / 100);

    if (throttle > 0) {
        gpio_set_level(IN1_PIN, 1);
        gpio_set_level(IN2_PIN, 0);
    } else if (throttle < 0) {
        gpio_set_level(IN1_PIN, 0);
        gpio_set_level(IN2_PIN, 1);
    } else {
        gpio_set_level(IN1_PIN, 0);
        gpio_set_level(IN2_PIN, 0);
    }

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void motor_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num   = ENABLE1_PIN,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
}

// ── ESP-NOW ───────────────────────────────────────────────────────────────────

static void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
{
    if (data_len != sizeof(remote_packet_t)) {
        ESP_LOGW(TAG, "Unexpected packet size: %d", data_len);
        return;
    }

    remote_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));

    ESP_LOGI(TAG, "throttle=%d  steering=%d  button=%d", pkt.throttle, pkt.steering, pkt.button);

    motor_set(pkt.throttle);
    // TODO: add steering control once hardware is defined
}

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    esp_now_peer_info_t peer = {
        .channel = 0,
        .ifidx   = ESP_IF_WIFI_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, remote_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ── Entry point ───────────────────────────────────────────────────────────────

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motor_init();
    wifi_init();
    espnow_init();

    ESP_LOGI(TAG, "Car ready — waiting for remote");
}
