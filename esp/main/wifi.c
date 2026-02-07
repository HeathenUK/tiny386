/*
 * WiFi backend for NE2000 emulation on Tanmatsu (ESP32-P4)
 *
 * Uses tanmatsu-wifi component to connect via launcher-saved credentials.
 * Bridges NE2000 emulated Ethernet to real WiFi through ESP-HOSTED SDIO
 * transport to the onboard ESP32-C6 radio.
 *
 * TX path: guest → NE2000 registers → ne2000.c qemu_send_packet()
 *        → esp32_send_packet atomic fn ptr → esp_wifi_internal_tx()
 *        → ESP-HOSTED SDIO → C6 → WiFi
 *
 * RX path: WiFi → C6 → SDIO → ESP-HOSTED → wlanif.c
 *        → wlanif_l2_input_hook() [weak override in ne2000.c]
 *        → ne2000_receive() → ring buffer → guest IRQ
 */

#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "wifi_remote.h"
#include "wifi_connection.h"

/* Atomic TX function pointer — shared with ne2000.c via extern */
void (*_Atomic esp32_send_packet)(uint8_t *buf, int size) = NULL;

static bool wifi_stack_ready = false;

static void send_packet(uint8_t *buf, int size)
{
	esp_wifi_internal_tx(ESP_IF_WIFI_STA, buf, size);
}

static void event_handler(void *arg, esp_event_base_t base,
			  int32_t id, void *data)
{
	if (base == WIFI_EVENT) {
		if (id == WIFI_EVENT_STA_CONNECTED) {
			wifi_event_sta_connected_t *con =
				(wifi_event_sta_connected_t *)data;
			fprintf(stderr, "WiFi: connected to %.32s (ch %d)\n",
				con->ssid, con->channel);
			atomic_store(&esp32_send_packet, send_packet);
		} else if (id == WIFI_EVENT_STA_DISCONNECTED) {
			wifi_event_sta_disconnected_t *dis =
				(wifi_event_sta_disconnected_t *)data;
			fprintf(stderr, "WiFi: disconnected from %.32s (reason %d)\n",
				dis->ssid, dis->reason);
			atomic_store(&esp32_send_packet, NULL);
		}
	} else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
		ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
		fprintf(stderr, "WiFi: got IP " IPSTR "\n",
			IP2STR(&event->ip_info.ip));
	}
}

static inline bool wifi_is_connected(void)
{
	return atomic_load(&esp32_send_packet) != NULL;
}

/*
 * Init ESP-HOSTED SDIO transport and WiFi stack.
 * Must be called from app_main BEFORE storage_init(),
 * because ESP-HOSTED uses SDMMC slot 1 and must init
 * the shared SDMMC host first.
 */
void wifi_init(void)
{
	if (wifi_remote_initialize() != ESP_OK) {
		fprintf(stderr, "WiFi: failed to initialize radio\n");
		return;
	}

	if (wifi_connection_init_stack() != ESP_OK) {
		fprintf(stderr, "WiFi: failed to initialize stack\n");
		return;
	}

	esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
				   event_handler, NULL);
	esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
				   event_handler, NULL);
	wifi_stack_ready = true;
	fprintf(stderr, "WiFi: stack ready\n");
}

/*
 * Connect to saved networks and maintain connection.
 * Runs in a background task — never returns.
 *
 * wifi_connect_try_all() may report failure even when a connection
 * succeeds (e.g., duplicate connect requests). We use the actual
 * STA_CONNECTED event (via esp32_send_packet) as the source of truth.
 */
void wifi_connect(void)
{
	if (!wifi_stack_ready)
		return;

	while (1) {
		if (!wifi_is_connected()) {
			wifi_connect_try_all();
			/* Give the event handler time to process */
			vTaskDelay(pdMS_TO_TICKS(2000));
		}

		if (wifi_is_connected()) {
			/* Monitor connection — poll until we lose it */
			while (wifi_is_connected())
				vTaskDelay(pdMS_TO_TICKS(5000));
			fprintf(stderr, "WiFi: connection lost, reconnecting...\n");
		}

		/* Not connected — wait before retrying */
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
}
