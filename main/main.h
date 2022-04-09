#ifndef _MAIN
#define _MAIN

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/semphr.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

#include "esp_netif.h"
#include "esp_now.h"
#include "esp_crc.h"

#define EX_UART_NUM UART_NUM_0
#define UART_SPEED 115200
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#define PACK_LEN 256
#define PACK_NUM 8

#define CONFIG_ESPNOW_PMK "pmk1234567890ylj"
#define CONFIG_ESPNOW_LMK "lmk1234567890ylj"
#define CONFIG_ESPNOW_CHANNEL 13
#define ESPNOW_WIFI_MODE WIFI_MODE_STA

#define espnow_send_max_cnt 3

enum
{
    ESPNOW_SEND_OK=0,
    ESPNOW_SEND_ERR,
    ESPNOW_RECV_OK,
    ESPNOW_RECV_ERR
};

enum
{
    UART_ESPNOW_IDLE=0,
    UART_ESPNOW_SEND
};

#endif
