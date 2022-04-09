#include "main.h"

static const char *TAG = "uart_events";
static QueueHandle_t uart0_queue;
static QueueHandle_t espnow_queue;

//target mac,not self mac
// static uint8_t espnow_send_mac[ESP_NOW_ETH_ALEN] = { 0x48,0x3f,0xda,0x48,0xba,0xa3 };//1
static uint8_t espnow_send_mac[ESP_NOW_ETH_ALEN] = { 0xf4,0xcf,0xa2,0x71,0x2a,0x20 };//2
// static uint8_t espnow_send_mac[ESP_NOW_ETH_ALEN] = { 0xe8,0x68,0xe7,0xd2,0x0e,0x93 };//T0-0
// static uint8_t espnow_send_mac[ESP_NOW_ETH_ALEN] = { 0xe8,0x68,0xe7,0xd1,0xcb,0xbd };//T0-1

static uint8_t uart_espnow_status=UART_ESPNOW_IDLE;
static uint8_t espnow_send_cnt=0;
static uint8_t data_pack[PACK_NUM][PACK_LEN];
static uint8_t espnow_recv_data[256];
static uint32_t uart_rx_index=0,uart_rx_counti=0,uart_rx_counto=0;
static uint8_t espnow_tx_index=0;
static uint8_t espnow_dataout_cnt=0,espnow_datain_cnt=0xFF;

static void backup_send_data(uint8_t *data,uint8_t len)
{
    data_pack[uart_rx_index][0]=0xaa;
    data_pack[uart_rx_index][1]=espnow_dataout_cnt++;//CNT
    data_pack[uart_rx_index][2]=len;
    memcpy(data_pack[uart_rx_index]+3,data,len);

    data_pack[uart_rx_index][len+3]=0;//checksum
    for(uint8_t i=0;i<len+3;i++)
        data_pack[uart_rx_index][len+3]+=data_pack[uart_rx_index][i];

    if(uart_rx_index==PACK_NUM-1) uart_rx_index=0;
    else uart_rx_index++;
    uart_rx_counti++;

    if(uart_rx_counti>uart_rx_counto+PACK_NUM){//buf full
        uart_rx_index=0;
        uart_rx_counti=0;
        uart_rx_counto=0;
        espnow_tx_index=0;
    }
}

static void espnow_wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    uint8_t state=0;
    if(mac_addr==NULL)
        state=ESPNOW_SEND_ERR;
    else{
        if(memcmp(espnow_send_mac,mac_addr,ESP_NOW_ETH_ALEN)==0&&status==0)
            state=ESPNOW_SEND_OK;
        else
            state=ESPNOW_SEND_ERR;
    }

    if (xQueueSend(espnow_queue, &state, pdMS_TO_TICKS(500)) != pdTRUE) 
    {
        ESP_LOGW(TAG, "espnow send send_queue fail\n");
    }
}

static void espnow_recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    uint8_t state=0;
    if (mac_addr == NULL || data == NULL || len <= 0 || len>sizeof(espnow_recv_data)) 
    {
        return;
    }

    if( memcmp(espnow_send_mac,mac_addr,ESP_NOW_ETH_ALEN)==0&&len==data[2]+4)
    {
        state=ESPNOW_RECV_OK;
        memcpy(espnow_recv_data,data,len);
    }
    else
    {
        return;
    }

    if (xQueueSend(espnow_queue, &state, pdMS_TO_TICKS(500)) != pdTRUE) 
    {
        ESP_LOGW(TAG, "espnow send recv_queue fail\n");
    }
}

static esp_err_t espnow_init()
{
    espnow_queue = xQueueCreate(6,1);
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Create mutex fail");
        return ESP_FAIL;
    }

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(espnow_recv_cb) );

    /* Set primary master key. */
    ESP_ERROR_CHECK( esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK) );

    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
        vSemaphoreDelete(espnow_queue);
        esp_now_deinit();
        return ESP_FAIL;
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESP_IF_WIFI_STA;
    peer->encrypt = true;
    memcpy(peer->peer_addr, espnow_send_mac, ESP_NOW_ETH_ALEN);
    memcpy(peer->lmk,CONFIG_ESPNOW_LMK,sizeof(CONFIG_ESPNOW_LMK));
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    free(peer);

    return ESP_OK;
}

static void uart_espnow_task(void *pvParameters)
{
    uint8_t espnow_state;
    for(;;){
        if(xQueueReceive(espnow_queue, (void *)&espnow_state, 1)){
            switch(espnow_state){
                case(ESPNOW_SEND_OK):
                {
                    if(espnow_tx_index==PACK_NUM-1) espnow_tx_index=0;
                    else espnow_tx_index++;
                    uart_rx_counto++;

                    espnow_send_cnt=0;
                    uart_espnow_status=UART_ESPNOW_IDLE;

                    gpio_set_level(2,1);
                    break;
                }
                case(ESPNOW_SEND_ERR):
                {
                    if(espnow_send_cnt>espnow_send_max_cnt-1){
                        if(espnow_tx_index==PACK_NUM-1) espnow_tx_index=0;
                        else espnow_tx_index++;
                        uart_rx_counto++;

                        espnow_send_cnt=0;
                        uart_espnow_status=UART_ESPNOW_IDLE;

                        gpio_set_level(2,1);
                    }
                    else{
                        // uart_write_bytes(EX_UART_NUM, (const char *) "espnow send err\r\n", sizeof("espnow send err\r\n"));
                        espnow_send_cnt++;
                        if (esp_now_send(espnow_send_mac, data_pack[espnow_tx_index], data_pack[espnow_tx_index][2]+4) != ESP_OK) 
                        {
                            ESP_LOGW(TAG, "espnow send err\n");
                        }
                    }
                    break;
                }
                case(ESPNOW_RECV_OK):
                {
                    // uart_write_bytes(EX_UART_NUM, (const char *) "espnow recv\r\n", sizeof("espnow recv\r\n"));
                    if(espnow_recv_data[0]==0xaa && espnow_recv_data[1]!=espnow_datain_cnt){
                        espnow_datain_cnt=espnow_recv_data[1];
                        uart_write_bytes(EX_UART_NUM, (const char *) espnow_recv_data+3, espnow_recv_data[2]);
                    }
                    break;
                }
                case(ESPNOW_RECV_ERR):
                {
                    break;
                }
                default:
                {
                    break;
                }
            }
        }

        if(uart_espnow_status==UART_ESPNOW_IDLE) {
            if(uart_rx_counto!=uart_rx_counti){
                // uart_write_bytes(EX_UART_NUM, (const char *) data_pack[espnow_tx_index], data_pack[espnow_tx_index][2]+4);

                if (esp_now_send(espnow_send_mac, data_pack[espnow_tx_index], data_pack[espnow_tx_index][2]+4) != ESP_OK) 
                {
                    ESP_LOGW(TAG, "espnow send err\n");
                }
                uart_espnow_status=UART_ESPNOW_SEND;
                gpio_set_level(2,0);
            }
        }
    }

    vTaskDelete(NULL);
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *dtmp = (uint8_t *) malloc(RD_BUF_SIZE);

    for (;;) {
        // Waiting for UART event.
        if (xQueueReceive(uart0_queue, (void *)&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);

            switch (event.type) {
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    // char temp[16];
                    // sprintf(temp,"%d\n",event.size);
                    // uart_write_bytes(EX_UART_NUM, (const char *) temp, 16);

                    backup_send_data(dtmp,event.size);
                    // uart_write_bytes(EX_UART_NUM, (const char *) dtmp, event.size);
                    break;

                // Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                // Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;

                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;

                // Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;

                // Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }

    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main()
{
    uart_config_t uart_config = {
        .baud_rate = UART_SPEED,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(EX_UART_NUM, &uart_config);
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 100, &uart0_queue, 0);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    espnow_wifi_init();
    esp_wifi_set_ps(WIFI_PS_NONE);
    espnow_init();

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1U<<2;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    gpio_set_level(2,1);

    uint8_t stamac[6];
    ESP_ERROR_CHECK(esp_read_mac(stamac,ESP_MAC_WIFI_STA));
    char temp[64],len;
    len=sprintf(temp,"\r\nsta:"MACSTR"\r\n",MAC2STR(stamac));
    uart_write_bytes(EX_UART_NUM, (const char *) temp, len);

    // Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    xTaskCreate(uart_espnow_task, "uart_espnow_task", 2048, NULL, 11, NULL);
}