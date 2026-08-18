#pragma GCC optimize ("Os")
#include "servo_controller.h"
#include "utils_uart_comms.h"
#include "init_wifi.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_log.h"
#include "protocol_manager.h"
#include "buffer_headers/buffer_header.h"


struct {
    uint8_t mac[6];
} molecube_data;

void init_cube() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI("CUBE_INIT", "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    memcpy(molecube_data.mac, mac, 6); //copying the mac address byte to byte to the molecube_data struct
}

//inizializza la logica del wifi, uart e del buffer dei comandi
//essenziale perchè il bridge wifi-uart carica i comandi in una coda che fa da buffer
// in modo che poi la task che invia i comandi al servo li esegua uno alla volta
// se in init_cmd_buffer() non si riesce a creare la coda viene lanciata un eccezione che blocca l'esecuzione
esp_err_t init_cmd_logic(){
    init_wifi();
    init_uart_comms();
    esp_err_t ris = init_cmd_buffer(); //todo gestire tutti i casi di errore terminando ogni task?
    return ris;
}

void handle_movement_ack(Msg* msg){
    //TODO complete this function, send the ack to the ui?
}

// Task: receive Msg* from the higher-level UART queue and translate into
// servo controller commands by calling move_servo_speed()
// this is needed beacause also the root has a servo

void task_execute_servo(void *arg) {
    (void)arg;
    extern QueueHandle_t h_queue_servo; // declared in utils_uart_comms.h / GLOBAL_VARS.cpp

    while (1) {
        Msg *msg = nullptr;
        if (xQueueReceive(h_queue_servo, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI("EXEC_SERVO", "Received servo message, speed=%.3f, acc=%.3f, jerk=%.3f", msg->payload.payload_servo.speed, msg->payload.payload_servo.acceleration, msg->payload.payload_servo.jerk);
            if (msg) {
                float radians = msg->payload.payload_servo.radians;
                float speed = msg->payload.payload_servo.speed;
                float acc = msg->payload.payload_servo.acceleration;
                float jerk = msg->payload.payload_servo.jerk;
                esp_err_t err = move_servo_speed(radians, speed, acc, jerk);
                if (err != ESP_OK) {
                    ESP_LOGW("EXEC_SERVO", "move_servo_speed failed: %d", err);
                }
                delete msg; // free message allocated by UART layer
            }
        }
    }
}

extern "C" void app_main() {
    // Print reset reason early to determine if the board was reset or app_main returned
    esp_reset_reason_t reason = esp_reset_reason();
    const char* reason_str;
    switch(reason){
        case ESP_RST_UNKNOWN: reason_str = "UNKNOWN"; break;
        case ESP_RST_POWERON: reason_str = "POWERON"; break;
        case ESP_RST_EXT: reason_str = "EXTERNAL_RESET"; break;
        case ESP_RST_SW: reason_str = "SOFTWARE_RESET"; break;
        case ESP_RST_PANIC: reason_str = "PANIC"; break;
        case ESP_RST_INT_WDT: reason_str = "INT_WDT"; break;
        case ESP_RST_TASK_WDT: reason_str = "TASK_WDT"; break;
        case ESP_RST_WDT: reason_str = "WDT"; break;
        case ESP_RST_DEEPSLEEP: reason_str = "DEEPSLEEP"; break;
        case ESP_RST_BROWNOUT: reason_str = "BROWNOUT"; break;
        case ESP_RST_SDIO: reason_str = "SDIO"; break;
        default: reason_str = "OTHER"; break;
    }
    ESP_LOGI("BOOT", "Reset reason: %d (%s)", reason, reason_str);

    //initializing wifi, uart comms, cube data (mac address) and servo controller
    if (init_cmd_logic() != ESP_OK) {
        ESP_LOGE("TEST", "Failed to initialize command logic. Halting execution.");
        return; // Exit if initialization fails
    }
    init_cube();
    servo_init();

    // create and start the task that listens for servo messages coming from
    // the UART/protocol layer and forwards movement commands to the
    // servo controller (move_servo_speed)
    xTaskCreate(
        task_execute_servo,
        "ExecServoTask",
        3072,
        NULL,
        2,
        NULL
    );

}

