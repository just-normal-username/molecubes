#pragma GCC optimize ("Os")
#include "servo_controller.h"
#include "utils_uart_comms.h"
#include "init_wifi.h"
#include "esp_mac.h"
#include "esp_log.h"
#include <buffer_headers/buffer_header.h>

void init_cube();
void task_execute_servo(void *arg);

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
    //init_wifi();
    init_uart_comms();
    esp_err_t ris = init_cmd_buffer(); //todo gestire tutti i casi di errore terminando ogni task?
    return ris;
}


// Task: receive Msg* from the higher-level UART queue and translate into
// servo controller commands by calling move_servo_speed()
void task_execute_servo(void *arg) {
    (void)arg;
    extern QueueHandle_t h_queue_servo; // declared in utils_uart_comms.h / GLOBAL_VARS.cpp

    while (1) {
        Msg *msg = nullptr;
        if (xQueueReceive(h_queue_servo, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI("EXEC_SERVO", "Received servo message, speed=%.3f, acc=%.3f, jerk=%.3f", msg->payload.payload_servo.speed, msg->payload.payload_servo.acceleration, msg->payload.payload_servo.jerk);
            if (msg) { //todo differenziare per i vari tipi di messaggi
                if (msg->type == type_servo){
                    float radians = msg->payload.payload_servo.radians;
                    float speed = msg->payload.payload_servo.speed;
                    float acc = msg->payload.payload_servo.acceleration;
                    float jerk = msg->payload.payload_servo.jerk;
                    bool relative = msg->payload.payload_servo.relative;
                    esp_err_t err = move_servo_speed(radians, speed, acc, jerk, relative);
                    if (err != ESP_OK) {
                        ESP_LOGW("EXEC_SERVO", "move_servo_speed failed: %d", err);
                    }
                    delete msg; // free message allocated by UART layer
                }
                
            }
        }
    }
}

extern "C" void app_main() {
    //initializing wifi, uart comms, cube data (mac address) and servo controller
    //init_wifi();
    init_cmd_logic();
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

