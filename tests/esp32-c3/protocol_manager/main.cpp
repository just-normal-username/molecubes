#include "servo_controller.h"
#include "utils_uart_comms.h"
#include "servo_tests.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "init_wifi.h"
#include "protocol_manager.h"
#include "buffer_headers/buffer_header.h"


void init_cube() {
    uint8_t mac_appo[6];
    esp_read_mac(mac_appo, ESP_MAC_WIFI_STA);
    ESP_LOGI("CUBE_INIT", "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac_appo[0], mac_appo[1], mac_appo[2], mac_appo[3], mac_appo[4], mac_appo[5]);
    memcpy(mac, mac_appo, 6); //copying the mac address byte to byte to the molecube_data struct
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

// Task: receive Msg* from the higher-level UART queue and translate into
// servo controller commands by calling move_servo_speed()
// this is needed beacause also the root has a servo

// void task_execute_servo(void *arg) {
//     (void)arg;
//     extern QueueHandle_t h_queue_servo; // declared in utils_uart_comms.h / GLOBAL_VARS.cpp

//     while (1) {
//         Msg *msg = nullptr;
//         if (xQueueReceive(h_queue_servo, &msg, portMAX_DELAY) == pdTRUE) {
//             ESP_LOGI("EXEC_SERVO", "Received servo message, speed=%.3f, acc=%.3f, jerk=%.3f", msg->payload.payload_servo.speed, msg->payload.payload_servo.acceleration, msg->payload.payload_servo.jerk);
//             if (msg) {
//                 float radians = msg->payload.payload_servo.radians;
//                 float speed = msg->payload.payload_servo.speed;
//                 float acc = msg->payload.payload_servo.acceleration;
//                 float jerk = msg->payload.payload_servo.jerk;
//                 esp_err_t err = move_servo_speed(radians, speed, acc, jerk, msg->payload.payload_servo.relative);
//                 if (err != ESP_OK) {
//                     ESP_LOGW("EXEC_SERVO", "move_servo_speed failed: %d", err);
//                 }
//                 delete msg; // free message allocated by UART layer
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(10)); // piccola attesa per evitare che venga triggerata la WDT
//     }
// }


extern "C" void app_main() {
    const esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 30000, // Set the timeout to 10 seconds
        .idle_core_mask = (1 << 0), // Monitor all cores
        .trigger_panic = false // non panica in timeout
    };
    esp_task_wdt_init(&wdt_config);
    ESP_LOGI("TEST", "Starting servo tests...");
    init_cube();
    if (init_cmd_logic() != ESP_OK) {
        ESP_LOGE("TEST", "Failed to initialize command logic. Halting execution.");
        return; // Exit if initialization fails
    }
    servo_init();
    // create and start the task that listens for servo messages coming from
    // the UART/protocol layer and forwards movement commands to the
    // servo controller (move_servo_speed)
    // xTaskCreate(
    //     task_execute_servo,
    //     "ExecServoTask",
    //     3072,
    //     NULL,
    //     2,
    //     NULL
    // );
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for Wi-Fi to initialize
    ProtocolManager::handle_incoming("G6 N0 P90.0 S1.0 A2.0 J3.0 N1 P45.0 J1.5 N2 P45.0 J1.5 N3 P45.0 J1.5");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("G4 5000");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("G6 N0 P0 S1.0 A2.0 J3.0");
    vTaskDelay(pdMS_TO_TICKS(2000));
    // ProtocolManager::handle_incoming("G6 N0 P90.0 S1.0 A2.0 J3.0 N1 P45.0 J1.5");
    // vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("M222 2.5");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("M204 3.5");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("M205 4.5");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("M24");
    vTaskDelay(pdMS_TO_TICKS(2000));
    // ProtocolManager::handle_incoming("M25");
    // vTaskDelay(pdMS_TO_TICKS(2000));
    // for (int i = 0; i < 98; i++) {
    //     ProtocolManager::handle_incoming("G6 N0 P0 S1.0 A2.0 J3.0");
    //     vTaskDelay(pdMS_TO_TICKS(10));
    // }
    // ProtocolManager::handle_incoming("G6 N0 P90 S1.0 A2.0 J3.0");
    // ProtocolManager::handle_incoming("M24");
    // ESP_LOGI("TEST", "debug log");
    // if (xTaskNotify(buffer_task_handle, 0x1, eSetValueWithOverwrite) != pdPASS) {
    //     ESP_LOGE("TEST", "Failed to notify buffer task.");
    // }
    // else{
    //     ESP_LOGI("TEST", "Buffer task notified successfully.");
    // }

}