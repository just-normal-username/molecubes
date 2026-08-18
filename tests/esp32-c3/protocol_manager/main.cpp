#include "servo_controller.h"
#include "utils_uart_comms.h"
#include "servo_tests.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "init_wifi.h"
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
extern "C" void app_main() {
    ESP_LOGI("TEST", "Starting servo tests...");
    if (init_cmd_logic() != ESP_OK) {
        ESP_LOGE("TEST", "Failed to initialize command logic. Halting execution.");
        return; // Exit if initialization fails
    }
    init_cube();
    servo_init();
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for Wi-Fi to initialize
    ProtocolManager::handle_incoming("G6 N0 P90.0 S1.0 A2.0 J3.0");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("G6 N0 P90.0 S1.0 A2.0 J3.0 N1 P45.0 J1.5");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("M222 2.5");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("M204 3.5");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("M205 4.5");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("G4 1000");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("M24");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ProtocolManager::handle_incoming("M25");
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI("TEST", "debug log");
    if (xTaskNotify(buffer_task_handle, 0x1, eSetValueWithOverwrite) != pdPASS) {
        ESP_LOGE("TEST", "Failed to notify buffer task.");
    }
    else{
        ESP_LOGI("TEST", "Buffer task notified successfully.");
    }

}