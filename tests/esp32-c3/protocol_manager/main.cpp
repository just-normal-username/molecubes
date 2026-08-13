#include "servo_controller.h"
#include "utils_uart_comms.h"
#include "servo_tests.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "init_wifi.h"
#include "protocol_manager.h"
struct {
    uint8_t mac[6];
} molecube_data;

void init_cube() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI("CUBE_INIT", "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    memcpy(molecube_data.mac, mac, 6); //copying the mac address byte to byte to the molecube_data struct
}
extern "C" void app_main() {
    ESP_LOGI("TEST", "Starting servo tests...");
    init_uart_comms();
    init_cube();
    servo_init();
    init_wifi();
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

}