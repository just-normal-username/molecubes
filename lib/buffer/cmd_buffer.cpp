#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "msg_structs.h"
#include <esp_log.h>
#include <stdexcept>
#include <atomic>
#include <utils_uart_comms.h>
#include "buffer_headers/buffer_header.h"

using namespace std;

QueueHandle_t h_queue_cmd_buffer;
QueueHandle_t h_queue_start_processing_cmd_buffer;
TaskHandle_t buffer_task_handle = NULL;
std::atomic<int> ack_to_receive; //indice che indica quanti ack sono ancora da ricevere

//flag che indica lo stato della task. atomic per sicurezza e scalabilità
// true = la task processa i comandi, false = la task non processa i comandi
std::atomic<bool> status = false; 


void buffer_task(void *pvParameters) {
    bool value;
    int group_number=1;
    while (1) {
        if (status.load()==false){
            //se in stato di stop attende la ricezione del messaggio di start, anche in caso di timeout esegue il ciclo e ritorna qui
            if (ulTaskNotifyTake(h_queue_start_processing_cmd_buffer, &value ,portMAX_DELAY) == pdTRUE) {
                if (value) {
                    status.store(true);
                    ESP_LOGI("CMD_BUFFER", "Buffer task started processing commands.");
                }
            }
        }
        else{
            // se in stato di esecuzione prima di eseguire ogni comando controlla se è arrivato un comando di stop
            if (xQueueReceive(h_queue_start_processing_cmd_buffer, &value, 0) == pdTRUE) {
                if (!value) {
                    status.store(false);
                    ESP_LOGI("CMD_BUFFER", "Buffer task stopped processing commands.");
                }
            }
            if (status.load()==true){
                //se in stato di esecuzione esegue il prossimo comando
                //todo implementare G4, group e ack
                Msg* msg;
                //processa i nuovi comandi solo se sono raggruppati o se non ci sono ack da ricevere
                if (group_number>0||ack_to_receive.load()==0){
                    group_number=group_number>0? group_number-1:0;
                    if (group_number==0){
                        //stoppa la task se il gruppo di comandi da processare è terminato
                        //la task verrà svegliata quando verranno ricevuti i messaggi di ack
                        xQueueSend(h_queue_start_processing_cmd_buffer, (void *)false, 0);
                    }
                    if (xQueueReceive(h_queue_cmd_buffer, &msg, 0) == pdTRUE) {
                        // Process the received command
                        ESP_LOGI("CMD_BUFFER", "Processing command");
                        if (msg->type == type_group) {
                            group_number=msg->payload.payload_group.group_number;
                        }
                        else if (msg->type == type_servo) {
                            // gestione comando per il servo
                            //aumentando il numero di ack da ricevere,
                            ack_to_receive.fetch_add(1);
                            if (msg->target_id == SELF_ID) {
                                // It's for the Root: send to the local servo queue
                                sort_new_msg(msg);
                            } else {
                                // It's for a Slave: route it through UART
                                send_msg_to_slave(msg);
                            }
                        }
                        else if (msg->type == type_g4) {
                            // blocca l'esecuzione per un certo numero
                            // vtaskdelay usa il numero di tick
                            vTaskDelay(msg->payload.payload_g4.millis / portTICK_PERIOD_MS);
                        }
                    }
                }
            }
        }
        //todo
    }
}

esp_err_t init_cmd_buffer() {
    // la dimensione è 2 per sicurezza
    h_queue_cmd_buffer = xQueueCreate(100, sizeof(Msg*)); //! crea una coda con spazio per massimo 200 puntatori a Msg
    ack_to_receive.store(0); // inizializza il contatore degli ack da ricevere a 0
    xTaskCreate(buffer_task, "buffer_task", 4096, NULL, 5, &buffer_task_handle);
    if (h_queue_cmd_buffer == NULL) {
        // Handle error: Queue creation failed
        ESP_LOGE("CMD_BUFFER", "Failed to create command buffer queue");
        return ESP_FAIL;
    }
    return ESP_OK;
}