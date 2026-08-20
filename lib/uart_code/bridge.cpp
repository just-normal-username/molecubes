#include "utils_uart_comms.h"
#include "esp_log.h"
#include "protocol_manager.h"
#include <cmath>
#include <buffer_headers/buffer_header.h>
using namespace std;



//*BRIDGE WIFI

// Helper to convert degrees to radians if your servo logic requires it
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
// todo questi valori verranno modificati dai comandi M222 M204 e M205
float default_speed = 1.0f;
float default_acc = 2.0f;
float default_jerk = 5.0f;

esp_err_t create_and_buffer_msg(int sender_id, int target_id, Payload& p){
    // se la posizione del servo non è valida e quindi target_id è -1 viene ignorato il messaggio
    if (target_id != -1){
        Msg* msg = create_msg(sender_id, target_id, type_servo, p);
        if (h_queue_cmd_buffer!=NULL){
            if ( xQueueSend(h_queue_cmd_buffer, &msg, 0) != pdTRUE) { // aggiunge il nuovo comando al buffer, se è pieno ritorna subito
                ESP_LOGW("SERVO_API", "impossibile aggiungere il comando al buffer, coda piena");
                return ESP_FAIL; // questa eccezione verrà catturata in init_wifi.cpp
            }
        }
        else{
            ESP_LOGE("SERVO_API", "h_queue_cmd_buffer è NULL");
            return ESP_FAIL;
        }
    }
    return ESP_OK;

}



esp_err_t convert_servo_instructions(const Command& command){
    // qua ci sono solo comandi validi, quindi non serve fare controlli di validità
    int total_nodes = get_ids_array_len();
    int ids_arr[total_nodes];
    get_ids_array(ids_arr, total_nodes);

    // printf("total_nodes %d\n", total_nodes);
    // for(int i=0; i<3; i++){
    //     printf("%d\n", ids_arr[i]);
    // }

    Payload p{};
    p.payload_servo.speed = default_speed;
    p.payload_servo.acceleration = default_acc;
    p.payload_servo.jerk = default_jerk;
    int group_number=0;
    esp_err_t result = ESP_OK;
    switch(command.gcode){
        case Gcode::G6:{
            // Handle G6 command specifics
            int target_id=0;
            bool relative=false; // todo da implementare nel payload
            // Value-initialize the payload to avoid leaking uninitialized stack bytes
            if (command.args[0]==R){
                relative=true;
            }
            for (size_t i =0; i<command.args.size(); i++){
                if (command.args[i]==N){
                    //ignora il comando se la posizione del modulo non è valida
                    if (command.values[i]>=0 && command.values[i]<total_nodes){
                        group_number+=1;
                    }
                }
            }

            // creazione e buffering del comando group
            if (group_number>1){
                Payload group_payload{};
                group_payload.payload_group.group_number = group_number;
                Msg* msg = create_msg(SELF_ID, SELF_ID, type_group, group_payload);
                if (h_queue_cmd_buffer!=NULL){
                    if ( xQueueSend(h_queue_cmd_buffer, &msg, 0) != pdTRUE) { // aggiunge il nuovo comando al buffer, se è pieno ritorna subito
                        ESP_LOGW("SERVO_API", "impossibile aggiungere il comando al buffer, coda piena");
                        return ESP_FAIL; // questa eccezione verrà catturata in init_wifi.cpp
                    }
                }
                else{
                    ESP_LOGE("SERVO_API", "h_queue_cmd_buffer è NULL");
                    return ESP_FAIL;
                }
            }

            for (size_t i = 0; i < command.args.size(); i++) { // salterà R se c'è 
                ESP_LOGI(
                    "SERVO_API",
                    "Processing arg %zu: %d with value %.3f",
                    i,
                    command.args[i],
                    command.values[i]
                );
                if (command.args[i] == Args::P) {
                    p.payload_servo.radians = command.values[i] * (M_PI / 180.0f); // Convert degrees to radians
                } else if (command.args[i] == Args::S) {
                    // Handle S argument specifics
                    p.payload_servo.speed = command.values[i];
                } else if (command.args[i] == Args::A) {
                    // Handle A argument specifics
                    p.payload_servo.acceleration = command.values[i];
                } else if (command.args[i] == Args::J) {
                    // Handle J argument specifics
                    p.payload_servo.jerk = command.values[i];
                } else if (command.args[i] == Args::N) {
                    // setta il flag per il comando di posizione relativo
                    p.payload_servo.relative=relative;
                    if (i<2){
                        //dopo N è riportata la posizione del servo, che viene usata per identificare l'ID del servo
                        if (command.values[i]<0 || command.values[i]>=total_nodes){
                            target_id=-1; //posizione non valida
                        }
                        else{
                            target_id = ids_arr[static_cast<int>(round(command.values[i]))];
                        } 
                    }
                    else{
                        ESP_LOGI(
                            "SERVO_API",
                            "total_nodes= %d, target_id=%d, angle=%.2f deg, radians=%.4f, speed=%.3f, acc=%.3f, jerk=%.3f",
                            total_nodes,
                            target_id,
                            p.payload_servo.radians * (180.0f / M_PI), // Convert radians back to degrees for logging
                            p.payload_servo.radians,
                            p.payload_servo.speed,
                            p.payload_servo.acceleration,
                            p.payload_servo.jerk
                        );
                        if (target_id == SELF_ID) {
                            // It's for the Root: send to the local servo queue
                            if (create_and_buffer_msg(SELF_ID, SELF_ID, p)!= ESP_OK){
                                return ESP_FAIL;
                            }
                            //sort_new_msg(msg);

                        } else {
                            // It's for a Slave: route it through UART
                            if (create_and_buffer_msg(SELF_ID, target_id, p)!= ESP_OK){
                                return ESP_FAIL;
                            }
                            //send_msg_to_slave(msg);
                        }
                        p = {}; // Reset payload for next command
                        p.payload_servo.speed = default_speed;
                        p.payload_servo.acceleration = default_acc;
                        p.payload_servo.jerk = default_jerk;
                        if (command.values[i]<0 || command.values[i]>=total_nodes){
                            target_id=-1; //posizione non valida
                        }
                        else{
                            target_id = ids_arr[static_cast<int>(round(command.values[i]))];
                        } 
                    }
                }
            }
            
            // setta il flag per il comando di posizione relativo
            p.payload_servo.relative=relative;
            if (target_id == SELF_ID) {
                // It's for the Root: send to the local servo queue
                if (create_and_buffer_msg(SELF_ID, SELF_ID, p)!= ESP_OK){
                    return ESP_FAIL;
                }
                //sort_new_msg(msg);
            } else {
                // It's for a Slave: route it through UART
                if (create_and_buffer_msg(SELF_ID, target_id, p)!= ESP_OK){
                    return ESP_FAIL;
                }
                //send_msg_to_slave(msg);
            }
            ESP_LOGI(
                "SERVO_API",
                "target_id=%d, angle=%.2f deg, radians=%.4f, speed=%.3f, acc=%.3f, jerk=%.3f",
                target_id,
                p.payload_servo.radians * (180.0f / M_PI), // Convert radians back to degrees for logging
                p.payload_servo.radians,
                p.payload_servo.speed,
                p.payload_servo.acceleration,
                p.payload_servo.jerk
            );
            break;
        }
        case Gcode::M222:{
            // Handle M222 command
            default_speed = command.values[0];
            ESP_LOGI("SERVO_API", "Updated default speed to %.3f", default_speed);
            break;
        }
        case Gcode::M204:{
            // Handle M204 command
            default_acc = command.values[0];
            ESP_LOGI("SERVO_API", "Updated default acceleration to %.3f", default_acc);
            break;
        }
        case Gcode::M205:{
            // Handle M205 command
            default_jerk = command.values[0];
            ESP_LOGI("SERVO_API", "Updated default jerk to %.3f", default_jerk);
            break;
        }
        case Gcode::G4:{
            // Handle G4 command
            ESP_LOGI("SERVO_API", "G4 command received");
            p.payload_g4.millis = static_cast<uint32_t>(command.values[0]);
            // It's for the Root: send to the local servo queue
            Msg* msg = create_msg(SELF_ID, SELF_ID, type_g4, p);
            if (h_queue_cmd_buffer!=NULL){
                if ( xQueueSend(h_queue_cmd_buffer, &msg, 0) != pdTRUE) { // aggiunge il nuovo comando al buffer, se è pieno ritorna subito
                    ESP_LOGW("SERVO_API", "impossibile aggiungere il comando al buffer, coda piena");
                    return ESP_FAIL; // questa eccezione verrà catturata in init_wifi.cpp
                }
            }
            else{
                ESP_LOGE("SERVO_API", "h_queue_cmd_buffer è NULL");
                return ESP_FAIL;
            }
            break;
        }
        case Gcode::M24:{
            // Handle M24 command
            manual_pause.store(false); // setta la pausa manuale a falso
            if (xTaskNotify(buffer_task_handle, 0x1, eSetValueWithOverwrite) != pdPASS) {
                ESP_LOGE("SERVO_API", "Failed to notify buffer task.");
            }
            else{
                ESP_LOGI("SERVO_API", "Buffer task notified successfully.");
            }
            ESP_LOGI("SERVO_API", "M24 command received, but not implemented yet.");
            break;
        }
        case Gcode::M25:{
            // Handle M25 command
            manual_pause.store(true); // setta la pausa manuale a vero
            if (xTaskNotify(buffer_task_handle, 0x2, eSetValueWithOverwrite) != pdPASS) {
                ESP_LOGE("SERVO_API", "Failed to notify buffer task.");
            }
            else{
                ESP_LOGI("SERVO_API", "Buffer task notified successfully.");
            }
            ESP_LOGI("SERVO_API", "M25 command received, but not implemented yet.");
            break;
        }
        
    }



    // The vector 'angles' comes from the computer. 
    // We assume angles[0] is for Root (ID 0), angles[1] for first Slave, etc.
    // for (size_t i = 0; i < command.args.size(); i++) {
    //     //if (i >= (size_t)total_nodes) break; // Safety check

        

    //     // Value-initialize the payload to avoid leaking uninitialized stack bytes
    //     Payload p{};
    //     // Convert degree (uint16_t) to Radians (float) as expected by your Payload struct
    //     p.payload_servo.radians = angles[i] * (M_PI / 180.0f); //! ATTENTO ALLA CONVERSIONE IN RADIANTI, LA VUOI VERAMENTE???
    //     // Provide safe defaults for motion parameters if the sender doesn't set them
    //     p.payload_servo.speed = velocities[i];           // default normalized speed (1.0 = full)
    //     p.payload_servo.acceleration =  accelerations[i];  // reasonable default
    //     p.payload_servo.jerk =  jerks[i];         // reasonable default
    //     int target_id = ids_arr[i];
    //     ESP_LOGI(
    //         "SERVO_API",
    //         "target_id=%d, angle=%.2f deg, radians=%.4f, speed=%.3f, acc=%.3f, jerk=%.3f",
    //         target_id,
    //         angles[i],
    //         p.payload_servo.radians,
    //         p.payload_servo.speed,
    //         p.payload_servo.acceleration,
    //         p.payload_servo.jerk
    //     );

    //     if (target_id == SELF_ID) {
    //         // It's for the Root: send to the local servo queue
    //         Msg* msg = create_msg(SELF_ID, SELF_ID, type_servo, p);
    //         sort_new_msg(msg);
    //     } else {
    //         // It's for a Slave: route it through UART
    //         Msg* msg = create_msg(SELF_ID, target_id, type_servo, p);
    //         send_msg_to_slave(msg);
    //     }
    // }
    return ESP_OK;
}


//*BRIDGE ???
void send_servo_movement_ack_to_root(int my_id, float radians){ //todo viene chiamata?
    // Ensure payload is zero-initialized to avoid garbage bytes
    Payload p{};
    p.payload_servo.radians = radians;
    Msg* msg = create_msg(my_id, ROOT_ID, type_servo_ack, p);
    send_msg_to_master(msg);
}
