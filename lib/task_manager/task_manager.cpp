#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_manager.h"
#include "task_handler.h"

using namespace std;

//codice wifi
TaskHandle_t tcp_server_task_handle= NULL;

//codice uart
TaskHandle_t task_receive_uart_master_handle= NULL;
TaskHandle_t task_receive_uart_slave_handle= NULL;
TaskHandle_t task_send_uart_master_handle= NULL;
TaskHandle_t task_send_uart_slave_handle= NULL;
TaskHandle_t task_ping_slave_handle= NULL;
TaskHandle_t task_ping_master_handle= NULL;
TaskHandle_t task_handle_handshakes_handle= NULL;
TaskHandle_t task_handle_report_handle= NULL;
TaskHandle_t task_loop_print_ids_array_handle= NULL;

//servo
TaskHandle_t move_servo_speed_task_handle= NULL;

//buffer
TaskHandle_t buffer_task_handle;


void terminate_every_task(){
    //la memoria usasta dalle altre task è automaticamente liberata dalla API di FreeRTOS
    if (tcp_server_task_handle != NULL) {
        vTaskDelete(tcp_server_task_handle);
        tcp_server_task_handle = NULL;
    }
    if (task_receive_uart_master_handle != NULL) {
        vTaskDelete(task_receive_uart_master_handle);
        task_receive_uart_master_handle = NULL;
    }
    if (task_receive_uart_slave_handle != NULL) {
        vTaskDelete(task_receive_uart_slave_handle);
        task_receive_uart_slave_handle = NULL;
    }
    if (task_send_uart_master_handle != NULL) {
        vTaskDelete(task_send_uart_master_handle);
        task_send_uart_master_handle = NULL;
    }
    if (task_send_uart_slave_handle != NULL) {
        vTaskDelete(task_send_uart_slave_handle);
        task_send_uart_slave_handle = NULL;
    }
    if (task_ping_slave_handle != NULL) {
        vTaskDelete(task_ping_slave_handle);
        task_ping_slave_handle = NULL;
    }
    if (task_ping_master_handle != NULL) {
        vTaskDelete(task_ping_master_handle);
        task_ping_master_handle = NULL;
    }
    if (task_handle_handshakes_handle != NULL) {
        vTaskDelete(task_handle_handshakes_handle);
        task_handle_handshakes_handle = NULL;
    }
    if (task_handle_report_handle != NULL) {
        vTaskDelete(task_handle_report_handle);
        task_handle_report_handle = NULL;
    }
    if (task_loop_print_ids_array_handle != NULL) {
        vTaskDelete(task_loop_print_ids_array_handle);
        task_loop_print_ids_array_handle = NULL;
    }
    if (move_servo_speed_task_handle != NULL) {
        vTaskDelete(move_servo_speed_task_handle);
        move_servo_speed_task_handle = NULL;
    }
    if (buffer_task_handle != NULL) {
        vTaskDelete(buffer_task_handle);
        buffer_task_handle = NULL;
    }

}