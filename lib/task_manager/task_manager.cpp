#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_manager.h"
#include "task_handler.h"

using namespace std;


void terminate_every_task(){
    //la memoria usasta dalle altre task è automaticamente liberata dalla API di FreeRTOS
    if (tcp_server_task_handle != nullptr) {
        vTaskDelete(tcp_server_task_handle);
        tcp_server_task_handle = nullptr;
    }
    if (task_receive_uart_master_handle != nullptr) {
        vTaskDelete(task_receive_uart_master_handle);
        task_receive_uart_master_handle = nullptr;
    }
    if (task_receive_uart_slave_handle != nullptr) {
        vTaskDelete(task_receive_uart_slave_handle);
        task_receive_uart_slave_handle = nullptr;
    }
    if (task_send_uart_master_handle != nullptr) {
        vTaskDelete(task_send_uart_master_handle);
        task_send_uart_master_handle = nullptr;
    }
    if (task_send_uart_slave_handle != nullptr) {
        vTaskDelete(task_send_uart_slave_handle);
        task_send_uart_slave_handle = nullptr;
    }
    if (task_ping_slave_handle != nullptr) {
        vTaskDelete(task_ping_slave_handle);
        task_ping_slave_handle = nullptr;
    }
    if (task_ping_master_handle != nullptr) {
        vTaskDelete(task_ping_master_handle);
        task_ping_master_handle = nullptr;
    }
    if (task_handle_handshakes_handle != nullptr) {
        vTaskDelete(task_handle_handshakes_handle);
        task_handle_handshakes_handle = nullptr;
    }
    if (task_handle_report_handle != nullptr) {
        vTaskDelete(task_handle_report_handle);
        task_handle_report_handle = nullptr;
    }
    if (task_loop_print_ids_array_handle != nullptr) {
        vTaskDelete(task_loop_print_ids_array_handle);
        task_loop_print_ids_array_handle = nullptr;
    }
    if (move_servo_speed_task_handle != nullptr) {
        vTaskDelete(move_servo_speed_task_handle);
        move_servo_speed_task_handle = nullptr;
    }
    if (buffer_task_handle != nullptr) {
        vTaskDelete(buffer_task_handle);
        buffer_task_handle = nullptr;
    }

}