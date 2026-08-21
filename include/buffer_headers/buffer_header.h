#include <freertos/queue.h>
#include <atomic>

using namespace std;
// coda per il buffer dei comandi
extern QueueHandle_t h_queue_cmd_buffer;

extern std::atomic<int> ack_to_receive; //indice che indica quanti ack sono ancora da ricevere

extern TaskHandle_t buffer_task_handle; // Handle for the buffer task

extern std::atomic<bool> manual_pause; // Flag che indica lo start/stop manuale

#if defined(TEST_PROTOCOL_MANAGER)
    extern std::atomic<bool> status; // Flag che indica lo stato della task. true = la task processa i comandi, false = la task non processa i comandi
#endif

esp_err_t init_cmd_buffer();