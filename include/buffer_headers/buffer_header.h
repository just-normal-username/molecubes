#include <freertos/queue.h>
#include <atomic>

using namespace std;
// coda per il buffer dei comandi
extern QueueHandle_t h_queue_cmd_buffer;

extern QueueHandle_t h_queue_start_processing_cmd_buffer;

extern std::atomic<int> ack_to_receive; //indice che indica quanti ack sono ancora da ricevere