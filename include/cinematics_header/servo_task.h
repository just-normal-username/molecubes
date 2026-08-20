#ifndef SERVO_TASK_H
#define SERVO_TASK_H

#include "servo_types.h"
#include "msg_structs.h"

// forward declarations
void move_servo_speed_task_state_machine(void * pvParameters);
ServoTaskParams sanitize_servo_command(Msg* msg);
void servo_init();
void send_movement_ack();

#endif
