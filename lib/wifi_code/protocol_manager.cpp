#include "protocol_manager.h"
#include "tcp_server.h"

#include <sstream>
#include <string>
#include "esp_log.h"

static const char* TAG = "ProtocolManager";

static constexpr uint8_t  MAX_SERVOS   = 5;
static constexpr float ANGLE_MIN    = -139.0;
static constexpr float ANGLE_MAX    = +139.0;
static constexpr float MIN_SPEED    =  0.1;
static constexpr float MAX_SPEED = 5.2;
static constexpr float MIN_ACC = 0.1;
static constexpr float MAX_ACC = 100.0;
static constexpr float MIN_JERK = 000.1;
static constexpr float MAX_JERK = 1500.0;


uint8_t s_num_servos = 1;

namespace {
    static ServoCommandCallback  s_on_command;
}

// ---------------------------------------------------------------------------
// Internal: send a message to the computer via TCP
// ---------------------------------------------------------------------------
void reply(const std::string& msg)
{
    TcpServer::send(msg);
    ESP_LOGI(TAG, "-> Computer: %s", msg.c_str());
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void ProtocolManager::init(uint8_t num_servos, ServoCommandCallback on_servo_command)
{
    s_on_command = on_servo_command;
    s_num_servos = num_servos;
    // Non inviamo nulla qui — il computer non è ancora connesso.
    // set_num_servos() verrà chiamato dalla on_connect del TcpServer.
}

void ProtocolManager::set_num_servos(uint8_t num_servos) //!HERE
{
    if (num_servos!=s_num_servos){
        s_num_servos = num_servos;

        // Notify the computer immediately about the updated number of connected peripherals
        reply("SERVOS " + std::to_string(s_num_servos));
        ESP_LOGI(TAG, "Periferiche collegate: %d", s_num_servos);
    }
}

#include <cstdlib> 

void ProtocolManager::handle_incoming(const std::string& line)
{
    ESP_LOGI(TAG, "<- Computer: %s", line.c_str());

    std::vector<float> angles;
    std::vector<float> velocities;
    std::vector<float> accelerations;
    std::vector<float> jerks;

    Command command;
    command.args= std::vector<Args>();
    command.values= std::vector<float>();

    // Use a stream to parse the space-separated values received from the computer
    std::istringstream stream(line);
    std::string token;
    
    int token_count = 0; // Contatore dei valori letti
    bool command_started = false; // Flag per indicare se il comando è iniziato
    Gcode current_gcode; // Gcode corrente
    std::string previous_arg; // Argomento precedente per il controllo della sequenza
    int valuei;
    float valuef;
    while (stream >> token) { //todo considerare newline
        if (!command_started) {
            if (token == "G6"){
                command.gcode = Gcode::G6;
                current_gcode = Gcode::G6;
                command_started = true;
                previous_arg = "G6";
            }
            else if (token == "M222"){
                command.gcode = Gcode::M222;
                current_gcode = Gcode::M222;
                command_started = true;
                previous_arg = "M222";
            }
            else if (token == "M204"){
                command.gcode = Gcode::M204;
                current_gcode = Gcode::M204;
                command_started = true;
                previous_arg = "M204";
            }
            else if (token == "M205"){
                command.gcode = Gcode::M205;
                current_gcode = Gcode::M205;
                command_started = true;
                previous_arg = "M205";
            }
            else if (token == "G4"){
                command.gcode = Gcode::G4;
                current_gcode = Gcode::G4;
                command_started = true;
                previous_arg = "G4";
            }
            else if (token == "M24"){
                command.gcode = Gcode::M24;
                current_gcode = Gcode::M24;
                command_started = true;
                previous_arg = "M24";
            }
            else if (token == "M25"){
                command.gcode = Gcode::M25;
                current_gcode = Gcode::M25;
                command_started = true;
                previous_arg = "M25";
            }
            else {
                reply("ERROR unknown_command — expected G6, M222, M204, or M205");
                return;
            }
        }
        else{
            //parser per comando G6
            if (current_gcode == Gcode::G6){
                if (previous_arg=="G6"&&token[0]== 'R'){ // R deve per forza essere dopo G6
                    command.args.push_back(Args::R);
                    command.values.push_back(0.0f); // Placeholder value for R
                    previous_arg = "R";
                }
                else if (token[0]== 'N'&&(previous_arg=="G6"|| previous_arg=="R" 
                    ||previous_arg=="P" || previous_arg=="S" || previous_arg=="V" 
                    || previous_arg=="J")){ //N può trovarsi dopo G6, R, P, S, A o J
                    std::string value=token.substr(1);
                    char* endptr = nullptr;
                    // Parse strings to floats and validate the physical limits of the motors
                    float value2 = std::strtof(value.c_str(), &endptr); //todo strtoi?
                    
                    // Check number format (exception-free)
                    if (endptr == value.c_str() || *endptr != '\0') {
                        reply("ERROR invalid_format — expected numbers separated by spaces");
                        return;
                    }
                    command.values.push_back(value2);
                    command.args.push_back(Args::N);
                    previous_arg = "N";

                }
                else if (previous_arg == "N"){ // dopo N deve esserci per forza P
                    if (token[0] == 'P'){
                        std::string value=token.substr(1);
                        char* endptr = nullptr;
                        // Parse strings to floats and validate the physical limits of the motors
                        float value2 = std::strtof(value.c_str(), &endptr);
                        
                        // Check number format (exception-free)
                        if (endptr == value.c_str() || *endptr != '\0') {
                            reply("ERROR invalid_format — expected numbers separated by spaces");
                            return;
                        }
                        command.values.push_back(value2);
                        command.args.push_back(Args::P);
                        
                        previous_arg = "P";
                    }
                    else{
                        reply("ERROR invalid_argument — expected P after N for G6 command");
                        return;
                    }
                    
                }
                else if (previous_arg == "P"&&token[0] == 'S'){ //S deve trovarsi per forza dopo P
                    std::string value=token.substr(1);
                    char* endptr = nullptr;
                    // Parse strings to floats and validate the physical limits of the motors
                    float value2 = std::strtof(value.c_str(), &endptr);
                    
                    // Check number format (exception-free)
                    if (endptr == value.c_str() || *endptr != '\0') {
                        reply("ERROR invalid_format — expected numbers separated by spaces");
                        return;
                    }
                    command.values.push_back(value2);
                    command.args.push_back(Args::S);
                    
                    previous_arg = "S";
                }
                else if ((previous_arg == "P"||previous_arg=="S")&&token[0] == 'A'){ //A può trovarsi dopo P o S
                    std::string value=token.substr(1);
                    char* endptr = nullptr;
                    // Parse strings to floats and validate the physical limits of the motors
                    float value2 = std::strtof(value.c_str(), &endptr);
                    
                    // Check number format (exception-free)
                    if (endptr == value.c_str() || *endptr != '\0') {
                        reply("ERROR invalid_format — expected numbers separated by spaces");
                        return;
                    }
                    command.values.push_back(value2);
                    command.args.push_back(Args::A);
                    
                    previous_arg = "A";
                }
                else if ((previous_arg == "P"||previous_arg=="S"||previous_arg=="A")&&token[0] == 'J'){ //J può trovarsi dopo P, S o A
                    std::string value=token.substr(1);
                    char* endptr = nullptr;
                    // Parse strings to floats and validate the physical limits of the motors
                    float value2 = std::strtof(value.c_str(), &endptr);
                    
                    // Check number format (exception-free)
                    if (endptr == value.c_str() || *endptr != '\0') {
                        reply("ERROR invalid_format — expected numbers separated by spaces");
                        return;
                    }
                    command.values.push_back(value2);
                    command.args.push_back(Args::J);
                    
                    previous_arg = "J";
                }
                else{ // in questo caso l'ordine di argomenti è errato
                    reply("ERROR invalid_format — unexpected argument for G6 command");
                    return;
                }
            }
            else if (
                (current_gcode== Gcode::M222 && previous_arg == "M222")||
                (current_gcode== Gcode::M204 && previous_arg == "M204")||
                (current_gcode== Gcode::M205 && previous_arg == "M205")||
                (current_gcode== Gcode::G4 && previous_arg == "G4")
            ){ //parser per comando M222
                char* endptr = nullptr;
                // Parse strings to floats
                float value = std::strtof(token.c_str(), &endptr);
                
                // Check number format (exception-free)
                if (endptr == token.c_str() || *endptr != '\0') {
                    reply("ERROR invalid_format — expected numbers separated by spaces");
                    return;
                }
                command.values.push_back(value);
                command.args.push_back(Args::N); // argument placeholder
                previous_arg = "N";
            }
            else if (
                (current_gcode== Gcode::M24 && previous_arg == "M24")||
                (current_gcode== Gcode::M25 && previous_arg == "M25")){
                // placeholder
            }
            else{ // in questo caso gli argomenti sono errati
                reply("ERROR invalid_format — unexpected argument for command");
                return;
            }


        }



        // char* endptr = nullptr;
        // // Parse strings to floats and validate the physical limits of the motors
        // float value = std::strtof(token.c_str(), &endptr);
        
        // // Check number format (exception-free)
        // if (endptr == token.c_str() || *endptr != '\0') {
        //     reply("ERROR invalid_format — expected numbers separated by spaces");
        //     return;
        // }

        // // Group parameters into packets of 4: Angle, Speed, Acceleration, Jerk
        // int param_type = token_count % 4;

        // if (param_type == 0) { // ANGOLO
        //     if (value < ANGLE_MIN || value > ANGLE_MAX) {
        //         reply("ERROR angle_out_of_range — value " + token + 
        //               " not in [" + std::to_string(ANGLE_MIN) + ", " + std::to_string(ANGLE_MAX) + "]");
        //         return;
        //     }
        //     angles.push_back(value);
            
        //     // Check sul numero massimo di servi (fatto solo quando leggiamo un nuovo angolo)
        //     if (angles.size() > MAX_SERVOS) {
        //         reply("ERROR too_many_servos — max allowed is " + std::to_string(MAX_SERVOS));
        //         return;
        //     }
            
        // } else if (param_type == 1) { // VELOCITÀ
        //     if (value < MIN_SPEED || value > MAX_SPEED) {
        //         reply("ERROR speed_out_of_range — value " + token + 
        //               " not in [" + std::to_string(MIN_SPEED) + ", " + std::to_string(MAX_SPEED) + "]");
        //         return;
        //     }
        //     velocities.push_back(value);
            
        // } else if (param_type == 2) { // ACCELERAZIONE
        //     if (value < MIN_ACC || value > MAX_ACC) {
        //         reply("ERROR acc_out_of_range — value " + token + 
        //               " not in [" + std::to_string(MIN_ACC) + ", " + std::to_string(MAX_ACC) + "]");
        //         return;
        //     }
        //     accelerations.push_back(value);
            
        // } else if (param_type == 3) { // JERK
        //     if (value < MIN_JERK || value > MAX_JERK) {
        //         reply("ERROR jerk_out_of_range — value " + token + 
        //               " not in [" + std::to_string(MIN_JERK) + ", " + std::to_string(MAX_JERK) + "]");
        //         return;
        //     }
        //     jerks.push_back(value);
        // }

        token_count++;
    }

    // 1. Check if the string is not empty
    if (token_count == 0) {
        reply("ERROR empty_command — send values as: angle vel acc jerk ...");
        return;
    }

    // 2. Check if the values are coming in complete packets of four
    // if (token_count % 4 != 0) {
    //     reply("ERROR incomplete_data — each servo requires exactly 4 parameters (angle, speed, acc, jerk)");
    //     return;
    // }

    //3. Verify that the number of received command groups matches the current peripheral count
    // if (angles.size() != s_num_servos) {
    //     reply("ERROR wrong_count — expected data for " + std::to_string(s_num_servos) + 
    //           " servos, but received data for " + std::to_string(angles.size()));
    //     return;
    // }

    // OK — callback
    reply("OK");
    
    // ATTENZIONE: Now you have to pass all the four parameters to callback
    if (s_on_command) {
        s_on_command(command);
    }
}