#pragma once

#include <stdint.h>
#include <vector>
#include <functional>
#include <string>

enum Gcode {
    G6,
    M222,
    M204,
    M205,
    G4,
    M24,
    M25
};
enum Args{
    P,
    S,
    A,
    J,
    R,
    N
};
typedef struct{
    Gcode gcode;
    std::vector<float> values;
    std::vector<Args> args;
}Command;

// Callback chiamata quando arriva un comando valido dal computer.
// Contiene gli angoli dei servomotori (1-5 valori, range 0-270).
using ServoCommandCallback =  std::function<void(const Command& command)>;
void reply(const std::string& msg);
class ProtocolManager {
public:
    /**
     * Inizializza il protocollo.
     *
     * @param num_servos      Numero di servomotori collegati (1-5).
     *                        Inviato al computer ad ogni cambio.
     * @param on_servo_command Callback chiamata quando arriva un comando valido.
     */
    static void init(uint8_t num_servos, ServoCommandCallback on_servo_command);

    /**
     * Chiama questa funzione ogni volta che il numero di periferiche cambia.
     * Invia automaticamente il nuovo conteggio al computer.
     */
    static void set_num_servos(uint8_t num_servos);

    /**
     * Chiama questa funzione quando arriva una riga grezza dal TCP server.
     * Esegue il parsing, la validazione e invoca la callback o invia un errore.
     */
    static void handle_incoming(const std::string& line);
};
extern uint8_t s_num_servos;






