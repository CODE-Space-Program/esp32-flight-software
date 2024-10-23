/* HEADERS */
#include "sensors.h"


/* DATA STRUCTURES */
// `State` represents all states of the flight and has an additional "Boot" and "Error" state
enum class State {

    Boot,
    Ready,
    Flight,
    Chute,
    Land,

    Error,

} STATE;

/* SETUP  

    This function only runs once when the flight computer is turned on;
    Here we can initialize all the sensors 

*/



void setup() {

    // initialize all sensors

}


/* LOOP

    This funtion will run for the whole duration of the flight

*/

void loop() {

    // switch case function for all the flight case scenarios
    switch (STATE)  {

    case State::Boot:
        /* code */
        break;
    
    case State::Ready:
        /* code */
        break;

    case State::Flight:
        /* code */
        break;
    
    case State::Chute:
        /* code */
        break;
    
    case State::Land:
        break;

    case State::Error:
        break;
    }
}