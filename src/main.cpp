#include <Arduino.h>
#include <EEPROM.h>
#include <MIDIUSB.h>
#include <Bounce2.h>
#include <Encoder.h>

const bool direction = false; //if this is false the pedal direction is swapped

const uint8_t analog_pin = A0; //Pin connected to the potentiometer slider
const uint8_t button_pin = 16;  // Button on rotary encoder
const uint8_t encoder_pins[2] = {0, 1}; //Rotary encoder pins
const uint8_t encoder_divisor = 4;  //Resolution of encoder

const uint8_t midi_channel = 4; //Selected midi channel
const uint8_t cc_control = 0; //CC control number

const int analog_hys = 2; //Hysteresis of analog value before sending new MIDI CD message
const int analog_min = 0; //Minimum possible analog value
const int analog_max = 1023; //Maximum possible analog value
const int cc_min = 0; //Minimm value of cc signal according to MIDI standard
const int cc_max = 127; //Maximim value of cc signal accoring to MIDI standard;
const int analog_low_lim = analog_min+10; //Maximum analog value that counts as lowest value
const int analog_high_lim = analog_max-10; //Minimum analog value that counts as higest value
const unsigned int cc_swap_diff = 1 ; //Auto swap if cc value set at the end position of the pedal is this close to the other end.

// unsigned long t_prev=millis(); //Previous time for sampling
int analog_low = 0; //Analog signal at low value
int analog_high = 1023; //Analog signal at low value
int analog_range = analog_high-analog_low; //Maximum value at high value
int cc_low = 0; //CC value at analog low value
int cc_high = 127; //CC value at analog low value
int cc_range; //CC value at analog high value
int cc_range_min;
int cc_range_max;

int analog_value = 0; //Analog reading ofthe pedal
int analog_value_hys = 0; //Analog value with hysteresis
uint8_t cc_value = 0; //Value of controller MIDI output
long knob_value = 0;

//States of the pedal
uint8_t state = 0; 
const uint8_t state_normal = 0; // Normal operation
const uint8_t state_set_cc_high = 1; // Set cc value for the high analog value, accessed by pressing knob when pedal at low end
const uint8_t state_set_cc_low = 2; // Set cc value for the low analog value, accessed by pressing knob when pedal at high end
const uint8_t state_calibrate = 3; // Setup calibration, accesed by presing knob when powering up.

Encoder knob(0,1);
Bounce2::Button button = Bounce2::Button();

//General purpose functions
inline int limit(int x, int low, int high) {
    return (x>high ) ? high : ((x<low) ? low : x);
}

inline bool is_close_to(int x, int ref, int range) {
    return (x <= ref+range) && (x >=ref-range);
}

inline void debug_begin() {
    #ifdef DEBUG 
        Serial.begin(115200);
        while (!Serial) delay(10); //Wait for serial
    #endif
}

inline void debug_int(const char *str, int value) {
    #ifdef DEBUG
        Serial.print(str); 
        Serial.println(value); 
    #endif
}

inline void debug_str(const char *str) {
    #ifdef DEBUG
        Serial.println(str); 
    #endif
}

//Midi functions
void send_CC(uint8_t value) {

    midiEventPacket_t event = {0x0B, (uint8_t)(0xB0 | midi_channel), cc_control, (uint8_t)(value & 0x7F)};

    MidiUSB.sendMIDI(event);
    MidiUSB.flush();
}

void calc_cc_range() {
    cc_range = cc_high-cc_low;
    //Increase cc_range by one
    cc_range += ( cc_range >= 0 ) ? 1 : -1;
    cc_range_min = min(cc_low, cc_high);
    cc_range_max = max(cc_low, cc_high);
    debug_int("cc_low=", cc_low);
    debug_int("cc_high=", cc_high);
    debug_int("cc_range=", cc_range);
    debug_int("cc_rane_min=", cc_range_min);
    debug_int("cc_range_max=", cc_range_max);
}

inline uint8_t map_analog_to_cc(long a) {
    //Convert an analog value to a cc value based on global settings
    return limit(((a-analog_low)*cc_range)/analog_range+cc_low, cc_range_min, cc_range_max);
}

//EEPROM functions
inline bool readEEPROM_int_value(int &addr, int &value, int min_value, int max_value) {
    int result;
    EEPROM.get(addr, result);
    addr+=sizeof(result);
    Serial.print("EEPROM["); 
    Serial.print(addr);
    Serial.print("]="); 
    Serial.print(result);
    if(result>=min_value && result<=max_value) {
        debug_str(" Value in range, updating value.");
        value=result;
        return true;
    }
    debug_str(" Value not in range!");
    return false;
}

inline void writeEEPROM_int_value(int &addr, int value) {
    debug_int("EEPROM write to address=",addr);
    debug_int("                value=",value);
    EEPROM.put(addr,value);
    addr+=sizeof(value);
}

bool get_settings() {
    int addr = 0;
    bool ok = true;
    ok &= readEEPROM_int_value(addr, analog_low, analog_min, analog_low_lim);
    ok &= readEEPROM_int_value(addr, analog_high, analog_high_lim, analog_max);
    ok &= readEEPROM_int_value(addr, cc_low, cc_min, cc_max);
    ok &= readEEPROM_int_value(addr, cc_high, cc_min, cc_max);
    calc_cc_range();
    return ok;
}

void apply_settings() {
    int addr = 0;
    writeEEPROM_int_value(addr,analog_low);
    writeEEPROM_int_value(addr,analog_high);
    writeEEPROM_int_value(addr,cc_low);
    writeEEPROM_int_value(addr,cc_high);
    calc_cc_range();
}

//Main setup
void setup() {

    button.attach( button_pin ,  INPUT_PULLUP );
    button.setPressedState(LOW); 

    debug_begin();

    button.update();
    if( !get_settings() || button.isPressed() ) 
    {
        state = state_calibrate;
        analog_high = analog_min;
        analog_low = analog_max;
        debug_str("Calibration mode");
    }
    else
    {
        calc_cc_range();
        state = state_normal;
        debug_str("Normal mode");
    }
}

//Main loop
void loop() {
        // Handle pedal readings
        analog_value = (direction) ? analogRead(analog_pin) : analog_max-analogRead(analog_pin);
        uint8_t old_cc_value = cc_value;
        if ( !is_close_to(analog_value, analog_value_hys, analog_hys) ) {
            analog_value_hys = analog_value;
            cc_value = map_analog_to_cc(analog_value_hys);
        }
        bool cc_value_changed = cc_value != old_cc_value;
        if( cc_value_changed ) debug_int("CC-value: ", cc_value);

        // Handle button on rotary encoder
        button.update();
        if( button.pressed() ) debug_str("Button pressed");

        // Handle rotary encoder
        long old_knob_value = knob_value;
        long raw_knob_value = knob.read();
        knob_value = limit(raw_knob_value, cc_min, cc_max);
        if ( knob_value != raw_knob_value ) knob.write(knob_value);
        bool knob_moved = knob_value != old_knob_value; 
        if( knob_moved ) debug_int("Knob value: ", knob_value);

        //State machine
        switch (state)
        {   
            case state_calibrate:
                if( analog_value > analog_high ) {
                    analog_high=analog_value;
                    debug_int("Analog high value updated to: ", analog_high);
                }
                if( analog_value < analog_low ) {
                    analog_low=analog_value;
                    debug_int("Analog low value updated to: ", analog_low);
                }
                if( button.pressed() && analog_low <= analog_low_lim && analog_high >= analog_high_lim ) {
                    calc_cc_range();
                    apply_settings();
                    state=state_normal;
                    debug_str("Exit calibration mode");
                }
                break;
            case state_set_cc_low:
                if(  analog_value > analog_low_lim ) {
                    // Exit if pedal is not at end position
                    debug_str("Pedal moved from limit, exiting cc-range setup");
                    send_CC(cc_value);
                    state = state_normal;
                } 
                else if( button.pressed() ) {
                    // Button pressed, calibration ready
                    if( is_close_to(knob_value, cc_high, cc_swap_diff) ) {
                        // The selected cc value is close to the high limit let's swap the values
                        cc_high = cc_low;
                        debug_str("Close to the high value, doing auto swap");
                    }
                    cc_low = knob_value;
                    apply_settings();
                    state = state_normal;
                }
                else if( knob_moved ) {
                    //Knob moved, send new value over midi so that user can see progress
                    send_CC(knob_value);
                }                    
                break;
            case state_set_cc_high:
                if(  analog_value < analog_high_lim ) {
                    debug_str("Pedal moved from limit, exiting cc-range setup");
                    send_CC(cc_value);                    
                    state = state_normal;
                } 
                else if( button.pressed() ) {
                    // Button pressed, calibration ready
                    if( is_close_to(knob_value, cc_low, cc_swap_diff) ) {
                        // The selected cc value is close to the low limit let's swap values
                        debug_str("Close to the low value, doing auto swap");
                        cc_low = cc_high;
                    }
                    cc_high = knob_value;
                    apply_settings();
                    state = state_normal;
                }
                else if( knob_moved ) {
                    //Knob moved, send new value over midi so that user can see progress
                    send_CC(knob_value);
                }                    
                break;
            default: // state_normal
                if( cc_value_changed ) {
                    send_CC(cc_value);
                }
                if ( button.pressed() ) {
                    if(  analog_value > analog_high_lim ) {
                        debug_str("Setting cc-value for the high end");
                        knob.write(cc_value);
                        state = state_set_cc_high;
                    } 
                    else if ( analog_value < analog_low_lim ) {
                        debug_str("Setting cc-value for the low end");
                        knob.write(cc_value);
                        state = state_set_cc_low;
                    } 
                }
        }
}

/* Sample code snippets
    const unsigned long h = 2; //Sample time in ms
    unsigned long t_cur = millis();
    
    if(t_cur - t_prev > h) {
        t_prev= t_cur;
    }
*/