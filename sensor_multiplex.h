#pragma once

#include "sensor.h"


// alternates reads between two sensors. Calling SensorMultiplex functions return the primary sensor values
// Calling the secondary sensor functions only returns values
template<class Sensor1, class Sensor2>
class SensorMultiplex : public SensorBase {
 public:
    class SecondarySensor : public SensorBase {
     public:
        SecondarySensor(Sensor2 *secondary) : secondary_(secondary) {}
        int32_t get_value() const { return secondary_->get_value(); }
        int32_t read() { return get_value(); }
        bool init() { return secondary_->init(); }
        bool index_received() const { return secondary_->index_received(); }
     private:
        Sensor2 *secondary_;
    };

    SensorMultiplex(Sensor1 &primary, Sensor2 &secondary) :
        primary_(primary), secondary_(secondary), secondary_read_(&secondary) {}
    SecondarySensor &secondary() { return secondary_read_; }
    int32_t read() {if (toggle_) primary_.read(); else secondary_.read();  
                            toggle_ = !toggle_;
                            return primary_.get_value(); }
    int32_t get_value() const { return primary_.get_value(); }
    void trigger() { if (toggle_) primary_.trigger(); else secondary_.trigger(); }
    int32_t get_index_pos() const { return primary_.get_index_pos(); }
    bool index_received() const { return primary_.index_received(); }
    bool init() { return primary_.init(); }

 private:
    Sensor1 &primary_;
    Sensor2 &secondary_;
    SecondarySensor secondary_read_;
    bool toggle_ = false;
};
