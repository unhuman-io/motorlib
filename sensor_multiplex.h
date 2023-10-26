#ifndef UNHUMAN_MOTORLIB_SENSOR_MULTIPLEX_H_
#define UNHUMAN_MOTORLIB_SENSOR_MULTIPLEX_H_

#include "sensor.h"
#include "torque_sensor.h"


// alternates reads between two sensors. Calling SensorMultiplex functions alternate between 
// triggering the primary and secondary reads. It returns the primary sensor values
// Using secondary() returns a SecondarySensor functions only returns values
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
        void clear_faults() { secondary_->clear_faults(); }
     private:
        Sensor2 *secondary_;
    };

    SensorMultiplex(Sensor1 &primary, Sensor2 &secondary, uint8_t decimation = 0) :
        primary_(primary), secondary_(secondary), secondary_read_(&secondary), decimation_(decimation) {}
    SecondarySensor &secondary() { return secondary_read_; }
    int32_t read() {if (count_ == 0) {
         if (toggle_) primary_.read(); else secondary_.read();  
            toggle_ = !toggle_;
         }
         return primary_.get_value(); 
      }
    int32_t get_value() const { return primary_.get_value(); }
    void trigger() { 
       if (count_++ >= decimation_) {
         count_ = 0;
         if (toggle_) { 
            primary_.reinit(); primary_.trigger(); 
         } else { 
            secondary_.reinit(); secondary_.trigger(); 
         }
       }
    }
    int32_t get_index_pos() const { return primary_.get_index_pos(); }
    bool index_received() const { return primary_.index_received(); }
    bool init() { return primary_.init(); }
    void clear_faults() { primary_.clear_faults(); }

 protected:
    Sensor1 &primary_;
    Sensor2 &secondary_;
    SecondarySensor secondary_read_;
    bool toggle_ = false;
    uint8_t count_;
    uint8_t decimation_;
};

// Specialization for when a torque sensor is the primary sensor
template<class Sensor1, class Sensor2>
class TorqueSensorMultiplex : public SensorMultiplex<Sensor1, Sensor2> {
 public:
   TorqueSensorMultiplex(Sensor1 &primary, Sensor2 &secondary, uint8_t decimation = 0) : SensorMultiplex<Sensor1, Sensor2>(primary, secondary, decimation),
      gain_(primary.gain_), bias_(primary.bias_), k_temp_(primary.k_temp_), torque_(primary.torque_) {}
   using SensorMultiplex<Sensor1, Sensor2>::trigger;
   using SensorMultiplex<Sensor1, Sensor2>::init;
   void set_param(const TorqueSensorParam &param) { this->primary_.set_param(param); }
   float read() {
      if (this->count_ == 0) {
         if (this->toggle_) this->primary_.read(); else this->secondary_.read();  
                           this->toggle_ = !this->toggle_;
         }
      return this->primary_.get_value();
   }
   
   float get_value() const { return this->primary_.get_value(); }
   void clear_faults() { this->primary_.clear_faults(); }
 protected:
   float &gain_, &bias_, &k_temp_, &torque_;
   friend class System;
};

#endif  // UNHUMAN_MOTORLIB_SENSOR_MULTIPLEX_H_
