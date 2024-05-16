# Sensors

Sensors are required for the operation of the motor controller. There are a few required sensors for each loop.

- fast_loop
    - motor_encoder

- main_loop
    - output_encoder
    - torque_sensor

Each loop is run at a defined frequency and is run in an interrupt context. The sensor api required by the fast and main
loop are `void trigger()` and `int32_t/float read()`. The trigger function is called early in the loop before some other
processing, then read is called to reap the results. Other sensors may be added and read from the system_loop or main()
context. Often sensors share a common peripheral, which makes coordinating these loops more difficult. One solution is
to use SensorMultiplex and to make the primary sensor in the faster loop. Another solution is to use DMA to set up a
background read of sensors.

## Asynchronous reads

Often sensors also have diagnostic data. If one wishes to read this diagnostic data asynchronously to the sensor reads,
there are a few options. Sensors based on SPI may use the SPIPause feature which will temporarily pause sensor reads in
the interrupt context such that diagnostic data can be read in the main() context. SPIPause only supports two contexts,
main() and interrupt. It does not work to use SPIPause to read from two different interrupts of different priorities.
SPIPause will work to coordinate reads between main() and the interrupt both with blocking sensor reads and continuous
DMA reads. In both cases the interrupt data will be temporarily paused while getting the diagnostic data. SPIPause is
currently built in to the SPIDMA class and can be accessed from that class using `claim()` and `release()`. This system
is designed for efficiency in the interrupt context and requires extra overhead when used from the main() context for
diagnostics. SPIPause is a set of global exclusive locks, they can be accessed directly at
`SPIDMA::spi_pause[SPIDMA::SPx]`. Sometimes it is preferable to have diagnostic data read without pausing sensor reads.
In this case it is best to try to interleave diagnostic reads with sensor reads during the main sensor read. The ICPZDMA
class provides an example of this. 

### SPIPause example

```cpp
struct Sensor {
    ...
    void read() {
        // if paused this readwrite will skip (not block) and not update the outputs
        spidma_.readwrite(...);
    }
    int get_diag() {
        spidma_.claim();
        spidma_.readwrite(...);
        spidma_.release();
    }
    SPIDMA &spidma_;
};

SPIDMA spi1_dma1(SPI1, ...);u
Sensor sensor1(spi1_dma1);
SPIDMA spi1_dma2(SPI1, ...);
Sensor sensor2(spi1_dma2);

void isr() {
    sensor1.read();
    sensor2.read();
}

void main() {
    while(1) {
        ms_delay(1000);
        // will pause both isr reads() until complete
        sensor1.get_diag();
    }
}
```