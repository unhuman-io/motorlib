#pragma once

class TorqueSensor {
 public:
    virtual void init() {}
    virtual void trigger() {}
    virtual float read() = 0;
};
