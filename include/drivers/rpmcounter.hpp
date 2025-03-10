#ifndef RPMCOUNTER_HPP
#define RPMCOUNTER_HPP

#include "mbed.h"

class Counter {
public:
    Counter(PinName pin);
    void increment();
    int read();
    
private:
    
    InterruptIn _interrupt;
    volatile int _count = 0;
};

#endif // RPMCOUNTER_HPP
