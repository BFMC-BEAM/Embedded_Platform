#include "drivers/rpmcounter.hpp"

Counter::Counter(PinName pin) : _interrupt(pin) {
    _interrupt.rise(callback(this, &Counter::increment));
}

void Counter::increment() {
    _count++;
}

int Counter::read() {
    return _count;
}
