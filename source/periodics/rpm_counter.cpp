#include "periodics/rpm_counter.hpp"

#define M_PI 3.1415926535897932
#define RADIOUS 0.05

// TODO: Add your code here
namespace periodics
{
    /**
     * @brief Class constructor rpm_counter
     *
     */
     CRpm_counter::CRpm_counter(
          std::chrono::milliseconds f_period,
          mbed::InterruptIn& rpmCounterPin
          )
          : utils::CTask(f_period)
          , rpmCounterPin(rpmCounterPin)
          , _count(0)
          , _rpm(0)
     {
          _timer.start(); // Inicializa el temporizador
          rpmCounterPin.rise(callback(this, &CRpm_counter::increment));
     }

     /** @brief  CRpm_counter class destructor
      */
     CRpm_counter::~CRpm_counter()
     {
     }

    void CRpm_counter::increment() {
        // Get the elapsed time in milliseconds since the timer was started or last reset
        auto elapsed = _timer.elapsed_time().count();
    
        if (elapsed > 5) { // Evita contar pulsos si han pasado menos de 5 ms | Debounce
             _count++;
             _timer.reset();
             _timer.start(); // Restart the timer after reset
        }
    }
    
    double CRpm_counter::read() {
        return _count;
    }
    
    double CRpm_counter::getRpm() {
        return _rpm;
    }

    double CRpm_counter::getVelocity() {
        return _rpm * RADIOUS * 2 * M_PI/60;
    }

    /* Run method */
    void CRpm_counter::_run()
    {
        /* Run method behaviour */
        if(!m_isActive) return;

        // Get the elapsed time in milliseconds since the timer was started or last reset
        auto elapsed_ms = _timer.elapsed_time().count();

        if (elapsed_ms > 0) { 
             // Cálculo de RPM
             _rpm = (_count * 60000) / elapsed_ms;

             // Reiniciar contador y actualizar tiempo de referencia
             _count = 0;
             _timer.reset();
             _timer.start(); // Restart the timer after reset
        }

    }

}; // namespace periodics
