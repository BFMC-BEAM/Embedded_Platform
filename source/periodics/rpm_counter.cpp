#include "periodics/rpm_counter.hpp"

#define M_PI 3.1415926535897932
#define RADIOUS 0.005

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
          rpmCounterPin.fall(callback(this, &CRpm_counter::increment));
     }

     /** @brief  CRpm_counter class destructor
      */
     CRpm_counter::~CRpm_counter()
     {
     }

    void CRpm_counter::serialCallbackRPMcommand(char const * a, char * b) {
        uint8_t l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);

        if (1 == l_res) {
            m_isActive = (l_isActivate >= 1);
            sprintf(b, "1");
        } else {
            sprintf(b, "syntax error");
        }
    }

    void CRpm_counter::increment() {
        // Get the elapsed time in milliseconds since the timer was started or last reset
        auto elapsed = _timer.elapsed_time().count();
        
        // printf("Elapsed time: %d\n", elapsed);
        
        // Calculate RPM based on the elapsed time
        if( elapsed >= 1000) _rpm = 0;
        else _rpm = (60000.0 / elapsed);

        _count++;

        _timer.reset();
        _timer.start(); // Restart the timer after reset
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
        // if(!m_isActive) return;

        printf("RPM: %f | Count: %d \n", _rpm, _count);

    }

}; // namespace periodics
