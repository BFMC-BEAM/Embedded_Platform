#include "periodics/rpm_counter.hpp"
#include <cmath>
#ifndef M_PI
    #define M_PI 3.1415926535897932
#endif

#define RADIOUS 0.005
#define MIN_DELTA_TIME_MS 100   //si el encoder itera 1 vez cada 10cm -> delta minimo a 60cm/seg = 167 [ms]
#define MAX_DELTA_TIME_MS 2100   //si el encoder itera 1 vez cada 10cm -> delta minimo a 5cm/seg = 2000 [ms]
#define MAX_DELAY_TO_RESET_MS 3000
#define DEFAULT_RPM 0
#define DEFAULT_CMS_VEL 0

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
          , deltaTimeMs(0)
          , previousDeltaTimeMs(0)
          , previousCount(0)
          , TimeToResetMs(0)
          , velCMS(0)
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

    //interrupcion por flanco del pin del encoder
    void CRpm_counter::increment() {
        // Get the deltaTimeMs time in milliseconds since the timer was started or last reset
        deltaTimeMs=validDeltaTimeMs(_timer.elapsed_time().count()/1000);
        calculateRPM(deltaTimeMs);
        calculateVelocityCMS(_rpm);
        _count++;

        _timer.reset();
        _timer.start(); // Restart the timer after reset
    }
    
    int CRpm_counter::read() {
        return _count;
    }
    
    int CRpm_counter::getRpm() {
        return _rpm;
    }

    int CRpm_counter::getVelocity() {
        //return _rpm * RADIOUS * 2 * M_PI/60;
        return velCMS;
    }

    /* Run method */
    void CRpm_counter::_run()
    {
        // if(!m_isActive) return;
        
        TimeToResetMs=validDeltaTimeMs(_timer.elapsed_time().count()/1000);
        printf("timer base %d | timer reset %d", (int)_timer.elapsed_time().count()/1000000, (int)TimeToResetMs);
        if(TimeToResetMs > MAX_DELAY_TO_RESET_MS)
        {   
            //printf("filtro de rebote esta manqueando");
            _rpm = DEFAULT_RPM;
            velCMS = DEFAULT_CMS_VEL;
            TimeToResetMs = MAX_DELAY_TO_RESET_MS;
        }

        printf("RPM: %d | CM/S: %d | Count: %d | deltaTimeMs: %d | Time to Reset RPM: %d \n", _rpm, velCMS , _count, deltaTimeMs, TimeToResetMs);

        previousCount=_count;

    }
    
    //valida que el delta de tiempo sea mayor al minimo posible para filtrar rebotes indeseados del encoder
    int CRpm_counter::validDeltaTimeMs(int currentDeltaTimeMs)
    {
        if(currentDeltaTimeMs < MIN_DELTA_TIME_MS)
        {
            return previousDeltaTimeMs;
        }
            
        previousDeltaTimeMs = currentDeltaTimeMs;
        return currentDeltaTimeMs;
    }

    //calcula el rpm en base al delta de tiempo. Si el delta de tiempo es muy alto 
    void CRpm_counter::calculateRPM(int currentDeltaTimeMs)
    {
        if(currentDeltaTimeMs >= MAX_DELTA_TIME_MS)
            _rpm = DEFAULT_RPM;
        else 
            _rpm = (60000.0 / currentDeltaTimeMs);
    }

    //convierte la velocidad de RPM a cm/Seg
    void CRpm_counter::calculateVelocityCMS(int RpmVelocity)
    {
        velCMS = (int)((float)RpmVelocity * 1.0472);
    }
    
    
}; // namespace periodics
