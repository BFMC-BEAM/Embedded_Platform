#ifndef RPM_COUNTER_HPP
#define RPM_COUNTER_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#include <mbed.h>

namespace periodics
{
   /**
    * @brief Class rpm_counter
    *
    */
    class CRpm_counter: public utils::CTask
    {
        public:
            /* Constructor */
            CRpm_counter(
                std::chrono::milliseconds f_period,
                mbed::InterruptIn& rpmCounterPin
            );
            /* Destructor */
            ~CRpm_counter();
            double getRpm();
            int getVelocity();

            void serialCallbackRPMcommand(char const * a, char * b);

        private:
            /* private variables & method member */

            mbed::InterruptIn& rpmCounterPin;
            // std::chrono::steady_clock::time_point _lastTime;
            mbed::Timer _timer;
            int _count;
            int previousCount;   
            int velCMS;         
            int _rpm;
            int TimeToResetMs;
            int deltaTimeMs; //tiempo transcurrido entre deteccion del encoder 
            int previousDeltaTimeMs; //diferencia de tiempos entre pulsos del encoder anterior
            
            void increment();
            double read();
            int validDeltaTimeMs(int currentDeltaTimeMs);
            void calculateRPM(int currentDeltaTimeMs);
            /* Run method */
            virtual void        _run();

            /** @brief Active flag  */
            bool m_isActive;

    }; // class CRpm_counter
}; // namespace periodics

#endif // RPM_COUNTER_HPP
