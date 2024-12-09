#ifndef ULTRASONIDO_HPP
#define ULTRASONIDO_HPP

// TODO: Add your code here

#include <mbed.h>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include <chrono>

namespace periodics
{
   /**
    * @brief Class ultrasonido
    *
    */
    class CUltrasonido: public utils::CTask
    {
        public:
            /* Constructor */
            CUltrasonido(
                std::chrono::milliseconds f_period,
                UnbufferedSerial& f_serial,
                mbed::DigitalOut Trigger,
                mbed::InterruptIn& Echo
            );
            /* Destructor */
            ~CUltrasonido();
            /* Serial callback implementation */
            void serialCallbackULTRAcommand(char const * a, char * b);
        private:
            /* private variables & method member */

            UnbufferedSerial& m_serial;
            mbed::DigitalOut m_trigger;
            mbed::InterruptIn& m_echo;     
            int l_distance;

            //                                                                                      //
            // Se descarto promedio de muestras por la baja velocidad de respuesta del sensor       //
            //                                                                                      //
            // int n_muestras;
            // int prom_muestras;
            
            void triggerPulse();
            void startTimer();
            void stopTimer();

            /* Run method */
            virtual void        _run();

            /** @brief Active flag  */
            bool m_isActive;
            
            mbed::Timer m_timer_trigger;
            mbed::Timer m_timer_echo;   

    }; // class CUltrasonido
}; // namespace periodics

#endif // ULTRASONIDO_HPP
