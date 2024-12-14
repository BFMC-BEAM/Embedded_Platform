#ifndef ULTRASONIDO_HPP
#define ULTRASONIDO_HPP

#include <mbed.h>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include <chrono>
#include <periodics/alerts.hpp>
#include <drivers/speedingmotor.hpp>

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
                drivers::ISpeedingCommand&    f_speedingControl,
                mbed::DigitalOut& Trigger,
                mbed::InterruptIn& EchoFront,
                mbed::InterruptIn& EchoBack,
                mbed::InterruptIn& EchoLeft,
                mbed::InterruptIn& EchoRight
            );
            /* Destructor */
            ~CUltrasonido();
            /* Serial callback implementation */
            void serialCallbackULTRAcommand(char const * a, char * b);
        private:
            /* private variables & method member */

            UnbufferedSerial& m_serial;
            mbed::DigitalOut& m_trigger;
            mbed::InterruptIn& m_echoFront;     
            mbed::InterruptIn& m_echoBack;     
            mbed::InterruptIn& m_echoLeft;     
            mbed::InterruptIn& m_echoRight;     
            drivers::ISpeedingCommand& m_speedControl;
            mbed::Timer m_timerFront;
            mbed::Timer m_timerBack;
            mbed::Timer m_timerLeft;
            mbed::Timer m_timerRight;
            
            bool m_isActive;
            bool m_objectDetected;

            int n_muestras;
            int prom_muestrasFront;
            int prom_muestrasBack;
            int prom_muestrasLeft;
            int prom_muestrasRight;
            int l_distanceFront;
            int l_distanceBack;
            int l_distanceLeft;
            int l_distanceRight;

            void startTimerFront();
            void stopTimerFront();
            void startTimerBack();
            void stopTimerBack();
            void startTimerLeft();
            void stopTimerLeft();
            void startTimerRight();
            void stopTimerRight();

            /* Run method */
            virtual void _run();
    }; // class CUltrasonido
}; // namespace periodics

#endif // ULTRASONIDO_HPP
