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
                mbed::DigitalIn Echo
            );
            /* Destructor */
            ~CUltrasonido();
            /* Serial callback implementation */
            void serialCallbackULTRAcommand(char const * a, char * b);
        private:
            /* private variables & method member */

            UnbufferedSerial& m_serial;
            mbed::DigitalOut m_trigger;
            mbed::DigitalIn m_echo;     

            /* Run method */
            virtual void        _run();

            /** @brief Active flag  */
            bool m_isActive;


            mbed::Timer m_timer;   

    }; // class CUltrasonido
}; // namespace periodics

#endif // ULTRASONIDO_HPP
