#include "periodics/ultrasonido.hpp"
#include "mbed.h"

// TODO: Add your code here
namespace periodics
{
    /**
     * @brief Class constructor ultrasonido
     *
     */
    CUltrasonido::CUltrasonido(
            std::chrono::milliseconds f_period,
            UnbufferedSerial &f_serial,
            mbed::DigitalOut Trigger,
            mbed::InterruptIn &Echo
        )
        : utils::CTask(f_period)
        , m_serial(f_serial)
        , m_echo(Echo)
        , m_trigger(Trigger)
    {
        /* constructor behaviour */
        m_trigger = 0;
        m_echo.mode(PullDown);
        m_timer_trigger.start();
        l_distance=0;

        //                                                                                      //
        // Se descarto promedio de muestras por la baja velocidad de respuesta del sensor       //
        //                                                                                      //
        
        // n_muestras=0;
        // prom_muestras=0;
    }

    /** @brief  CUltrasonido class destructor
     */
    CUltrasonido::~CUltrasonido()
    {
    }

    void CUltrasonido::serialCallbackULTRAcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_ultra_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
            
        }else{
            sprintf(b,"syntax error");
        }
    }

    /* Run method */
    void CUltrasonido::_run()
    {
        /* Run method behaviour */
        if(!m_isActive) return;

        m_trigger = 1;
        if(m_timer_trigger.elapsed_time().count() >= 10){
            m_timer_trigger.reset();
            m_trigger = 0;
        }
        else
        {
            m_trigger = 1;
        }


        m_echo.rise(mbed::callback(this, &CUltrasonido::startTimer));
        m_echo.fall(mbed::callback(this, &CUltrasonido::stopTimer));

        //                                                                                      //
        // Se descarto promedio de muestras por la baja velocidad de respuesta del sensor       //
        //                                                                                      //

        // if (n_muestras<5)
        // {
        //     n_muestras++;
        //     prom_muestras += l_distance;
        // }
        // else
        // {
        //     printf("@ultra: %d\n\r;;", prom_muestras/5);
        //     n_muestras = 0;
        //     prom_muestras = 0;
        // }

        printf("@ultra: %d\n\r;;", l_distance);

    }

    void CUltrasonido::triggerPulse() {
        m_timer_echo.reset();
        m_timer_echo.start();
    }

    void CUltrasonido::startTimer() {
        m_timer_echo.reset();
        m_timer_echo.start();
    }

    void CUltrasonido::stopTimer() {
        m_timer_echo.stop();
        l_distance = m_timer_echo.elapsed_time().count() / 58.0; // Convert time to distance
    }   

}; // namespace periodics