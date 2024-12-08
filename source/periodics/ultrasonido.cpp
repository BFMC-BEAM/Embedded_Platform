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
            mbed::DigitalIn Echo
        )
        : utils::CTask(f_period)
        , m_serial(f_serial)
        , m_echo(Echo)
        , m_trigger(Trigger)
    {
        /* constructor behaviour */
        m_trigger = 0;
        m_echo.mode(PullDown);
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
        ThisThread::sleep_for(chrono::microseconds(5).count() / 1000); // Convert microseconds to milliseconds
        m_trigger = 0;
        ThisThread::sleep_for(chrono::microseconds(10).count() / 1000); // Convert microseconds to milliseconds
        m_trigger = 1;



    }

}; // namespace periodics