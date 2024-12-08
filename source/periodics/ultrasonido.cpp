#include "periodics/ultrasonido.hpp"

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
        PinName Echo,
        PinName Trigger)
        : utils::CTask(f_period), m_serial(f_serial), m_echo(Echo), m_trigger(Trigger)
    {
        /* constructor behaviour */
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


        printf("Starting Ultrasonic sensor data acquisition...\r\n");

    }

}; // namespace periodics