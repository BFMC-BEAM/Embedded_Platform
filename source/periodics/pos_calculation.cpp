#include "periodics/pos_calculation.hpp"
#include "periodics/imu.hpp"
#include "periodics/rpm_counter.hpp"

#define _100_chars                      100
#define DELTA                           0.1
#define CONVERT_CM_TO_M                 0.01
#define TIME_TO_SEND_MSG_MS             500
// TODO: Add your code here
namespace periodics
{
   /**
    * @brief Class constructor pos_calculation
    *
    */
    CPos_calculation::CPos_calculation(
        std::chrono::milliseconds f_period,
        UnbufferedSerial& f_serial,
        CImu& f_imu,
        CRpm_counter& f_rpm_counter
    )
    : utils::CTask(f_period)
    , m_serial(f_serial)
    , m_imu(f_imu)
    , m_rpm_counter(f_rpm_counter)
    {
        /* constructor behaviour */
        _timer.start();
    }

    /** @brief  CPos_calculation class destructor
     */
    CPos_calculation::~CPos_calculation()
    {
    }

    void CPos_calculation::serialCallbackPOScommand(char const * a, char * b) {
        uint8_t l_isActivate = 0;
        uint8_t l_res = sscanf(a, "%hhu", &l_isActivate);

        if (1 == l_res) {
            if (uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30) {
                m_isActive = (l_isActivate >= 1);
                bool_globalsV_ultra_isActive = (l_isActivate >= 1);
                sprintf(b, "1");
            } else {
                sprintf(b, "kl 15/30 is required!!");
            }
        } else {
            sprintf(b, "syntax error");
        }
    }

    /* Run method */
    void CPos_calculation::_run()
    {
        /* Run method behaviour */
        // if(!m_isActive) return;

        char buffer[_100_chars];      
        float yaw = m_imu.getYaw();
        int velocity = m_rpm_counter.getVelocity();

        // Cálculo de la posición

        _x += velocity * cos(yaw)*DELTA*CONVERT_CM_TO_M;    //acá hay un error xq hace mal la integral, no deberia ser una cte DELTA
        _y += velocity * sin(yaw)*DELTA*CONVERT_CM_TO_M;

        if (isTimeToSendMsg())
        {
            snprintf(buffer, sizeof(buffer), "@pos:%3.2f;%2.2f;%2.2f;%3d;;\r\n", yaw,_x,_y,velocity);
            m_serial.write(buffer,strlen(buffer));
        }
    }

    //cuenta un determinado tiempo y retorna true cuando llega a ese valor. luego reinicia el ciclo de cuentas
    bool CPos_calculation::isTimeToSendMsg(void)
    {   
        int currentTimeMs = _timer.elapsed_time().count()/1000;
        //printf("Tiempo transcurrido en mS:%5d ;;\r\n", currentTimeMs);    //descomentar para ver como se incrementa el timer
        if(currentTimeMs < TIME_TO_SEND_MSG_MS)
        {
            return false;
        }
        _timer.reset();
        _timer.start();
        return true;
    }
    
}; // namespace periodics