#include "periodics/pos_calculation.hpp"

#define _100_chars                      100

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
        
    }

    /** @brief  CPos_calculation class destructor
     */
    CPos_calculation::~CPos_calculation()
    {
    }

    /* Run method */
    void CPos_calculation::_run()
    {
        /* Run method behaviour */
        if(!m_isActive) return;

        char buffer[_100_chars];      
        float yaw = m_imu.getYaw();
        int velocity = m_rpm_counter.getVelocity();


        // Cálculo de la posición

        _x += velocity * cos(yaw);
        _y += velocity * sin(yaw);

        if (m_messageSendCounter >= 150)
        {

            m_messageSendCounter = 0;
            snprintf(buffer, sizeof(buffer), "@pos:%.3f;%.3f;;\r\n",_x,_y);
            m_serial.write(buffer,strlen(buffer));    
        }
        else
        {
            m_messageSendCounter++;
        }

    }

}; // namespace periodics