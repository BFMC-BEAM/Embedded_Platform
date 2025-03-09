#ifndef POS_CALCULATION_HPP
#define POS_CALCULATION_HPP

// TODO: Add your code here

#include <chrono>
#include <utils/task.hpp>
#include <periodics/imu.hpp>
#include <periodics/rpm_counter.hpp>
#include <brain/robotstatemachine.hpp>
#include <mbed.h>

namespace periodics
{
   /**
    * @brief Class pos_calculation
    *
    */
    class CPos_calculation: public utils::CTask
    {
        public:
            /* Constructor */
            CPos_calculation(
                std::chrono::milliseconds f_period,
                UnbufferedSerial& f_serial,
                CImu& imu,
                CRpm_counter& rpm_counter
            );
            /* Destructor */
            ~CPos_calculation();

            void serialCallbackPOScommand(char const * a, char * b);

        private:
            UnbufferedSerial&      m_serial;
            CImu& m_imu;                 
            CRpm_counter& m_rpm_counter; 
            /* private variables & method member */

            double _x = 0;
            double _y = 0;
            double _z = 0;

            double yaw = 0;
            double velocity = 0;

            int m_messageSendCounter;
            
            /* Run method */
            virtual void        _run();

            /** @brief Active flag  */
            bool m_isActive;

    }; // class CPos_calculation
}; // namespace periodics

#endif // POS_CALCULATION_HPP
