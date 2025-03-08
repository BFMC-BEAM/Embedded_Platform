/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

#ifndef IMU_H
#define IMU_H

#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)
#define I2C_BUFFER_LEN 8
#define I2C0           5

/* The mbed library */
#include <mbed.h>
#include <drivers/bno055.hpp>
#include <utils/task.hpp>
#include <brain/globalsv.hpp>
#include <chrono>

namespace periodics
{
   /**
    * @brief Class imu 
    * 
    * 
    */
    class CImu : public utils::CTask
    {
        public:
            /* Constructor */
            CImu(
                std::chrono::milliseconds    f_period, 
                UnbufferedSerial& f_serial,
                PinName SDA,
                PinName SCL
            );
            /* Destructor */
            ~CImu();
            static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
            static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
            static void BNO055_delay_msek(u32 msek);
            void serialCallbackIMUcommand(char const * a, char * b);
            void serialCallbackSPEEDcommand(char const * a, char * b);
            double getYaw(); // Método para obtener el valor de yaw
        private:
            virtual void I2C_routine(void);
            virtual void    _run();
            struct bno055_t bno055;
            static I2C* i2c_instance;
            bool            m_isActive;
            UnbufferedSerial&      m_serial;
            s32 m_delta_time;

            /* Variable contador para limitar la velocidad de envío del mensaje */
            int m_messageSendCounter;

            double yaw;
            double pitch;
            double roll;

            double dt;

    }; // class CImu

}; // namespace utils

#endif // IMU_H