/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

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


#include <periodics/imu.hpp>
#include <cmath>  // Para sin(), cos(), M_PI
#include "brain/robotstatemachine.hpp"
#include "drivers/rpmcounter.hpp"

#define M_PI 3.1415926535897932

#define _100_chars                      100
#define BNO055_EULER_DIV_DEG_int        16
#define BNO055_LINEAR_ACCEL_DIV_MSQ_int 100
#define precision_scaling_factor        1000

#define PERIOD 10 // old:MAX_NOISE

namespace periodics{
    I2C* periodics::CImu::i2c_instance = nullptr;

    // Declarar una instancia de Counter
    Counter counter(PC_3);

    CImu::CImu(
            std::chrono::milliseconds    f_period, 
            UnbufferedSerial& f_serial,
            PinName SDA,
            PinName SCL)
        : utils::CTask(f_period)
        , m_isActive(false)
        , m_serial(f_serial)
    {
        dt = 0.01;

        A_ << 1, dt, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, dt,
              0, 0, 0, 1;

        B_ << 0.5 * dt * dt, 0,
              dt, 0,
              0, 0.5 * dt * dt,
              0, dt;

        H_ << 1, 0, 0, 0,
              0, 0, 1, 0;

        Q_ = Eigen::MatrixXd::Identity(4, 4) * 0.001;
        R_ = Eigen::MatrixXd::Identity(2, 2) * 5;
        P_ = Eigen::MatrixXd::Identity(4, 4) * 1;

        x_ = Eigen::VectorXd::Zero(4);

        // Inicializar la posición
        x_(0) = 0.7; // Posición en X
        x_(2) = 4.0; // Posición en Y

        // Inicializar las velocidades a cero (o a un valor conocido)
        x_(1) = 0.0; // Velocidad en X
        x_(3) = 0.0; // Velocidad en Y

        m_messageSendCounter = 0;

        if(m_delta_time < PERIOD){
            setNewPeriod(PERIOD);
            m_delta_time = PERIOD;
        }
        
        s32 comres = BNO055_ERROR;
        u8 power_mode = BNO055_INIT_VALUE;

        printf("Starting IMU sensor data acquisition...\r\n");  
        i2c_instance = new I2C(SDA, SCL);
        i2c_instance->frequency(400000);

        ThisThread::sleep_for(chrono::milliseconds(300));

        I2C_routine();

        comres = bno055_init(&bno055);

        power_mode = BNO055_POWER_MODE_NORMAL;

        comres += bno055_set_power_mode(power_mode);

        comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

        comres += bno055_set_accel_range(BNO055_ACCEL_RANGE_2G);

        u8 euler_unit_u8 = BNO055_INIT_VALUE;

        comres = bno055_get_euler_unit(&euler_unit_u8);

        if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
        {
            comres += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
        }

    }

    CImu::~CImu()
    {
        /*-----------------------------------------------------------------------*
        ************************* START DE-INITIALIZATION ***********************
        *-------------------------------------------------------------------------*/
        s32 comres = BNO055_ERROR;
        /* variable used to set the power mode of the sensor*/
        u8 power_mode = BNO055_INIT_VALUE;
        /*  For de - initializing the BNO sensor it is required
        * to the operation mode of the sensor as SUSPEND
        * Suspend mode can set from the register
        * Page - page0
        * register - 0x3E
        * bit positions - 0 and 1*/
        power_mode = BNO055_POWER_MODE_SUSPEND;

        /* set the power mode as SUSPEND*/
        comres += bno055_set_power_mode(power_mode);

        if(i2c_instance != nullptr)
        {
            delete i2c_instance;
            i2c_instance = nullptr;
        }
        /*---------------------------------------------------------------------*
        ************************* END DE-INITIALIZATION **********************
        *---------------------------------------------------------------------*/
    };

    void CImu::serialCallbackIMUcommand(char const * a, char * b) {
        uint8_t l_isActivate=0;
        uint8_t l_res = sscanf(a,"%hhu",&l_isActivate);

        if(1 == l_res){
            if(uint8_globalsV_value_of_kl == 15 || uint8_globalsV_value_of_kl == 30)
            {
                m_isActive=(l_isActivate>=1);
                bool_globalsV_imu_isActive = (l_isActivate>=1);
                sprintf(b,"1");
            }
            else{
                sprintf(b,"kl 15/30 is required!!");
            }
            
        }else{
            sprintf(b,"syntax error");
        }
    }

    s8 CImu::BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
    {
        s32 BNO055_iERROR = BNO055_INIT_VALUE;
        u8 array[I2C_BUFFER_LEN];
        u8 stringpos = BNO055_INIT_VALUE;

        array[BNO055_INIT_VALUE] = reg_addr;
        for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
        {
            array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
        }

        if (i2c_instance->write(dev_addr, (const char*)array, cnt + 1) == 0)
        {
            BNO055_iERROR = BNO055_SUCCESS; // Return success (0)
        }
        else
        {
            BNO055_iERROR = BNO055_ERROR; // Return error (-1)
        }

        return (s8)BNO055_iERROR;
    }

    s8 CImu::BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
    {
        s32 BNO055_iERROR = BNO055_INIT_VALUE;
        u8 array[I2C_BUFFER_LEN] = { BNO055_INIT_VALUE };
        u8 stringpos = BNO055_INIT_VALUE;

        array[BNO055_INIT_VALUE] = reg_addr;

        /* Please take the below API as your reference
        * for read the data using I2C communication
        * add your I2C read API here.
        * "BNO055_iERROR = I2C_WRITE_READ_STRING(DEV_ADDR,
        * ARRAY, ARRAY, 1, CNT)"
        * BNO055_iERROR is an return value of SPI write API
        * Please select your valid return value
        * In the driver BNO055_SUCCESS defined as 0
        * and FAILURE defined as -1
        */
        for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
        {
            *(reg_data + stringpos) = array[stringpos];
        }

        // Write the register address to set the pointer for reading
        if (i2c_instance->write(dev_addr, (const char*)&reg_addr, 1) != 0)
        {
            BNO055_iERROR = BNO055_ERROR; // Return error (-1)
            return (s8)BNO055_iERROR;
        }

        // Read the data from the specified register address
        if (i2c_instance->read(dev_addr, (char*)reg_data, cnt) == 0)
        {
            BNO055_iERROR = BNO055_SUCCESS; // Return success (0)
        }
        else
        {
            BNO055_iERROR = BNO055_ERROR; // Return error (-1)
        }
        return (s8)BNO055_iERROR;
    }
    
    void CImu::I2C_routine(void)
    {
        bno055.bus_write = BNO055_I2C_bus_write;
        bno055.bus_read = BNO055_I2C_bus_read;
        bno055.delay_msec = BNO055_delay_msek;
        bno055.dev_addr = BNO055_I2C_ADDR2 << 1;
        // bno055.dev_addr = BNO055_I2C_ADDR1 << 1;

        ThisThread::sleep_for(chrono::milliseconds(300));
    }

    void CImu::BNO055_delay_msek(u32 msek)
    {
        /*Here you can write your own delay routine*/
        ThisThread::sleep_for(chrono::milliseconds(msek));
    }

    void CImu::predict(const Eigen::Vector2d& acceleration) {
        x_ = A_ * x_ + B_ * acceleration;
        P_ = A_ * P_ * A_.transpose() + Q_;
    }

    void CImu::update(const Eigen::Vector2d& position) {
        Eigen::Vector2d z = position;
        Eigen::Vector2d y = z - H_ * x_;
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(4, 4) - K * H_) * P_;
    }

    void CImu::_run() {
        /* Run method behaviour */
        if(!m_isActive) return;

        char buffer[_100_chars];
        s8 comres = BNO055_SUCCESS;

        // auto start = std::chrono::system_clock::now();
        //Inicialización de variables angulares y lectura de los valores brutos

        s16 s16_euler_h_raw = BNO055_INIT_VALUE;
        s16 s16_euler_p_raw = BNO055_INIT_VALUE;
        s16 s16_euler_r_raw = BNO055_INIT_VALUE;

        comres += bno055_read_euler_h(&s16_euler_h_raw);
        if(comres != BNO055_SUCCESS) return;

        comres += bno055_read_euler_p(&s16_euler_p_raw);
        if(comres != BNO055_SUCCESS) return;

        comres += bno055_read_euler_r(&s16_euler_r_raw);
        if(comres != BNO055_SUCCESS) return;

        //Inicializacion de variables de aceleración lineal y lectura de los valores brutos

        s16 s16_linear_accel_x_raw = BNO055_INIT_VALUE;
        s16 s16_linear_accel_y_raw = BNO055_INIT_VALUE;

        comres = bno055_read_linear_accel_x(&s16_linear_accel_x_raw);
        if(comres != BNO055_SUCCESS) return;
        
        comres = bno055_read_linear_accel_y(&s16_linear_accel_y_raw);
        if(comres != BNO055_SUCCESS) return;

        
        double accelx = ((double)s16_linear_accel_x_raw) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;
        double accely = ((double)s16_linear_accel_y_raw) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;
        accelx = (accelx > -0.1 && accelx < 0.1) ? 0.0 : accelx;
        accely = (accely > -0.1 && accely < 0.1) ? 0.0 : accely;
        
        Eigen::Vector2d acceleration(accelx, accely);
        Eigen::Vector2d position(x_(0), x_(2));

        if((-0.110 <= accelx && accelx <= 0.110) && (-0.110 <= accely && accely <= 0.110))
        {
            m_velocityX = 0;
            m_velocityY = 0;
            m_velocityStationaryCounter++;
        }
        else {
            m_velocityX += (accelx * dt);
            m_velocityY += (accely * dt);
            m_velocityStationaryCounter = 0;
        }

        predict(acceleration);
        update(position);

        double yaw =  ((double)s16_euler_h_raw/16.0) * M_PI / 180.0;

        // Convertir la posición del eje del auto a coordenadas globales
        double dx_relativo = x_(3); // Posición relativa en x
        double dy_relativo = x_(1); //Posicion relativa en y.

        double dx = dx_relativo * cos(yaw) - dy_relativo * sin(yaw);
        double dy = dx_relativo * sin(yaw) + dy_relativo * cos(yaw);
        
        // Obtener el valor del contador de RPM
        double rpmcounter = static_cast<double>(counter.read());

        if (m_messageSendCounter >= 20)
        {

            m_messageSendCounter = 0;
            snprintf(buffer, sizeof(buffer), "@imu:%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f;%d;;\r\n",  
            
                (float)(s16_euler_h_raw/16.0), 
                (float)(s16_euler_p_raw/16.0), 
                (float)(s16_euler_r_raw/16.0), 

                accelx,
                accely,
                dx,
                
                m_velocityX, 
                m_velocityY, 
                rpmcounter,
            
                x_(1), 
                x_(3), 
                brain::g_speed
            );
            m_serial.write(buffer,strlen(buffer));    
        }
        else
        {
            m_messageSendCounter++;
        }
        
    }

}; // namespace periodics

