/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
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
#include "imu.hpp"
#include <cmath>  // Para sin(), cos(), M_PI

#define M_PI 3.1415926535897932

#define _100_chars                      100
#define BNO055_EULER_DIV_DEG_int        16
#define BNO055_LINEAR_ACCEL_DIV_MSQ_int 100
#define precision_scaling_factor        100

#define MAX_NOISE 110 // old:MAX_NOISE

namespace periodics{
    /** \brief  Class constructor
     *
     *  It initializes the task and the state of the led. 
     *
     *  \param f_period       Toggling period of LED
     *  \param f_led          Digital output line to LED
     */

    /*--------------------------------------------------------------------------------------------------*
    *  Before initializiting with another value, the i2c_instance static pointer variable should be
    *  initialized with a nullptr.
    *---------------------------------------------------------------------------------------------------*/
    I2C* periodics::CImu::i2c_instance = nullptr;

    CImu::CImu(
            std::chrono::milliseconds    f_period, 
            UnbufferedSerial& f_serial,
            PinName SDA,
            PinName SCL)
        : utils::CTask(f_period)
        , m_isActive(false)
        , m_serial(f_serial)
        , m_velocityX(0)
        , m_velocityY(0)
        , m_velocityZ(0)
        , m_velocityStationaryCounter(0)
        , m_delta_time(f_period.count())
        , m_messageSendCounter(0)
        , posX(0)
        , posY(0)
        , posZ(0)
        , velX(0)
        , velY(0)
        , velZ(0)
    {


        P[0][0] = 0.1; P[0][1] = 0;
        P[1][0] = 0; P[1][1] = 0.1;

        Q[0][0] = 0.001; Q[0][1] = 0;
        Q[1][0] = 0; Q[1][1] = 0.003;  // Permitir cambios más suaves en posición y velocidad

        R = 10;  // Aumentado para confiar menos en las mediciones ruidosas


        

        // Inicialización de las variables de estado
        posX = 0;
        posY = 0;
        posZ = 0;
        velX = 0;
        velY = 0;
        velZ = 0;
        accelX = 0;
        accelY = 0;
        accelZ = 0;

        last_time = duration_cast<std::chrono::milliseconds>(Kernel::Clock::now().time_since_epoch()).count();

        if(m_delta_time < 10){
            setNewPeriod(10);
            m_delta_time = 10;
        }
        
        s32 comres = BNO055_ERROR;
        /* variable used to set the power mode of the sensor*/
        u8 power_mode = BNO055_INIT_VALUE;

        /*---------------------------------------------------------------------------*
        *********************** START INITIALIZATION ************************
        *--------------------------------------------------------------------------*/

        /*--------------------------------------------------------------------------------------------------*
        *  i2c_instance variable member will be initialized with the actual I2C of the target board.
        *---------------------------------------------------------------------------------------------------*/    
        
        printf("Starting IMU sensor data acquisition...\r\n");  
        i2c_instance = new I2C(SDA, SCL);
        i2c_instance->frequency(400000);

        ThisThread::sleep_for(chrono::milliseconds(300));

        /*  Based on the user need configure I2C interface.
        *  It is example code to explain how to use the bno055 API*/
        I2C_routine();

        /*--------------------------------------------------------------------------*
        *  This API used to assign the value/reference of
        *  the following parameters
        *  I2C address
        *  Bus Write
        *  Bus read
        *  Chip id
        *  Page id
        *  Accel revision id
        *  Mag revision id
        *  Gyro revision id
        *  Boot loader revision id
        *  Software revision id
        *-------------------------------------------------------------------------*/
        comres = bno055_init(&bno055);

        /*  For initializing the BNO sensor it is required to the operation mode
        * of the sensor as NORMAL
        * Normal mode can set from the register
        * Page - page0
        * register - 0x3E
        * bit positions - 0 and 1*/
        power_mode = BNO055_POWER_MODE_NORMAL;

        /* set the power mode as NORMAL*/
        comres += bno055_set_power_mode(power_mode);

        /************************* START READ RAW DATA ********
        * operation modes of the sensor
        * operation mode can set from the register
        * page - page0
        * register - 0x3D
        * bit - 0 to 3
        * for sensor data read following operation mode have to set
        * SENSOR MODE
        * 0x01 - BNO055_OPERATION_MODE_ACCONLY
        * 0x02 - BNO055_OPERATION_MODE_MAGONLY
        * 0x03 - BNO055_OPERATION_MODE_GYRONLY
        * 0x04 - BNO055_OPERATION_MODE_ACCMAG
        * 0x05 - BNO055_OPERATION_MODE_ACCGYRO
        * 0x06 - BNO055_OPERATION_MODE_MAGGYRO
        * 0x07 - BNO055_OPERATION_MODE_AMG
        * based on the user need configure the operation mode*/

        /************************* START READ RAW FUSION DATA ********
         * For reading fusion data it is required to set the
         * operation modes of the sensor
         * operation mode can set from the register
         * page - page0
         * register - 0x3D
         * bit - 0 to 3
         * for sensor data read following operation mode have to set
         * FUSION MODE
         * 0x08 - BNO055_OPERATION_MODE_IMUPLUS
         * 0x09 - BNO055_OPERATION_MODE_COMPASS
         * 0x0A - BNO055_OPERATION_MODE_M4G
         * 0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
         * 0x0C - BNO055_OPERATION_MODE_NDOF
         * based on the user need configure the operation mode*/
        comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

        // comres += bno055_set_accel_range(BNO055_ACCEL_RANGE_2G);

        /*----------------------------------------------------------------*
        ************************* END INITIALIZATION *************************
        *-----------------------------------------------------------------*/
        u8 euler_unit_u8 = BNO055_INIT_VALUE;

        /* Read the current Euler unit and set the
        * unit as degree if the unit is in radians */
        comres = bno055_get_euler_unit(&euler_unit_u8);
        if (euler_unit_u8 != BNO055_EULER_UNIT_DEG)
        {
            comres += bno055_set_euler_unit(BNO055_EULER_UNIT_DEG);
        }

    }

    /** @brief  CImu class destructor
     */
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

    /** \brief  Serial callback method to activate or deactivate the publisher. 
     * When the received integer value is bigger or equal to 1, then the publisher become 
     * active and send messages, otherwise is deactivated. 
     *
     * @param a                   input received string
     * @param b                   output reponse message
     * 
     */
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

    /**
    * \brief Writes data to the device over the I2C bus.
    * 
    * This function serves as a low-level I2C write routine tailored for the BNO055 sensor. 
    * It packages the register address and data to be written into a single buffer and then 
    * dispatches the write operation.
    * 
    * \param dev_addr   : I2C address of the BNO055 sensor.
    * \param reg_addr   : Address of the target register where the data needs to be written.
    * \param reg_data   : Pointer to an array containing the data bytes to be written to the sensor.
    * \param cnt        : Number of data bytes to write from the \a reg_data array.
    * 
    * \return BNO055_SUCCESS (0) on successful write operation.
    * \return BNO055_ERROR (-1) if the write operation encounters an error.
    * 
    * \note The actual data writing starts from the second position in the array, as the 
    *       first position is reserved for the register address.
    */
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

    /**
    * \brief Reads data from the device over the I2C bus.
    * 
    * This function facilitates reading data from a specific register of the BNO055 sensor 
    * over the I2C communication protocol. It sends the desired register address to the sensor, 
    * then reads back the requested amount of data bytes into a provided buffer.
    * 
    * \param dev_addr   : I2C address of the BNO055 sensor.
    * \param reg_addr   : Address of the target register from which the data needs to be read.
    * \param reg_data   : Pointer to an array where the read data bytes will be stored.
    * \param cnt        : Number of data bytes to read into the \a reg_data array.
    * 
    * \return BNO055_SUCCESS (0) on successful read operation.
    * \return BNO055_ERROR (-1) if the read operation encounters an error.
    * 
    * \note The function first writes the register address to the sensor to set the pointer 
    *       to the desired location and then initiates the I2C read operation.
    */
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
    
    /*-------------------------------------------------------------------------*
     *  By using bno055 the following structure parameter can be accessed
     *  Bus write function pointer: BNO055_WR_FUNC_PTR
     *  Bus read function pointer: BNO055_RD_FUNC_PTR
     *  Delay function pointer: delay_msec
     *  I2C address: dev_addr
     *--------------------------------------------------------------------------*/
    void CImu::I2C_routine(void)
    {
        bno055.bus_write = BNO055_I2C_bus_write;
        bno055.bus_read = BNO055_I2C_bus_read;
        bno055.delay_msec = BNO055_delay_msek;
        bno055.dev_addr = BNO055_I2C_ADDR2 << 1;
        // bno055.dev_addr = BNO055_I2C_ADDR1 << 1;

        ThisThread::sleep_for(chrono::milliseconds(300));
    }

    /**
    * \brief Introduces a delay for the specified duration in milliseconds.
    * 
    * This function relies on the `ThisThread::sleep_for` method to create 
    * a delay, making the current thread sleep for the specified duration.
    * 
    * \param msek The delay duration in milliseconds.
    */
    void CImu::BNO055_delay_msek(u32 msek)
    {
        /*Here you can write your own delay routine*/
        ThisThread::sleep_for(chrono::milliseconds(msek));
    }

    /** 
    * \brief  Periodically retrieves and processes IMU sensor values.
    * 
    * This method is invoked periodically and handles:
    * 1. Reading Euler angles (roll, pitch, yaw) from the BNO055 sensor.
    * 2. Reading linear acceleration values in the x, y, and z axes from the sensor.
    * 3. Based on the linear acceleration, it updates the current velocity of the device.
    * 4. If the device appears to be stationary (based on x and y acceleration thresholds), 
    *    a counter is incremented. If the device remains stationary for a certain duration 
    *    (15 cycles in this case), the velocity is reset.
    * 5. Formats and sends the acquired data over a serial connection.
    * 
    * \note If there are any issues reading from the BNO055 sensor, the method will exit early without sending data.
    */

    /* Predicción del estado usando Kalman */
    void CImu::predict() {
        dt = m_delta_time;  
    
        // Definir el umbral de ruido (ajustar según el sensor)
        float noise_threshold = 0.11;  // En m/s² (ajustar según pruebas)
    
        // Aplicar el umbral de ruido a las aceleraciones
        accelX_corrected = fabs(accelX) < noise_threshold ? 0.0 : accelX;
        accelY_corrected = fabs(accelY) < noise_threshold ? 0.0 : accelY;
        accelZ_corrected = fabs(accelZ) < noise_threshold ? 0.0 : accelZ;
    
        // Filtro de suavizado exponencial (reduce fluctuaciones)
        float alpha_accel = 0.8;
        accelX_corrected = alpha_accel * accelX_corrected + (1 - alpha_accel) * accelX;
        accelY_corrected = alpha_accel * accelY_corrected + (1 - alpha_accel) * accelY;
        accelZ_corrected = alpha_accel * accelZ_corrected + (1 - alpha_accel) * accelZ;
    
        // Integración RK2 para velocidad
        float k1_vx = accelX_corrected * dt;
        float k1_vy = accelY_corrected * dt;
        float k1_vz = accelZ_corrected * dt;
    
        float k2_vx = (accelX_corrected + k1_vx / 2.0f) * dt;
        float k2_vy = (accelY_corrected + k1_vy / 2.0f) * dt;
        float k2_vz = (accelZ_corrected + k1_vz / 2.0f) * dt;
    
        float alpha_vel = 0.95;
        float max_speed = 5.0;
    
        velX = alpha_vel * velX + (1 - alpha_vel) * k2_vx;
        velY = alpha_vel * velY + (1 - alpha_vel) * k2_vy;
        velZ = alpha_vel * velZ + (1 - alpha_vel) * k2_vz;
    
        // Aplicar lógica de cálculo de velocidad
        if ((-0.1 <= accelX_corrected && accelX_corrected <= .1) && (-.1 <= accelY_corrected && accelY_corrected <= .1))
        {
            velX += 0 * dt; // Δt = dt
            velY += 0 * dt;
            velZ += 0 * dt;
            m_velocityStationaryCounter += 1;
            if (m_velocityStationaryCounter == 10)
            {
                velX = 0;
                velY = 0;
                velZ = 0;
                m_velocityStationaryCounter = 0;
            }
        }
        else
        {
            velX += (accelX_corrected * (uint16_t)dt) / 1000; // Δt = dt
            velY += (accelY_corrected * (uint16_t)dt) / 1000;
            velZ += (accelZ_corrected * (uint16_t)dt) / 1000;
            m_velocityStationaryCounter = 0;
        }
    
        // Limitar velocidad
        velX = fmax(fmin(velX, max_speed), -max_speed);
        velY = fmax(fmin(velY, max_speed), -max_speed);
        velZ = fmax(fmin(velZ, max_speed), -max_speed);
    
        // Integración RK2 para posición
        float k1_px = velX * dt + 0.5 * accelX_corrected * dt * dt;
        float k1_py = velY * dt + 0.5 * accelY_corrected * dt * dt;
        float k1_pz = velZ * dt + 0.5 * accelZ_corrected * dt * dt;
    
        float k2_px = (velX + k1_vx / 2.0f) * dt + 0.5 * accelX_corrected * dt * dt;
        float k2_py = (velY + k1_vy / 2.0f) * dt + 0.5 * accelY_corrected * dt * dt;
        float k2_pz = (velZ + k1_vz / 2.0f) * dt + 0.5 * accelZ_corrected * dt * dt;
    
        posX += k2_px;
        posY += k2_py;
        posZ += k2_pz;
    
        // Actualización de covarianza con factor de amortiguación
        float dampening_factor = 0.99;
        P[0][0] = dampening_factor * (P[0][0] + Q[0][0]);
        P[1][1] = dampening_factor * (P[1][1] + Q[1][1]);
    }
    
    /* Actualización con mediciones */
    void CImu::update(s32 newAccelX, s32 newAccelY, s32 newAccelZ) {
        // Escalado de las variables para el filtro de Kalman
        accelX = (float)newAccelX / precision_scaling_factor;
        accelY = (float)newAccelY / precision_scaling_factor;
        accelZ = (float)newAccelZ / precision_scaling_factor;
    
        // **Filtro de ruido**: eliminar valores por debajo del umbral
        float noise_threshold = 0.02; // Ajustar según el sensor
        accelX = (fabs(accelX) < noise_threshold) ? 0.0 : accelX;
        accelY = (fabs(accelY) < noise_threshold) ? 0.0 : accelY;
        accelZ = (fabs(accelZ) < noise_threshold) ? 0.0 : accelZ;
    
        // **Suavizado de la aceleración (Media Móvil)**
        float alpha = 0.95;  // Suavizado extremo
        accelX = alpha * accelX + (1 - alpha) * prevAccelX;
        accelY = alpha * accelY + (1 - alpha) * prevAccelY;
        accelZ = alpha * accelZ + (1 - alpha) * prevAccelZ;
    
        prevAccelX = accelX;
        prevAccelY = accelY;
        prevAccelZ = accelZ;
    
        // **Reducir la confianza en la medición**
        float R_mod = R * 2.0; // Aumentar R para confiar menos en mediciones ruidosas
    
        // **Ganancia de Kalman**: K = P * H^T / (H * P * H^T + R)
        float S = P[0][0] + R_mod;
        float K[2] = { static_cast<float>(P[0][0] / S), static_cast<float>(P[1][0] / (S * 10.0)) };  // Reducir impacto en velocidad
    
        // **Innovación (diferencia entre medición y predicción)**
        float innovationX = accelX - posX;
        float innovationY = accelY - posY;
        float innovationZ = accelZ - posZ;
    
        // **Limitar la variación máxima para suavizar**
        float max_variation = 0.05;  // Más bajo para eliminar ruido
        innovationX = fmax(fmin(innovationX, max_variation), -max_variation);
        innovationY = fmax(fmin(innovationY, max_variation), -max_variation);
        innovationZ = fmax(fmin(innovationZ, max_variation), -max_variation);
    
        // **Corrección del estado**
        posX += K[0] * innovationX;
        velX += K[1] * innovationX;
    
        posY += K[0] * innovationY;
        velY += K[1] * innovationY;
    
        posZ += K[0] * innovationZ;
        velZ += K[1] * innovationZ;
    
        // **Reducir más el impacto de la actualización en velocidad**
        velX = 0.9 * velX + 0.1 * prevVelX;
        velY = 0.9 * velY + 0.1 * prevVelY;
        velZ = 0.9 * velZ + 0.1 * prevVelZ;
    
        prevVelX = velX;
        prevVelY = velY;
        prevVelZ = velZ;
    
        // **Actualizar la covarianza del error**: P = (I - K * H) * P
        P[0][0] *= (1 - K[0]);
        P[0][1] *= (1 - K[0]);
        P[1][0] *= (1 - K[1]);
        P[1][1] *= (1 - K[1]);
    }
    
    
    /* Método principal de ejecución */
    void CImu::_run() {

        if(!m_isActive) return;

        char buffer[_100_chars];
        s8 comres = BNO055_SUCCESS;

        s16 s16_euler_h_raw = BNO055_INIT_VALUE;
        s16 s16_euler_p_raw = BNO055_INIT_VALUE;
        s16 s16_euler_r_raw = BNO055_INIT_VALUE;

        s16 s16_linear_accel_x_raw = BNO055_INIT_VALUE;
        s16 s16_linear_accel_y_raw = BNO055_INIT_VALUE;
        s16 s16_linear_accel_z_raw = BNO055_INIT_VALUE;

        s16 s16_gravity_x_raw = BNO055_INIT_VALUE;
        s16 s16_gravity_y_raw = BNO055_INIT_VALUE;
        s16 s16_gravity_z_raw = BNO055_INIT_VALUE;

        comres += bno055_read_euler_h(&s16_euler_h_raw);
        if(comres != BNO055_SUCCESS) return;

        comres += bno055_read_euler_p(&s16_euler_p_raw);
        if(comres != BNO055_SUCCESS) return;

        comres += bno055_read_euler_r(&s16_euler_r_raw);
        if(comres != BNO055_SUCCESS) return;

        s32 s16_euler_h_deg = (s16_euler_h_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int;
        s32 s16_euler_p_deg = (s16_euler_p_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int;
        s32 s16_euler_r_deg = (s16_euler_r_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int;

        s16 s16_euler_h_rad = (s16_euler_h_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int * (M_PI/180);
        s16 s16_euler_p_rad = (s16_euler_p_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int * (M_PI/180);
        s16 s16_euler_r_rad = (s16_euler_r_raw * precision_scaling_factor) / BNO055_EULER_DIV_DEG_int * (M_PI/180);

        comres = bno055_read_linear_accel_x(&s16_linear_accel_x_raw);
        if(comres != BNO055_SUCCESS) return;
        
        comres = bno055_read_linear_accel_y(&s16_linear_accel_y_raw);
        if(comres != BNO055_SUCCESS) return;

        comres = bno055_read_linear_accel_z(&s16_linear_accel_z_raw);
        if(comres != BNO055_SUCCESS) return;

        s32 s16_linear_accel_x_msq = (s16_linear_accel_x_raw * precision_scaling_factor) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;
        s32 s16_linear_accel_y_msq = (s16_linear_accel_y_raw * precision_scaling_factor) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;
        s32 s16_linear_accel_z_msq = (s16_linear_accel_z_raw * precision_scaling_factor) / BNO055_LINEAR_ACCEL_DIV_MSQ_int;

        // Aplicar filtro de media móvil a las aceleraciones
        accelXSum -= accelXBuffer[accelXIndex];
        accelYSum -= accelYBuffer[accelYIndex];
        accelZSum -= accelZBuffer[accelZIndex];

        accelXBuffer[accelXIndex] = s16_linear_accel_x_msq;
        accelYBuffer[accelYIndex] = s16_linear_accel_y_msq;
        accelZBuffer[accelZIndex] = s16_linear_accel_z_msq;

        accelXSum += accelXBuffer[accelXIndex];
        accelYSum += accelYBuffer[accelYIndex];
        accelZSum += accelZBuffer[accelZIndex];

        accelXIndex = (accelXIndex + 1) % FILTER_SIZE;
        accelYIndex = (accelYIndex + 1) % FILTER_SIZE;
        accelZIndex = (accelZIndex + 1) % FILTER_SIZE;

        s16_linear_accel_x_msq = accelXSum / FILTER_SIZE;
        s16_linear_accel_y_msq = accelYSum / FILTER_SIZE;
        s16_linear_accel_z_msq = accelZSum / FILTER_SIZE;

        predict();
        update(s16_linear_accel_x_msq, s16_linear_accel_y_msq, s16_linear_accel_z_msq);

        m_accelX = accelX * precision_scaling_factor;
        m_accelY = accelY * precision_scaling_factor;
        m_accelZ = accelZ * precision_scaling_factor;

        m_velocityX = velX * precision_scaling_factor;
        m_velocityY = velY * precision_scaling_factor;
        m_velocityZ = velZ * precision_scaling_factor;

        m_accelX = (abs( (((float)s16_linear_accel_x_msq)/precision_scaling_factor) ) < 0.011) ? 0 : (((float)s16_linear_accel_x_msq)/precision_scaling_factor);
        m_accelY = (abs( (((float)s16_linear_accel_y_msq)/precision_scaling_factor) ) < 0.011) ? 0 : (((float)s16_linear_accel_y_msq)/precision_scaling_factor);
        m_accelZ = (abs( (((float)s16_linear_accel_z_msq)/precision_scaling_factor) ) < 0.011) ? 0 : (((float)s16_linear_accel_z_msq)/precision_scaling_factor);

        m_delta_time = 0.01;

        if((-0.2 <= (((float)s16_linear_accel_x_msq)/precision_scaling_factor) && (((float)s16_linear_accel_x_msq)/precision_scaling_factor) <= 0.2) 
        && (-0.2 <= (((float)s16_linear_accel_y_msq)/precision_scaling_factor) && (((float)s16_linear_accel_y_msq)/precision_scaling_factor) <= 0.2))
        {
            m_velocityX += 0 * m_delta_time; // Δt = m_delta_time
            m_velocityY += 0 * m_delta_time;
            m_velocityZ += 0 * m_delta_time;
            m_velocityStationaryCounter += 1;
            if (m_velocityStationaryCounter == 10)
            {
                m_velocityX = 0;
                m_velocityY = 0;
                m_velocityZ = 0;
                m_velocityStationaryCounter = 0;
            }
            
        }
        else{
            m_velocityX += (m_accelX * m_delta_time) ; // Δt = m_delta_time = 10ms
            m_velocityY += (m_accelY * m_delta_time) ;
            m_velocityZ += (m_accelZ * m_delta_time) ;
            m_velocityStationaryCounter = 0;
        }
        //MRUV
        m_positionX = m_positionX + m_velocityX * m_delta_time + 0.5 * m_accelX * m_delta_time * m_delta_time ;
        m_positionY = m_positionY + m_velocityY * m_delta_time + 0.5 * m_accelY * m_delta_time * m_delta_time ;
        m_positionZ = m_positionZ + m_velocityZ * m_delta_time + 0.5 * m_accelZ * m_delta_time * m_delta_time ;

        // MRU
        // m_positionX = m_positionX + m_velocityX * m_delta_time ;
        // m_positionY = m_positionY + m_velocityY * m_delta_time ;
        // m_positionZ = m_positionZ + m_velocityZ * m_delta_time ;

        // printf("AccelX: %d, AccelY: %d, AccelZ: %d\n", m_accelX, m_accelY, m_accelZ);

        if (m_messageSendCounter >= 15)
        {
            m_messageSendCounter = 0;
            snprintf(buffer, sizeof(buffer), 
                "@imu:%d.%03d;%d.%03d;%d.%03d;%d.%03d;%d.%03d;%d.%03d;%d.%03d;%d.%03d;%d.%03d;%d.%03d;%d.%03d;%d.%03d;;\r\n",
    
                s16_euler_r_deg / 1000, abs(s16_euler_r_deg % 1000),
                s16_euler_p_deg / 1000, abs(s16_euler_p_deg % 1000),
                s16_euler_h_deg / 1000, abs(s16_euler_h_deg % 1000),
                
                m_accelX / 1000, abs(m_accelX % 1000),
                m_accelY / 1000, abs(m_accelY % 1000),
                m_accelZ / 1000, abs(m_accelZ % 1000),

                m_velocityX / 1000, abs(m_velocityX % 1000),
                m_velocityY / 1000, abs(m_velocityY % 1000),
                m_velocityZ / 1000, abs(m_velocityZ % 1000),

                m_positionX / 1000, abs(m_positionX % 1000),
                m_positionY / 1000, abs(m_positionY % 1000),
                m_positionZ / 1000, abs(m_positionZ % 1000)
            );
    
            m_serial.write(buffer,strlen(buffer));
        }
        else
        {
            m_messageSendCounter++;
        }
    }


}; // namespace periodics